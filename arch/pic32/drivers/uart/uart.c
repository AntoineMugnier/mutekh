/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#include <arch/pic32/freq.h>
#include <arch/pic32/uart.h>


#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);
#endif

struct pic32_uart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t		read_q;
  dev_request_queue_root_t		write_q;
#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
# endif
#endif
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s          irq_ep[2];
#endif
  uint32_t                      mode;

  uint32_t                      bauds;
  struct dev_freq_s             freq;

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s    clk_ep;
#endif

  bool_t                        read_started:1;
  bool_t                        write_started:1;
};

static void pic32_uart_char_update_bauds(struct pic32_uart_context_s *pv)
{
  uint32_t x = ((pv->freq.num/(pv->bauds * pv->freq.denom))/16) - 1;
  cpu_mem_write_32(pv->addr + PIC32_UART_BAUD_ADDR, x);
}


static void pic32_uart_try_read(struct device_s *dev)
{
  struct pic32_uart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
    {
      size_t size = 0;

#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif
      uint32_t status;

      /* try to read more characters directly from device buffer */
      while(1)
        {
          status = endian_le32(cpu_mem_read_32(pv->addr + PIC32_UART_STATUS_ADDR));

          if (size >= rq->size || !(status & endian_le32(PIC32_UART_STATUS_URXDA)))
            break;

          rq->data[size++] = endian_le32(cpu_mem_read_32(pv->addr + PIC32_UART_RX_ADDR));
        }

      if (size)
        {
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_request_queue_pop(&pv->read_q);
              kroutine_exec(&rq->base.kr);
              continue;
            }
        }
#ifdef CONFIG_DEVICE_IRQ 
#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO == 0
      /* Enable RX interrupt */
      device_irq_src_enable(&pv->irq_ep[0]);
#endif
      /* more data will be available on next interrupt */
      return;
#endif
    }

#if defined(CONFIG_DEVICE_IRQ) && (CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO == 0)
    /* Disable TX interrupt if no software fifo */
    device_irq_src_disable(&pv->irq_ep[0]);
#endif

  pv->read_started = 0;

#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  /* copy more data from device fifo to driver fifo if no request currently need it */
  while (cpu_mem_read_32(pv->addr + PIC32_UART_STATUS_ADDR)
           & endian_le32(PIC32_UART_STATUS_URXDA))
    uart_fifo_pushback(&pv->read_fifo, endian_le32(cpu_mem_read_32(pv->addr + PIC32_UART_RX_ADDR)));
#endif
}

static void pic32_uart_try_write(struct device_s *dev)
{
  struct pic32_uart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  /* try to write data from driver fifo to device */
  while (!uart_fifo_isempty(&pv->write_fifo) &&
        !(cpu_mem_read_32(pv->addr + PIC32_UART_STATUS_ADDR)
           & endian_le32(PIC32_UART_STATUS_UTXBF)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + PIC32_UART_TX_ADDR, endian_le32(c));
    }
#endif

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q))))
    {
      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          while(1)
            {
              uint32_t status = endian_le32(cpu_mem_read_32(pv->addr + PIC32_UART_STATUS_ADDR));

              if (size >= rq->size || (status & endian_le32(PIC32_UART_STATUS_UTXBF)))
                break;

              cpu_mem_write_32(pv->addr + PIC32_UART_TX_ADDR, endian_le32(rq->data[size++]));
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
        }

      /* some characters were not written to the device buffer, push to driver fifo */
      if (size < rq->size)
          size += uart_fifo_pushback_array(&pv->write_fifo, rq->data + size, rq->size - size);
#endif

      if (size)
        {
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_request_queue_pop(&pv->write_q);
              kroutine_exec(&rq->base.kr);
              continue;
            }
        }

      /* A request is on-going HW and SW fifo are full */
#ifdef CONFIG_DEVICE_IRQ
      /* Enable TX interrupt */
      device_irq_src_enable(&pv->irq_ep[1]);
      /* more fifo space will be available on next interrupt */
      return;
#endif
    }


  /* Request queue is empty. SW fifo may not be empty */
#ifdef CONFIG_DEVICE_IRQ
  if (uart_fifo_isempty(&pv->write_fifo))
    device_irq_src_disable(&pv->irq_ep[1]);
  else
    device_irq_src_enable(&pv->irq_ep[1]);
#endif

  pv->write_started = 0;
}

#define pic32_uart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(pic32_uart_request)
{
  struct device_s               *dev = accessor->dev;
  struct pic32_uart_context_s	*pv = dev->drv_pv;
  error_t err = 0;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ: {
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));
      if (!pv->read_started)
        {
          pv->read_started = 1;
          pic32_uart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
      if (!pv->write_started)
        {
          pv->write_started = 1;
          pic32_uart_try_write(dev);
        }
      break;
    }
    default:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      kroutine_exec(&rq->base.kr);
    }
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(pic32_uart_rx_irq)
{
  struct device_s *dev = ep->base.dev;

  lock_spin(&dev->lock);

  pic32_uart_try_read(dev);
  
  lock_release(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(pic32_uart_tx_irq)
{
  struct device_s *dev = ep->base.dev;

  lock_spin(&dev->lock);

  pic32_uart_try_write(dev);

  lock_release(&dev->lock);
}

#endif

static DEV_INIT(pic32_uart_char_init);
static DEV_CLEANUP(pic32_uart_char_cleanup);
#define pic32_uart_char_use dev_use_generic

DRIVER_DECLARE(pic32_uart_drv, 0, "PIC32 UART", pic32_uart_char,
               DRIVER_CHAR_METHODS(pic32_uart));

DRIVER_REGISTER(pic32_uart_drv);

static DEV_INIT(pic32_uart_char_init)
{
  struct pic32_uart_context_s	*pv;
  device_mem_map( dev , 1 << 0 );


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  pv->read_started = pv->write_started = 0;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* init software fifos */
  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

  /* disable and clear uart */
  cpu_mem_write_32(pv->addr + PIC32_UART_MODE_ADDR, 0);
  cpu_mem_write_32(pv->addr + PIC32_UART_STATUS_ADDR, 0);

  /* setup iomux */
  if (device_iomux_setup(dev, "<rx? >tx?", NULL, NULL, NULL))
    goto err_clk;

  /* configure baud rate. */
  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

  pv->bauds = CONFIG_DRIVER_PIC32_UART_PRINTK_BAUDRATE;
  pic32_uart_char_update_bauds(pv);

  cpu_mem_write_32(pv->addr + PIC32_UART_MODE_ADDR, PIC32_UART_MODE_ON);

  uint32_t x = PIC32_UART_STATUS_UTXEN | PIC32_UART_STATUS_URXEN | 
               PIC32_UART_STATUS_UTXISEL(2) | PIC32_UART_STATUS_URXISEL(0); 
  
  
  cpu_mem_write_32(pv->addr + PIC32_UART_STATUS_ADDR, x);

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif
  /* init irq endpoints */
  device_irq_source_init(dev, &pv->irq_ep[0], 1, &pic32_uart_rx_irq);
  device_irq_source_init(dev, &pv->irq_ep[1], 1, &pic32_uart_tx_irq);
#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  /* Enable RX irq */
  if (device_irq_source_link(dev, pv->irq_ep, 2, 1))
    goto err_fifo;
#else
  if (device_irq_source_link(dev, pv->irq_ep, 2, 0))
    goto err_fifo;
#endif
#endif


  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
#endif
 err_clk:
 err_clku:
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(pic32_uart_char_cleanup)
{
  struct pic32_uart_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* disable irqs */
  device_irq_source_unlink(dev, pv->irq_ep, 2);
#endif

  /* disable the uart */
  cpu_mem_write_32(pv->addr + PIC32_UART_MODE_ADDR, 0);
  cpu_mem_write_32(pv->addr + PIC32_UART_STATUS_ADDR, 0);

#if CONFIG_DRIVER_PIC32_UART_CHAR_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
# endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  mem_free(pv);
}

