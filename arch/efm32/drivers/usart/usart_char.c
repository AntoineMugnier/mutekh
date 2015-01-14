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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
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
#include <device/class/clock.h>

#include <arch/efm32_usart.h>

#if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);
#endif

struct efm32_usart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t		read_q;
  dev_request_queue_root_t		write_q;
#if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
# endif
#endif
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s           irq_ep[2];
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

static void efm32_usart_char_update_bauds(struct efm32_usart_context_s *pv)
{
  uint32_t div = (256 * pv->freq.num) / (4 * pv->bauds * pv->freq.denom) - 256;
  cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, endian_le32(div));
}


static void efm32_usart_try_read(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
    {
      size_t size = 0;

#if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      /* try to read more characters directly from device buffer */
      if ( size < rq->size && (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
                                   & endian_le32(EFM32_USART_STATUS_RXDATAV)) )
        {
          rq->data[size++] = endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_RXDATAX_ADDR));
        }

      if (size)
        {
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if (rq->type == DEV_CHAR_READ_PARTIAL || rq->size == 0)
            {
              dev_request_queue_pop(&pv->read_q);
              lock_release(&dev->lock);
              kroutine_exec(&rq->base.kr, 0);
              lock_spin(&dev->lock);
              continue;
            }
        }

#ifdef CONFIG_DEVICE_IRQ
      /* more data will be available on next interrupt */
      return;
#endif
    }

  pv->read_started = 0;

  /* if no request currently need incoming data, copy to driver fifo or discard. */
  if (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
           & endian_le32(EFM32_USART_STATUS_RXDATAV))
    {
      __unused__ uint32_t c = endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_RXDATAX_ADDR));
#if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
      uart_fifo_pushback(&pv->read_fifo, c);
#endif
    }
}

static void efm32_usart_try_write(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
  /* try to write data from driver fifo to device */
  if (!uart_fifo_isempty(&pv->write_fifo) &&
         (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
           & endian_le32(EFM32_USART_STATUS_TXBL)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + EFM32_USART_TXDATAX_ADDR, endian_le32(c));
    }
#endif

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q))))
    {
      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          if (size < rq->size && (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
                    & endian_le32(EFM32_USART_STATUS_TXBL)))
            {
              cpu_mem_write_32(pv->addr + EFM32_USART_TXDATAX_ADDR,
                               endian_le32(rq->data[size++]));
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
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

          if (rq->type == DEV_CHAR_WRITE_PARTIAL || rq->size == 0)
            {
              dev_request_queue_pop(&pv->write_q);
              lock_release(&dev->lock);
              kroutine_exec(&rq->base.kr, 0);
              lock_spin(&dev->lock);
              continue;
            }
        }

#ifdef CONFIG_DEVICE_IRQ
      /* more fifo space will be available on next interrupt */
      return;
#endif
    }

  pv->write_started = 0;
}

static DEV_CHAR_REQUEST(efm32_usart_request)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;

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
          efm32_usart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
      if (!pv->write_started)
        {
          pv->write_started = 1;
          efm32_usart_try_write(dev);
        }
      break;
    }
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_EP_PROCESS(efm32_usart_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ir = endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR));

      if (!(ir & (EFM32_USART_IF_TXC | EFM32_USART_IF_RXDATAV)))
        break;

      if (ir & EFM32_USART_IF_TXC)
        {
          cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR,
                           endian_le32(EFM32_USART_IFC_TXC));
          efm32_usart_try_write(dev);
        }

      if (ir & EFM32_USART_IF_RXDATAV)
        efm32_usart_try_read(dev);
    }

  lock_release(&dev->lock);
}

#endif

static const struct driver_char_s	efm32_usart_char_drv =
{
  .class_		= DRIVER_CLASS_CHAR,
  .f_request		= efm32_usart_request,
};

static DEV_INIT(efm32_usart_char_init);
static DEV_CLEANUP(efm32_usart_char_cleanup);

const struct driver_s	efm32_usart_drv =
{
  .desc                 = "EFM32 UART and USART (char)",
  .f_init		= efm32_usart_char_init,
  .f_cleanup		= efm32_usart_char_cleanup,
  .classes              = { &efm32_usart_char_drv, 0 }
};

REGISTER_DRIVER(efm32_usart_drv);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(efm32_usart_char_clk_changed)
{
  struct efm32_usart_context_s	*pv = ep->dev->drv_pv;
  pv->freq = *freq;
  efm32_usart_char_update_bauds(pv);
}
#endif

static DEV_INIT(efm32_usart_char_init)
{
  struct efm32_usart_context_s	*pv;
  device_mem_map( dev , 1 << 0 );

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  pv->read_started = pv->write_started = 0;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, &efm32_usart_char_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto err_mem;

  if (!DEV_FREQ_IS_VALID(ckinfo.freq))
    goto err_mem;
  pv->freq = ckinfo.freq;

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clku;
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;
#endif

  /* wait for current TX to complete */
  if (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
      & endian_le32(EFM32_USART_STATUS_TXENS))
      while (!(cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
               & endian_le32(EFM32_USART_STATUS_TXBL)))
        ;

  /* disable and clear the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));

  /* setup pinmux */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, "<rx? >tx?", loc, NULL, NULL))
    goto err_clk;

  uint32_t route = 0;
  if (loc[0] != IOMUX_INVALID_DEMUX)
    route |= EFM32_USART_ROUTE_RXPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    route |= EFM32_USART_ROUTE_TXPEN;

  if (route == 0)
    goto err_clk;

  EFM32_USART_ROUTE_LOCATION_SETVAL(route, loc[0] != IOMUX_INVALID_DEMUX ? loc[0] : loc[1]);

  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(route));

  /* init software fifos */
  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

#if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  /* init irq endpoints */
  device_irq_source_init(dev, pv->irq_ep, 2,
                         &efm32_usart_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 2, -1))
    goto err_fifo;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR,
                   endian_le32(EFM32_USART_IEN_TXC | EFM32_USART_IEN_RXDATAV));
#endif

  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR,
                   endian_le32(EFM32_USART_CTRL_SYNC(ASYNC) | EFM32_USART_CTRL_OVS(X4)));

  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR,
                   endian_le32(EFM32_USART_FRAME_DATABITS(EIGHT) |
                               EFM32_USART_FRAME_PARITY(NONE) |
                               EFM32_USART_FRAME_STOPBITS(ONE)));

  /* setup baud rate */
  pv->bauds = CONFIG_DRIVER_EFM32_USART_RATE;
  efm32_usart_char_update_bauds(pv);

  /* enable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXEN | EFM32_USART_CMD_TXEN | EFM32_USART_CMD_RXBLOCKDIS));

  dev->drv = &efm32_usart_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
#endif
 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(efm32_usart_char_cleanup)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* disable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 2);
#endif

  /* disable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

#if CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
# endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  mem_free(pv);
}

