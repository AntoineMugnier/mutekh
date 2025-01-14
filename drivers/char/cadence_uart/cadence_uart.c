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

    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2013

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

#include "cadence_uart_regs.h"

/**************************************************************/

#if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);
#endif

DRIVER_PV(struct cadence_uart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t		read_q;
  dev_request_queue_root_t		write_q;
#if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
# endif
#endif
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif
  uint32_t                      mode;

  bool_t                        read_started:1;
  bool_t                        write_started:1;
});

static void cadence_uart_try_read(struct device_s *dev)
{
  struct cadence_uart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_rq_head(&pv->read_q)))
    {
      size_t size = 0;

#if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      /* try to read more characters directly from device fifo */
      while ( size < rq->size && !(cpu_mem_read_32(pv->addr + CADENCE_UART_STS_ADDR)
                                   & endian_le32(CADENCE_UART_STS_REMPTY)) )
        {
          rq->data[size++] = endian_le32(cpu_mem_read_32(pv->addr + CADENCE_UART_FIFO_ADDR));
        }

      if (size)
        {
          rq->size -= size;
          rq->error = 0;
          rq->data += size;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_char_rq_pop(&pv->read_q);
              dev_char_rq_done(rq);
              continue;
            }
        }

#ifdef CONFIG_DEVICE_IRQ
      /* more data will be available on next interrupt */
      return;
#endif
    }

  pv->read_started = 0;

#if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
  /* copy more data from device fifo to driver fifo if no request currently need it */
  while (!(cpu_mem_read_32(pv->addr + CADENCE_UART_STS_ADDR)
           & endian_le32(CADENCE_UART_STS_REMPTY)))
    {
      uart_fifo_pushback(&pv->read_fifo, endian_le32(cpu_mem_read_32(pv->addr + CADENCE_UART_FIFO_ADDR)));
    }
#endif
}

static void cadence_uart_try_write(struct device_s *dev)
{
  struct cadence_uart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
  /* try to write data from driver fifo to device */
  while (!uart_fifo_isempty(&pv->write_fifo) &&
         !(cpu_mem_read_32(pv->addr + CADENCE_UART_STS_ADDR)
           & endian_le32(CADENCE_UART_STS_TFUL)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + CADENCE_UART_FIFO_ADDR, endian_le32(c));
    }
#endif

  while ((rq = dev_char_rq_head(&pv->write_q)))
    {
      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          while (size < rq->size)
            {
              if (cpu_mem_read_32(pv->addr + CADENCE_UART_STS_ADDR)
                   & endian_le32(CADENCE_UART_STS_TFUL))
                break;

              cpu_mem_write_32(pv->addr + CADENCE_UART_FIFO_ADDR, endian_le32(rq->data[size++]));
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
        }

      /* some characters were not written to the device fifo, push to driver fifo */
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
              dev_char_rq_pop(&pv->write_q);
              dev_char_rq_done(rq);
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

#define cadence_uart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(cadence_uart_request)
{
  struct device_s               *dev = accessor->dev;
  struct cadence_uart_context_s	*pv = dev->drv_pv;
  error_t err = 0;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ: {
      dev_char_rq_pushback(&pv->read_q, rq);
      if (!pv->read_started)
        {
          pv->read_started = 1;
          cadence_uart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_char_rq_pushback(&pv->write_q, rq);
      if (!pv->write_started)
        {
          pv->write_started = 1;
          cadence_uart_try_write(dev);
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
      dev_char_rq_done(rq);
    }
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(cadence_uart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct cadence_uart_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ir = cpu_mem_read_32(pv->addr + CADENCE_UART_INT_STS_ADDR);

      if (!ir)
        break;

      if (ir & endian_le32(CADENCE_UART_INT_STS_TEMPTY))
        cadence_uart_try_write(dev);

      if (ir & endian_le32(CADENCE_UART_INT_STS_RFUL | CADENCE_UART_INT_STS_TIMEOUT))
        cadence_uart_try_read(dev);

      cpu_mem_write_32(pv->addr + CADENCE_UART_INT_STS_ADDR, ir);
    }

  lock_release(&dev->lock);
}

#endif


#define cadence_uart_use dev_use_generic

/* Set divider value, update pv->mode and return actual divider. Returns 0 on error. */
static uint32_t cadence_uart_set_divider(struct cadence_uart_context_s *pv, uint32_t divisor)
{
  uint32_t n, bdiv = 1, min;

  if (divisor % 8 == 0)
    {
      pv->mode |= CADENCE_UART_MODE_CLKS_UARTCLK_8;
      divisor /= 8;
    }
  else
    {
      pv->mode &= ~CADENCE_UART_MODE_CLKS_UARTCLK_8;
    }

  /* find best largest bdiv */
  for (min = n = 256; n > 5; n--)
    {
      uint32_t m = divisor % n;
      if (m < min)
        {
          bdiv = n;
          if (m == 0)
            break;
          min = m;
        }
    }
  divisor /= bdiv;

  if (divisor < 1 || divisor > 65535)
    return 0;

  cpu_mem_write_32(pv->addr + CADENCE_UART_RATE_GEN_ADDR, endian_le32(divisor));
  cpu_mem_write_32(pv->addr + CADENCE_UART_RATE_DIVIDER_ADDR, endian_le32(bdiv - 1));

  if (pv->mode & CADENCE_UART_MODE_CLKS_UARTCLK_8)
    divisor *= 8;
  return divisor * bdiv;
}

static DEV_INIT(cadence_uart_init)
{
  struct cadence_uart_context_s	*pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  pv->read_started = pv->write_started = 0;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  cpu_mem_write_32(pv->addr + CADENCE_UART_EN_ADDR, 0);
  cpu_mem_write_32(pv->addr + CADENCE_UART_DIS_ADDR, 0);

  /* disable and reset the uart */
  cpu_mem_write_32(pv->addr + CADENCE_UART_CONTROL_ADDR,
    endian_le32(0x00000128 | CADENCE_UART_CONTROL_TXRES | CADENCE_UART_CONTROL_RXRES));

  /* wait for reset done */
  while (cpu_mem_read_32(pv->addr + CADENCE_UART_CONTROL_ADDR) &
         endian_le32(CADENCE_UART_CONTROL_TXRES | CADENCE_UART_CONTROL_RXRES))
    ;

  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);

#if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &cadence_uart_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_fifo;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
  cpu_mem_write_32(pv->addr + CADENCE_UART_RX_TIMEOUT_ADDR, endian_le32(255));

  cpu_mem_write_32(pv->addr + CADENCE_UART_EN_ADDR,
    endian_le32(CADENCE_UART_EN_TEMPTY | CADENCE_UART_EN_RFUL | CADENCE_UART_EN_TIMEOUT));
  cpu_mem_write_32(pv->addr + CADENCE_UART_INT_STS_ADDR, 0xffffffff);
#else
  cpu_mem_write_32(pv->addr + CADENCE_UART_RX_TIMEOUT_ADDR, 0);
#endif

  /* configure uart */
  pv->mode = (CADENCE_UART_MODE_CHRL(CADENCE_UART_MODE_CHRL_8BITS) |
              CADENCE_UART_MODE_PAR(CADENCE_UART_MODE_PAR_NONE) |
              CADENCE_UART_MODE_NBSTOP(CADENCE_UART_MODE_NBSTOP_1_BIT) |
              CADENCE_UART_MODE_CHMODE(CADENCE_UART_MODE_CHMODE_NORMAL));

  uintptr_t divisor;
  if (!device_get_param_uint(dev, "divisor", &divisor))
    {
      if (!cadence_uart_set_divider(pv, divisor))
        printk("cadence_uart: bad divisor value %u, not updated", divisor);
    }
  else
    {
      pv->mode |= endian_le32(cpu_mem_read_32(pv->addr + CADENCE_UART_MODE_ADDR))
                & CADENCE_UART_MODE_CLKS_UARTCLK_8;
    }

  cpu_mem_write_32(pv->addr + CADENCE_UART_MODE_ADDR, endian_le32(pv->mode));

  /* enable the uart */
  cpu_mem_write_32(pv->addr + CADENCE_UART_CONTROL_ADDR,
                   endian_le32(CADENCE_UART_CONTROL_STPBRK |
                               CADENCE_UART_CONTROL_TXEN |
                               CADENCE_UART_CONTROL_RXEN));


  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(cadence_uart_cleanup)
{
  struct cadence_uart_context_s	*pv = dev->drv_pv;

  if (pv->read_started || pv->write_started)
    return -EBUSY;

#ifdef CONFIG_DEVICE_IRQ
  /* disable irqs */
  cpu_mem_write_32(pv->addr + CADENCE_UART_EN_ADDR, 0);
#endif

  /* disable the uart */
  cpu_mem_write_32(pv->addr + CADENCE_UART_CONTROL_ADDR, endian_le32(0x00000128));

#if CONFIG_DRIVER_CHAR_CADENCE_UART_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
# endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(cadence_uart_drv, 0, "Cadence UART", cadence_uart,
               DRIVER_CHAR_METHODS(cadence_uart));

DRIVER_REGISTER(cadence_uart_drv,
                DEV_ENUM_FDTNAME_ENTRY("cadence_uart"));

