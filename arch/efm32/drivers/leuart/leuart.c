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

#include <arch/efm32_leuart.h>

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_ring.h>

CONTAINER_TYPE(uart_fifo, RING, uint8_t, CONFIG_DRIVER_EFM32_LEUART_SWFIFO);
CONTAINER_FUNC(uart_fifo, RING, static inline, uart_fifo);
#endif

struct efm32_leuart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_char_queue_root_t		read_q;
  dev_char_queue_root_t		write_q;
#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
# endif
#endif
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s           irq_ep;
#endif
  uint32_t                      mode;
};

static void efm32_leuart_try_read(struct device_s *dev)
{
  struct efm32_leuart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_queue_head(&pv->read_q)))
    {
      size_t size = 0;

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      /* try to read more characters directly from device buffer */
      if ( size < rq->size && (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
                                   & endian_le32(EFM32_LEUART_STATUS_RXDATAV)) )
        {
          rq->data[size++] = endian_le32(cpu_mem_read_32(pv->addr + EFM32_LEUART_RXDATAX_ADDR));
        }

      if (size)
        {
          rq->size -= size;
          rq->error = 0;

          if (rq->callback(rq, size) || rq->size == 0)
            {
              dev_char_queue_remove(&pv->read_q, rq);
              continue;
            }

          rq->data += size;
        }

#ifdef CONFIG_DEVICE_IRQ
      /* more data will be available on next interrupt */
      return;
#endif
    }

  /* if no request currently need incoming data, copy to driver fifo or discard. */
  if (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
           & endian_le32(EFM32_LEUART_STATUS_RXDATAV))
    {
      uint32_t c = endian_le32(cpu_mem_read_32(pv->addr + EFM32_LEUART_RXDATAX_ADDR));
#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
      uart_fifo_pushback(&pv->read_fifo, c);
#endif
    }
}

static void efm32_leuart_try_write(struct device_s *dev)
{
  struct efm32_leuart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  /* try to write data from driver fifo to device */
  if (!uart_fifo_isempty(&pv->write_fifo) &&
         (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
           & endian_le32(EFM32_LEUART_STATUS_TXBL)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + EFM32_LEUART_TXDATAX_ADDR, endian_le32(c));
    }
#endif

  while ((rq = dev_char_queue_head(&pv->write_q)))
    {
      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          if (size < rq->size && (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
                    & endian_le32(EFM32_LEUART_STATUS_TXBL)))
            {
              cpu_mem_write_32(pv->addr + EFM32_LEUART_TXDATAX_ADDR,
                               endian_le32(rq->data[size++]));
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
        }

      /* some characters were not written to the device buffer, push to driver fifo */
      if (size < rq->size)
          size += uart_fifo_pushback_array(&pv->write_fifo, rq->data + size, rq->size - size);
#endif

      if (size)
        {
          rq->size -= size;
          rq->error = 0;

          if (rq->callback(rq, size) || rq->size == 0)
            {
              dev_char_queue_remove(&pv->write_q, rq);
              continue;
            }

          rq->data += size;
        }

#ifdef CONFIG_DEVICE_IRQ
      /* more fifo space will be available on next interrupt */
      return;
#endif
    }
}

static DEVCHAR_REQUEST(efm32_leuart_request)
{
  struct device_s               *dev = cdev->dev;
  struct efm32_leuart_context_s	*pv = dev->drv_pv;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ: {
#ifdef CONFIG_DEVICE_IRQ
      bool_t empty = dev_char_queue_isempty(&pv->read_q);
#endif
      dev_char_queue_pushback(&pv->read_q, rq);
#ifdef CONFIG_DEVICE_IRQ
      if (empty)
#endif
	efm32_leuart_try_read(dev);
      break;
    }

    case DEV_CHAR_WRITE: {
#ifdef CONFIG_DEVICE_IRQ
      bool_t empty = dev_char_queue_isempty(&pv->write_q);
#endif
      dev_char_queue_pushback(&pv->write_q, rq);
#ifdef CONFIG_DEVICE_IRQ
      if (empty)
#endif
	efm32_leuart_try_write(dev);
      break;
    }
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_EP_PROCESS(efm32_leuart_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_leuart_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ir = endian_le32(cpu_mem_read_32(pv->addr + EFM32_LEUART_IF_ADDR));

      if (!(ir & (EFM32_LEUART_IF_TXC | EFM32_LEUART_IF_RXDATAV)))
        break;

      if (ir & EFM32_LEUART_IF_TXC)
        {
          cpu_mem_write_32(pv->addr + EFM32_LEUART_IFC_ADDR,
                           endian_le32(EFM32_LEUART_IFC_TXC));
          efm32_leuart_try_write(dev);
        }

      if (ir & EFM32_LEUART_IF_RXDATAV)
        efm32_leuart_try_read(dev);
    }

  lock_release(&dev->lock);
}

#endif

static const struct driver_char_s	efm32_leuart_char_drv =
{
  .class_		= DRIVER_CLASS_CHAR,
  .f_request		= efm32_leuart_request,
};

static DEV_INIT(efm32_leuart_init);
static DEV_CLEANUP(efm32_leuart_cleanup);

const struct driver_s	efm32_leuart_drv =
{
  .desc                 = "EFM32 Low Energy UART",
  .f_init		= efm32_leuart_init,
  .f_cleanup		= efm32_leuart_cleanup,
  .classes              = { &efm32_leuart_char_drv, 0 }
};

REGISTER_DRIVER(efm32_leuart_drv);

static DEV_INIT(efm32_leuart_init)
{
  struct efm32_leuart_context_s	*pv;
  device_mem_map( dev , 1 << 0 );

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* wait for current TX to complete */
  if (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
      & endian_le32(EFM32_LEUART_STATUS_TXENS))
      while (!(cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
               & endian_le32(EFM32_LEUART_STATUS_TXBL)))
        ;

  /* disable and clear the uart */
  cpu_mem_write_32(pv->addr + EFM32_LEUART_CMD_ADDR,
                   endian_le32(EFM32_LEUART_CMD_RXDIS | EFM32_LEUART_CMD_TXDIS));

  cpu_mem_write_32(pv->addr + EFM32_LEUART_CMD_ADDR,
                   endian_le32(EFM32_LEUART_CMD_CLEARRX | EFM32_LEUART_CMD_CLEARTX));

  dev_char_queue_init(&pv->read_q);
  dev_char_queue_init(&pv->write_q);

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_leuart_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_fifo;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
  cpu_mem_write_32(pv->addr + EFM32_LEUART_IEN_ADDR,
                   endian_le32(EFM32_LEUART_IEN_TXC | EFM32_LEUART_IEN_RXDATAV));
#endif

  /* enable the uart */
  cpu_mem_write_32(pv->addr + EFM32_LEUART_CMD_ADDR,
                   endian_le32(EFM32_LEUART_CMD_RXEN | EFM32_LEUART_CMD_TXEN | EFM32_LEUART_CMD_RXBLOCKDIS));

  dev->drv = &efm32_leuart_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_char_queue_destroy(&pv->read_q);
  dev_char_queue_destroy(&pv->write_q);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(efm32_leuart_cleanup)
{
  struct efm32_leuart_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* disable irqs */
  cpu_mem_write_32(pv->addr + EFM32_LEUART_IEN_ADDR, 0);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif

  /* disable the uart */
  cpu_mem_write_32(pv->addr + EFM32_LEUART_CMD_ADDR,
                   endian_le32(EFM32_LEUART_CMD_RXDIS | EFM32_LEUART_CMD_TXDIS));

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
# endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_char_queue_destroy(&pv->read_q);
  dev_char_queue_destroy(&pv->write_q);

  mem_free(pv);
}
