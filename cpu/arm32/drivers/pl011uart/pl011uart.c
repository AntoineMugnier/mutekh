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

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/iomux.h>

#include "pl011uart_regs.h"

/**************************************************************/

#if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_CHAR_PL011_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);
#endif

struct pl011uart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t	read_q;
  dev_request_queue_root_t	write_q;
#if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
# endif
#endif

#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif

  bool_t                        BITFIELD(read_started,1);
  bool_t                        BITFIELD(write_started,1);
};

static bool_t pl011uart_try_read(struct device_s *dev)
{
  struct pl011uart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;
  bool_t ack_done = 0;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
    {
      size_t size = 0;

#if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      /* try to read more characters directly from device fifo */
      while ( size < rq->size && !(cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
                                   & endian_le32(PL011_FR_RXFE)) )
        {
          rq->data[size++] = PL011_DR_DATA_GET(endian_le32(cpu_mem_read_32(pv->addr + PL011_DR_ADDR)));
          ack_done = 1;
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
      return ack_done;
#endif
    }

  pv->read_started = 0;

#if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
  /* copy more data from device fifo to driver fifo if no request currently need it */
  while (!(cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
           & endian_le32(PL011_FR_RXFE)))
    {
      uart_fifo_pushback(&pv->read_fifo, PL011_DR_DATA_GET(endian_le32(cpu_mem_read_32(pv->addr + PL011_DR_ADDR))));
      ack_done = 1;
    }
#endif

  return ack_done;
}

static void pl011uart_try_write(struct device_s *dev)
{
  struct pl011uart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
  /* try to write data from driver fifo to device */
  while (!uart_fifo_isempty(&pv->write_fifo) &&
         !(cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
           & endian_le32(PL011_FR_TXFF)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + PL011_DR_ADDR, endian_le32(PL011_DR_DATA(c)));
    }
#endif

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q))))
    {
      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          while (size < rq->size)
            {
              if (cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
                   & endian_le32(PL011_FR_TXFF))
                break;

              cpu_mem_write_32(pv->addr + PL011_DR_ADDR, endian_le32(PL011_DR_DATA(rq->data[size++])));
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
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

static DEV_CHAR_REQUEST(pl011uart_request)
{
  struct device_s               *dev = accessor->dev;
  struct pl011uart_context_s	*pv = dev->drv_pv;

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
          pl011uart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
      if (!pv->write_started)
        {
          pv->write_started = 1;
          pl011uart_try_write(dev);
        }
      break;
    }
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(pl011uart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pl011uart_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ir = cpu_mem_read_32(pv->addr + PL011_MIS_ADDR);

      if (!ir)
        break;

      if (ir & endian_le32(PL011_MIS_TXMIS))
        {
          pl011uart_try_write(dev);
          cpu_mem_write_32(pv->addr + PL011_ICR_ADDR, endian_le32(PL011_MIS_TXMIS));
        }

      if (ir & endian_le32(PL011_MIS_RXMIS))
        {
          if (!pl011uart_try_read(dev))
            /* discard 1 byte to acknowledge RX fifo level irq */
            cpu_mem_read_32(pv->addr + PL011_DR_ADDR);
        }
      else if (ir & endian_le32(PL011_MIS_RTMIS))
        {
          if (!pl011uart_try_read(dev))
            cpu_mem_write_32(pv->addr + PL011_ICR_ADDR, endian_le32(PL011_MIS_RTMIS));
        }
    }

  lock_release(&dev->lock);
}

#endif

static DEV_INIT(pl011uart_init);
static DEV_CLEANUP(pl011uart_cleanup);
#define pl011uart_use dev_use_generic

DRIVER_DECLARE(pl011uart_drv, 0, "PL011 UART", pl011uart,
               DRIVER_CHAR_METHODS(pl011uart));

DRIVER_REGISTER(pl011uart_drv,
                DEV_ENUM_FDTNAME_ENTRY("pl011"));

static DEV_INIT(pl011uart_init)
{
  struct pl011uart_context_s	*pv;
  device_mem_map( dev , 1 << 0 );

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_IOMUX
  /* setup pinmux */
  if (device_iomux_setup(dev, "<rx? >tx?", NULL, NULL, NULL))
    goto err_mem;
#endif

  pv->read_started = pv->write_started = 0;

  /* disable the uart */
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, 0);
  cpu_mem_write_32(pv->addr + PL011_DMACR_ADDR, 0);
  cpu_mem_write_32(pv->addr + PL011_LCRH_ADDR, 0);

  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

#if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &pl011uart_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_fifo;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
  cpu_mem_write_32(pv->addr + PL011_IMSC_ADDR, endian_le32(PL011_IMSC_RXIM | PL011_IMSC_TXIM | PL011_IMSC_RTIM));
#else
  /* disable irqs */
  cpu_mem_write_32(pv->addr + PL011_IMSC_ADDR, 0);
#endif

  /* configure uart */
  cpu_mem_write_32(pv->addr + PL011_IFLS_ADDR,
    endian_le32(PL011_IFLS_TXIFLSEL(PL011_IFLS_TXIFLSEL_1_8) |
                PL011_IFLS_RXIFLSEL(PL011_IFLS_RXIFLSEL_7_8)));

  cpu_mem_write_32(pv->addr + PL011_LCRH_ADDR,
    endian_le32(PL011_LCRH_FEN | PL011_LCRH_WLEN(PL011_LCRH_WLEN_8_BITS)));

  uintptr_t divisor;
  if (!device_get_param_uint(dev, "divisor", &divisor))
    {
      cpu_mem_write_32(pv->addr + PL011_IBRD_ADDR, endian_le32(divisor >> 6));
      cpu_mem_write_32(pv->addr + PL011_FBRD_ADDR, endian_le32(divisor & 0x3f));
    }

  /* enable the uart */
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(PL011_CR_TXE | PL011_CR_RXE | PL011_CR_UARTEN));

  dev->drv = &pl011uart_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(pl011uart_cleanup)
{
  struct pl011uart_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* disable irqs */
  cpu_mem_write_32(pv->addr + PL011_IMSC_ADDR, 0);
#endif
  /* disable the uart */
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, 0);

#if CONFIG_DRIVER_CHAR_PL011_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
#endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  mem_free(pv);
}
