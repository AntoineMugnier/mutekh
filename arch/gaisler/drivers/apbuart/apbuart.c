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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech

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

#include <gct_platform.h>
#include <gct/container_ring.h>

#define APBUART_REG_DATA	0
#define APBUART_REG_STATUS	4
#define  APBUART_REG_STATUS_DR 0x001
#define  APBUART_REG_STATUS_TF 0x200
#define APBUART_REG_CTRL	8
#define  APBUART_REG_CTRL_SI (1 << 14)
#define  APBUART_REG_CTRL_DI (1 << 13)
#define  APBUART_REG_CTRL_BI (1 << 12)
#define  APBUART_REG_CTRL_DB (1 << 11)
#define  APBUART_REG_CTRL_RF (1 << 10)
#define  APBUART_REG_CTRL_TF (1 << 9)
#define  APBUART_REG_CTRL_EC (1 << 8)
#define  APBUART_REG_CTRL_LB (1 << 7)
#define  APBUART_REG_CTRL_FL (1 << 6)
#define  APBUART_REG_CTRL_PE (1 << 5)
#define  APBUART_REG_CTRL_PS (1 << 4)
#define  APBUART_REG_CTRL_TI (1 << 3)
#define  APBUART_REG_CTRL_RI (1 << 2)
#define  APBUART_REG_CTRL_TE (1 << 1)
#define  APBUART_REG_CTRL_RE (1 << 0)
#define APBUART_REG_SCALER	12
#define APBUART_REG_DEBUG	16

/**************************************************************/

#define GCT_CONTAINER_ALGO_uart_fifo RING

GCT_CONTAINER_TYPES(uart_fifo, uint8_t, 32);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);

DRIVER_PV(struct gaisler_apbuart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t	read_q;
  dev_request_queue_root_t	write_q;
  uart_fifo_root_t		read_fifo;
#ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
  struct dev_irq_src_s          irq_ep;
#endif

  bool_t                        read_started:1;
  bool_t                        write_started:1;
});

static void gaisler_apbuart_try_read(struct device_s *dev)
{
  struct gaisler_apbuart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_rq_head(&pv->read_q)))
    {
      size_t size = 0;

      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);

      /* try to read more characters directly from device fifo */
      while ( size < rq->size && (cpu_mem_read_32(pv->addr + APBUART_REG_STATUS)
                                  & endian_be32(APBUART_REG_STATUS_DR)) )
        {
          rq->data[size++] = endian_be32(cpu_mem_read_32(pv->addr + APBUART_REG_DATA));
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

  /* copy more data from device fifo to driver fifo if no request currently need it */
  while (cpu_mem_read_32(pv->addr + APBUART_REG_STATUS)
         & endian_be32(APBUART_REG_STATUS_DR))
    {
      uart_fifo_pushback(&pv->read_fifo, endian_be32(cpu_mem_read_32(pv->addr + APBUART_REG_DATA)));
    }
}

static void gaisler_apbuart_try_write(struct device_s *dev)
{
  struct gaisler_apbuart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#ifdef CONFIG_DEVICE_IRQ
  /* try to write data from driver fifo to device */
  while (!uart_fifo_isempty(&pv->write_fifo) &&
         !(cpu_mem_read_32(pv->addr + APBUART_REG_STATUS)
           & endian_be32(APBUART_REG_STATUS_TF)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + APBUART_REG_DATA, endian_be32(c));
    }
#endif

  while ((rq = dev_char_rq_head(&pv->write_q)))
    {
      size_t size = 0;

#ifdef CONFIG_DEVICE_IRQ
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          while (size < rq->size)
            {
              if ( cpu_mem_read_32(pv->addr + APBUART_REG_STATUS)
                   & endian_be32(APBUART_REG_STATUS_TF) )
                break;

              cpu_mem_write_32(pv->addr + APBUART_REG_DATA, endian_be32(rq->data[size++]));
            }

#ifdef CONFIG_DEVICE_IRQ
        }

      /* some characters were not written to the device fifo, push to driver fifo */
      if (size < rq->size)
          size += uart_fifo_pushback_array(&pv->write_fifo, rq->data + size, rq->size - size);
#endif

      if (size)
        {
          rq->size -= size;
          rq->error = 0;
          rq->data += size;

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

#define gaisler_apbuart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(gaisler_apbuart_request)
{
  struct device_s               *dev = accessor->dev;
  struct gaisler_apbuart_context_s	*pv = dev->drv_pv;
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
          gaisler_apbuart_try_read(dev);
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
          gaisler_apbuart_try_write(dev);
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

static DEV_IRQ_SRC_PROCESS(gaisler_apbuart_irq)
{
  struct device_s *dev = ep->base.dev;

  lock_spin(&dev->lock);

  gaisler_apbuart_try_read(dev);
  gaisler_apbuart_try_write(dev);

  lock_release(&dev->lock);
}

#endif

#define gaisler_apbuart_use dev_use_generic

static DEV_INIT(gaisler_apbuart_init)
{
  struct gaisler_apbuart_context_s	*pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  pv->read_started = pv->write_started = 0;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  uintptr_t scaler;
  if (!device_get_param_uint(dev, "scaler", &scaler))
    cpu_mem_write_32(pv->addr + APBUART_REG_SCALER, endian_be32(scaler));

  uint32_t c = endian_be32(cpu_mem_read_32(pv->addr + APBUART_REG_CTRL));

  /* enable transmitter and receiver */
  c |= (APBUART_REG_CTRL_TE | APBUART_REG_CTRL_RE);

  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);

  uart_fifo_init(&pv->read_fifo);

#ifdef CONFIG_DEVICE_IRQ

  c |=  (APBUART_REG_CTRL_TI | APBUART_REG_CTRL_RI);
  c &= ~(APBUART_REG_CTRL_DI | APBUART_REG_CTRL_BI | APBUART_REG_CTRL_SI | APBUART_REG_CTRL_RF | APBUART_REG_CTRL_TF);

  uart_fifo_init(&pv->write_fifo);

  device_irq_source_init(dev, &pv->irq_ep, 1, &gaisler_apbuart_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_fifo;

#else

  /* disable irqs */
  c &= ~(APBUART_REG_CTRL_RF | APBUART_REG_CTRL_TF | APBUART_REG_CTRL_DI |
         APBUART_REG_CTRL_BI | APBUART_REG_CTRL_SI | APBUART_REG_CTRL_TI | APBUART_REG_CTRL_RI);
#endif

  cpu_mem_write_32(pv->addr + APBUART_REG_CTRL, endian_be32(c));


  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(gaisler_apbuart_cleanup)
{
  struct gaisler_apbuart_context_s	*pv = dev->drv_pv;

  uint32_t c = endian_be32(cpu_mem_read_32(pv->addr + APBUART_REG_CTRL));

#ifdef CONFIG_DEVICE_IRQ

  /* disable irqs */
  c &= ~(APBUART_REG_CTRL_RF | APBUART_REG_CTRL_TF | APBUART_REG_CTRL_DI |
         APBUART_REG_CTRL_BI | APBUART_REG_CTRL_SI | APBUART_REG_CTRL_TI | APBUART_REG_CTRL_RI);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  uart_fifo_destroy(&pv->write_fifo);

#endif

  /* disable transmitter and receiver */
  c &= ~(APBUART_REG_CTRL_TE | APBUART_REG_CTRL_RE);

  cpu_mem_write_32(pv->addr + APBUART_REG_CTRL, endian_be32(c));

  uart_fifo_destroy(&pv->read_fifo);

  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(gaisler_apbuart_drv, 0, "Gaisler APB UART", gaisler_apbuart,
               DRIVER_CHAR_METHODS(gaisler_apbuart));

DRIVER_REGISTER(gaisler_apbuart_drv,
                DEV_ENUM_GAISLER_ENTRY(0x1, 0x00c));

