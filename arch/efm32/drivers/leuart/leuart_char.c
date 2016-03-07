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

#include <mutek/printk.h>

#include <arch/efm32/leuart.h>

static void efm32_leuart_write_reg(uint32_t addr, uintptr_t reg, uint32_t x)
{
  while (cpu_mem_read_32(addr + EFM32_LEUART_SYNCBUSY_ADDR)
         & endian_le32(EFM32_LEUART_SYNCBUSY_MASK))
    ;
  cpu_mem_write_32(addr + reg, x);
}

#include <mutek/mem_alloc.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_EFM32_LEUART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);
#endif

struct efm32_leuart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t	read_q;
  dev_request_queue_root_t	write_q;
#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t		write_fifo;
# endif
#endif
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif
  uint32_t                      mode;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  uint32_t                      bauds;
#endif
  struct dev_freq_s             freq;

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s    clk_ep;
#endif
};

#if CONFIG_DEVICE_START_LOG2INC < 2
# error
#endif
enum efm32_leuart_start_e
{
  EFM32_LEUART_STARTED_READ = 1,
  EFM32_LEUART_STARTED_WRITE = 2,
};

static uint32_t efm32_leuart_char_bauds(struct device_s *dev, uint32_t bauds)
{
  struct efm32_leuart_context_s *pv = dev->drv_pv;
  return EFM32_LEUART_CLKDIV_DIV((32 * pv->freq.num) / (bauds * pv->freq.denom) - 32);
}

static void efm32_leuart_try_read(struct device_s *dev)
{
  struct efm32_leuart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
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
    irqen:
      efm32_leuart_write_reg(pv->addr, EFM32_LEUART_IEN_ADDR,
                             endian_le32(EFM32_LEUART_IEN_TXC | EFM32_LEUART_IEN_RXDATAV));
      /* more data will be available on next interrupt */
      return;
#endif
    }

/* if no request currently need incoming data, copy to driver fifo or discard. */
  if (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
           & endian_le32(EFM32_LEUART_STATUS_RXDATAV))
    {
      __unused__ uint32_t c = endian_le32(cpu_mem_read_32(pv->addr + EFM32_LEUART_RXDATAX_ADDR));
#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
      uart_fifo_pushback(&pv->read_fifo, c);
#endif
    }

#ifdef CONFIG_DEVICE_IRQ
  /* disable rx irqs */
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_IEN_ADDR,
                         endian_le32(EFM32_LEUART_IEN_TXC));

# ifdef CONFIG_DEVICE_CLOCK_GATING
  /* race: we can not disable clock yet if an irq has been edge triggered */
  if (cpu_mem_read_32(pv->addr + EFM32_LEUART_IF_ADDR)
      & endian_le32(EFM32_LEUART_IF_RXDATAV))
    goto irqen;
# endif
#endif

  dev->start_count &= ~EFM32_LEUART_STARTED_READ;
}

static void efm32_leuart_try_write(struct device_s *dev)
{
  struct efm32_leuart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;
  bool_t done = 0;

#ifdef CONFIG_DEVICE_IRQ
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_IFC_ADDR,
                         endian_le32(EFM32_LEUART_IFC_TXC));
#endif

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  /* try to write data from driver fifo to device */
  if (!uart_fifo_isempty(&pv->write_fifo) &&
         (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
           & endian_le32(EFM32_LEUART_STATUS_TXBL)) )
    {
      efm32_leuart_write_reg(pv->addr, EFM32_LEUART_TXDATAX_ADDR,
                             endian_le32(uart_fifo_pop(&pv->write_fifo)));
      done = 1;
    }
#endif

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q))))
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
              efm32_leuart_write_reg(pv->addr, EFM32_LEUART_TXDATAX_ADDR,
                                     endian_le32(rq->data[size++]));
              done = 1;
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
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_request_queue_pop(&pv->write_q);
              kroutine_exec(&rq->base.kr);
              continue;
            }
        }

#ifdef CONFIG_DEVICE_IRQ
      /* more fifo space will be available on next interrupt */
      return;
#endif
    }

#if CONFIG_DRIVER_EFM32_LEUART_CHAR_SWFIFO > 0
  if (!uart_fifo_isempty(&pv->write_fifo))
    return;
#endif

#ifdef CONFIG_DEVICE_IRQ
  if (!done)
#endif
    dev->start_count &= ~EFM32_LEUART_STARTED_WRITE;
}

#define efm32_leuart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(efm32_leuart_request)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_leuart_context_s	*pv = dev->drv_pv;
  error_t err = 0;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ: {
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));
      dev->start_count |= EFM32_LEUART_STARTED_READ;
#ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif
      efm32_leuart_try_read(dev);
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
      dev->start_count |= EFM32_LEUART_STARTED_WRITE;
#ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif
      efm32_leuart_try_write(dev);
      break;
    }
    default:
      err = -ENOTSUP;
    }

#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (!dev->start_count)
    dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      kroutine_exec(&rq->base.kr);
    }
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(efm32_leuart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_leuart_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ir = endian_le32(cpu_mem_read_32(pv->addr + EFM32_LEUART_IF_ADDR));

      if (!(ir & (EFM32_LEUART_IF_TXC | EFM32_LEUART_IF_RXDATAV)))
        break;

      if (ir & EFM32_LEUART_IF_TXC)
        efm32_leuart_try_write(dev);

      if (ir & EFM32_LEUART_IF_RXDATAV)
        efm32_leuart_try_read(dev);
    }

#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (!dev->start_count)
    dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  lock_release(&dev->lock);
}

#endif

static DEV_INIT(efm32_leuart_init);
static DEV_CLEANUP(efm32_leuart_cleanup);

static DEV_USE(efm32_leuart_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_NOTIFY: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_leuart_context_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
# ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
# endif
      efm32_leuart_write_reg(pv->addr, EFM32_LEUART_CLKDIV_ADDR,
                       endian_le32(efm32_leuart_char_bauds(dev, pv->bauds)));
# ifdef CONFIG_DEVICE_CLOCK_GATING
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
# endif
      return 0;
    }
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_leuart_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_leuart_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

DRIVER_DECLARE(efm32_leuart_drv, 0, "EFM32 Low Energy UART", efm32_leuart,
               DRIVER_CHAR_METHODS(efm32_leuart));

DRIVER_REGISTER(efm32_leuart_drv);

static DEV_INIT(efm32_leuart_init)
{
  struct efm32_leuart_context_s	*pv;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, DEV_CLOCK_EP_SINK_NOTIFY |
                      DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_SINK_SYNC);

  if (dev_clock_sink_link(&pv->clk_ep, 0, &pv->freq))
    goto err_mem;
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;
#endif

  /* wait for current TX to complete */
  if (cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
      & endian_le32(EFM32_LEUART_STATUS_TXENS))
      while (!(cpu_mem_read_32(pv->addr + EFM32_LEUART_STATUS_ADDR)
               & endian_le32(EFM32_LEUART_STATUS_TXBL)))
        ;

  /* disable and clear the uart */
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_CMD_ADDR,
                    endian_le32(EFM32_LEUART_CMD_RXDIS | EFM32_LEUART_CMD_TXDIS));

  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_CMD_ADDR,
                    endian_le32(EFM32_LEUART_CMD_CLEARRX | EFM32_LEUART_CMD_CLEARTX));

  /* setup pinmux */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, "<rx? >tx?", loc, NULL, NULL))
    goto err_clk;

  uint32_t route = 0;
  if (loc[0] != IOMUX_INVALID_DEMUX)
    route |= EFM32_LEUART_ROUTE_RXPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    route |= EFM32_LEUART_ROUTE_TXPEN;

  if (route == 0)
    goto err_clk;

  EFM32_LEUART_ROUTE_LOCATION_SETVAL(route, loc[0] != IOMUX_INVALID_DEMUX ? loc[0] : loc[1]);

  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_ROUTE_ADDR, endian_le32(route));

  /* init software fifos */
  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  /* init irq endpoint */
  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_leuart_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_fifo;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_IEN_ADDR,
                         endian_le32(EFM32_LEUART_IEN_TXC));
#endif

  /* setup baud rate */
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  pv->bauds = 9600;
#endif
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_CLKDIV_ADDR,
                         endian_le32(efm32_leuart_char_bauds(dev, 9600)));

  /* enable the uart */
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_CMD_ADDR,
                    endian_le32(EFM32_LEUART_CMD_RXEN | EFM32_LEUART_CMD_TXEN
                                | EFM32_LEUART_CMD_RXBLOCKDIS));

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
#endif
 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(&pv->clk_ep);
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
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_IEN_ADDR, 0);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif

  /* disable the uart */
  efm32_leuart_write_reg(pv->addr, EFM32_LEUART_CMD_ADDR,
                         endian_le32(EFM32_LEUART_CMD_RXDIS | EFM32_LEUART_CMD_TXDIS));

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(&pv->clk_ep);
#endif

#if CONFIG_DRIVER_EFM32_LEUART_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
# endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  mem_free(pv);

  return 0;
}

