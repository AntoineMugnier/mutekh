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
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2018
*/

#define LOGK_MODULE_ID "efmu"

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/iomux.h>
#include <device/resource/uart.h>
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
# include <device/class/valio.h>
# include <device/valio/uart_config.h>
#endif
#include <device/clock.h>

#include <arch/efm32/usart.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct/container_ring.h>

#define GCT_CONTAINER_ALGO_uart_fifo RING

GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_EFM32_USART_CHAR_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, isfull,
                   pop, pop_array, pushback, pushback_array);

#if CONFIG_DEVICE_START_LOG2INC < 2
# error
#endif

enum efm32_usart_start_e
{
  EFM32_USART_STARTED_READ = 1,
  EFM32_USART_STARTED_WRITE = 2,
};

struct efm32_usart_context_s
{
  uintptr_t addr;

  /* tty input request queue and char fifo */
  dev_request_queue_root_t		read_q;
  dev_request_queue_root_t		write_q;

  uart_fifo_root_t		read_fifo;
  uart_fifo_root_t		write_fifo;

  struct kroutine_s             read_kr;
  struct kroutine_s             write_kr;

  struct dev_irq_src_s           irq_ep[2];

  struct dev_uart_config_s      cfg;
  bool_t                        cfg_pending;

  struct dev_freq_s             freq;
  struct dev_clock_sink_ep_s    clk_ep;
};

DRIVER_PV(struct efm32_usart_context_s);

static
void efm32_usart_char_cfg_apply(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  logk_trace("cfg %s", pv->cfg_pending ? "pending" : "valid");

  if (!pv->cfg_pending)
    return;

  pv->cfg_pending = 0;

  uint64_t num = 256 * (uint64_t)pv->freq.num;
  uint64_t den = 16 * pv->cfg.baudrate * pv->freq.denom;
  uint32_t div = num / den - 256;

  uint32_t frame = 0;

  EFM32_USART_FRAME_DATABITS_SETVAL(frame,
    EFM32_USART_FRAME_DATABITS_FOUR + pv->cfg.data_bits - 4);

  switch (pv->cfg.parity)
    {
    case DEV_UART_PARITY_EVEN:
      EFM32_USART_FRAME_PARITY_SET(frame, EVEN);
      break;
    case DEV_UART_PARITY_ODD:
      EFM32_USART_FRAME_PARITY_SET(frame, ODD);
      break;
    case DEV_UART_PARITY_NONE:
    default:
      EFM32_USART_FRAME_PARITY_SET(frame, NONE);
      break;
    }

  switch (pv->cfg.stop_bits)
    {
    case 2:
      EFM32_USART_FRAME_STOPBITS_SET(frame, TWO);
      break;
    case 1:
    default:
      EFM32_USART_FRAME_STOPBITS_SET(frame, ONE);
      break;
    }

  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, frame);
  cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, div);
}

static
void efm32_usart_char_cfg_pending(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  logk_trace("cfg to be applied");

  pv->cfg_pending = 1;

  if (dev->start_count)
    efm32_usart_char_cfg_apply(dev);
}

static
void efm32_usart_read_flush(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  logk_trace("read flush");

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
    {
      size_t size = 0;

      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);

      logk_trace(" -> %d bytes", size);

      cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR,
                       EFM32_USART_IEN_RXDATAV
                       | cpu_mem_read_32(pv->addr + EFM32_USART_IEN_ADDR));

      if (!size)
        break;

      rq->size -= size;
      rq->data += size;
      rq->error = 0;

      if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
        {
          dev_request_queue_pop(&pv->read_q);
          kroutine_exec(&rq->base.kr);

          device_sleep_schedule(dev);
        }
    }

  device_sleep_schedule(dev);
}

static void efm32_usart_write_refill(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  logk_trace("write refill");

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q))))
    {
      size_t size;

      size = uart_fifo_pushback_array(&pv->write_fifo, rq->data, rq->size);

      cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR,
                       EFM32_USART_IEN_TXBL
                       | cpu_mem_read_32(pv->addr + EFM32_USART_IEN_ADDR));

      logk_trace(" -> %d bytes", size);

      if (!size)
        return;

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

  device_sleep_schedule(dev);
}

static KROUTINE_EXEC(efm32_usart_write_refill_kr)
{
  struct efm32_usart_context_s *pv = KROUTINE_CONTAINER(kr, *pv, write_kr);
  struct device_s *dev = pv->irq_ep[0].base.dev;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  efm32_usart_write_refill(dev);
}

static KROUTINE_EXEC(efm32_usart_read_flush_kr)
{
  struct efm32_usart_context_s *pv = KROUTINE_CONTAINER(kr, *pv, read_kr);
  struct device_s *dev = pv->irq_ep[0].base.dev;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  efm32_usart_read_flush(dev);
}

static DEV_CHAR_CANCEL(efm32_usart_cancel)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  dev_request_queue_root_t *q;

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL:
      q = &pv->read_q;
      break;

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
      q = &pv->write_q;
      break;

    default:
      return -EINVAL;
    }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  bool_t present = 0;

  GCT_FOREACH(dev_request_queue, q, item,
              if (item == dev_char_rq_s_base(rq))
                {
                  present = 1;
                  GCT_FOREACH_BREAK;
                }
      );

   if (!present)
      return -ENOENT;

  dev_request_queue_remove(q, dev_char_rq_s_base(rq));

  if (dev_request_queue_isempty(q))
    device_sleep_schedule(dev);

  return 0;
}

static DEV_CHAR_REQUEST(efm32_usart_request)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  dev_request_queue_root_t *q;

  assert(rq->size);

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL:
      q = &pv->read_q;
      break;

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
      q = &pv->write_q;
      break;

    default:
      rq->error = -ENOTSUP;
      kroutine_exec(&rq->base.kr);
      return;
    }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_trace("enqueue %s", rq->type & _DEV_CHAR_WRITE ? "write" : "read");

  if (dev->start_count == 0)
    {
      logk_trace("power on");
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
      efm32_usart_char_cfg_apply(dev);
    }

  enum efm32_usart_start_e start = rq->type & _DEV_CHAR_WRITE
    ? EFM32_USART_STARTED_WRITE
    : EFM32_USART_STARTED_READ;

  if (!(dev->start_count & start))
    {
      uint32_t cmd = rq->type & _DEV_CHAR_WRITE
        ? EFM32_USART_CMD_TXEN
        : EFM32_USART_CMD_RXEN;

      logk_trace("start %s", rq->type & _DEV_CHAR_WRITE ? "write" : "read");

      dev->start_count |= start;
      cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, cmd);
    }

  dev_request_queue_pushback(q, dev_char_rq_s_base(rq));

  if (rq->type & _DEV_CHAR_WRITE)
    efm32_usart_write_refill(dev);
  else
    efm32_usart_read_flush(dev);
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)

static DEV_VALIO_REQUEST(efm32_usart_valio_request)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_context_s *pv = dev->drv_pv;

  req->error = -ENOTSUP;

  if (req->attribute == VALIO_UART_CONFIG)
    {
      switch (req->type)
        {
        case DEVICE_VALIO_READ:
          LOCK_SPIN_IRQ(&dev->lock);
          *(struct dev_uart_config_s*)req->data = pv->cfg;
          LOCK_RELEASE_IRQ(&dev->lock);

          req->error = 0;
          break;

        case DEVICE_VALIO_WRITE:
          LOCK_SPIN_IRQ(&dev->lock);
          pv->cfg = *(struct dev_uart_config_s*)req->data;
          efm32_usart_char_cfg_pending(dev);
          LOCK_RELEASE_IRQ(&dev->lock);

          req->error = 0;
          break;

        default:
          break;
        }
    }

  kroutine_exec(&req->base.kr);
}

#define efm32_usart_valio_cancel (dev_valio_cancel_t*)dev_driver_notsup_fcn
#endif


static DEV_IRQ_SRC_PROCESS(efm32_usart_irq_rx)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  LOCK_SPIN_SCOPED(&dev->lock);

  logk_trace("RX IRQ");

  while (cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR) & EFM32_USART_IF_RXDATAV)
    {
      if (uart_fifo_isfull(&pv->read_fifo))
        {
          cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR,
                           ~EFM32_USART_IEN_RXDATAV
                           & cpu_mem_read_32(pv->addr + EFM32_USART_IEN_ADDR));
          break;
        }

      uint8_t data = cpu_mem_read_32(pv->addr + EFM32_USART_RXDATA_ADDR);
      uart_fifo_pushback(&pv->read_fifo, data);
    }

  kroutine_exec(&pv->read_kr);
}

static DEV_IRQ_SRC_PROCESS(efm32_usart_irq_tx)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  LOCK_SPIN_SCOPED(&dev->lock);

  logk_trace("TX IRQ");

  while (cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR) & EFM32_USART_IF_TXBL)
    {
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR,
                           ~EFM32_USART_IEN_TXBL
                           & cpu_mem_read_32(pv->addr + EFM32_USART_IEN_ADDR));
          break;
        }

      uint8_t data = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + EFM32_USART_TXDATA_ADDR, data);
    }

  kroutine_exec(&pv->write_kr);
}

static DEV_USE(efm32_usart_char_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_usart_context_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      efm32_usart_char_cfg_pending(dev);
      return 0;
    }
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_usart_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        {
          dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
          efm32_usart_char_cfg_apply(dev);
        }
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      device_sleep_schedule(dev);
      return 0;
    }
#endif

    case DEV_USE_SLEEP: {
      struct device_s *dev = param;
      struct efm32_usart_context_s *pv = dev->drv_pv;

      logk_trace("Sleep ?");

      if ((dev->start_count & EFM32_USART_STARTED_READ)
          && dev_request_queue_isempty(&pv->read_q)) {
        logk_trace(" -> rx disable");
        dev->start_count &= ~EFM32_USART_STARTED_READ;
        cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, EFM32_USART_CMD_RXDIS);
      }

      if ((dev->start_count & EFM32_USART_STARTED_WRITE)
          && dev_request_queue_isempty(&pv->write_q)
          && uart_fifo_isempty(&pv->write_fifo)
          && (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
              & EFM32_USART_STATUS_TXC)) {
        logk_trace(" -> tx disable");
        dev->start_count &= ~EFM32_USART_STARTED_WRITE;
        cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, EFM32_USART_CMD_TXDIS);
      }

#ifdef CONFIG_DEVICE_CLOCK_GATING
      if (!dev->start_count) {
        logk_trace(" -> power disable");
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
      }
#endif

      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(efm32_usart_char_init)
{
  struct efm32_usart_context_s	*pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL);
  if (err) {
    logk_error("Bad device address");
    goto err_mem;
  }

  /* setup pinmux */
  iomux_io_id_t pin[2];
  iomux_demux_t loc[2];
  err = device_iomux_setup(dev, "<rx? >tx?", loc, pin, NULL);
  if (err) {
    logk_error("Bad pins");
    goto err_mem;
  }

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
  uint32_t enable = 0;
  uint32_t route = 0;

  if (loc[0] != IOMUX_INVALID_DEMUX)
    {
      enable |= EFM32_USART_ROUTEPEN_RXPEN;
      EFM32_USART_ROUTELOC0_RXLOC_SETVAL(route, loc[0]);
    }
  if (loc[1] != IOMUX_INVALID_DEMUX)
    {
      enable |= EFM32_USART_ROUTEPEN_TXPEN;
      EFM32_USART_ROUTELOC0_TXLOC_SETVAL(route, loc[1]);
    }
  if (enable == 0) {
    err = -EINVAL;
    logk_error("Bad IOMUX")
    goto err_mem;
  }
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  uint32_t route = 0;
  if (loc[0] != IOMUX_INVALID_DEMUX)
    route |= EFM32_USART_ROUTE_RXPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    route |= EFM32_USART_ROUTE_TXPEN;

  EFM32_USART_ROUTE_LOCATION_SETVAL(route, loc[0] != IOMUX_INVALID_DEMUX ? loc[0] : loc[1]);

  if (route == 0) {
    err = -EINVAL;
    logk_error("Bad IOMUX");
    goto err_mem;
  }
#else
# error
#endif

  /* init queues */
  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

  kroutine_init_deferred(&pv->read_kr, efm32_usart_read_flush_kr);
  kroutine_init_deferred(&pv->write_kr, efm32_usart_write_refill_kr);

  uart_fifo_init(&pv->read_fifo);
  uart_fifo_init(&pv->write_fifo);

  err = dev_drv_clock_init(
    dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
    DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->freq);
  if (err) {
    logk_error("Bad clock init");
    goto err_fifo;
  }

  /* wait for current TX to complete */
  if (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
      & EFM32_USART_STATUS_TXENS)
      while (!(cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
               & EFM32_USART_STATUS_TXBL))
        ;

  /* disable and clear the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS);

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX);

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTELOC0_ADDR, route);
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTEPEN_ADDR, enable);
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, route);
#else
# error
#endif

  /* enable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, EFM32_USART_IFC_MASK);
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

  /* configure */
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR,
                   EFM32_USART_CTRL_OVS(X16) | EFM32_USART_CTRL_TXBIL(HALF_FULLD));

  /* set config */
  if (device_get_res_uart(dev, &pv->cfg)) {
    pv->cfg.baudrate = CONFIG_DRIVER_EFM32_USART_RATE;
    pv->cfg.data_bits = 8;
    pv->cfg.parity = DEV_UART_PARITY_NONE;
    pv->cfg.stop_bits = 1;
  }

  efm32_usart_char_cfg_pending(dev);

  /* enable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   EFM32_USART_CMD_TXEN | EFM32_USART_CMD_RXBLOCKDIS);

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  /* init irq endpoints */
  device_irq_source_init(dev, &pv->irq_ep[0], 1, &efm32_usart_irq_rx);
  device_irq_source_init(dev, &pv->irq_ep[1], 1, &efm32_usart_irq_tx);

  err = device_irq_source_link(dev, pv->irq_ep, 2, -1);
  if (err) {
    logk_error("Bad IRQ");
    goto err_irq;
  }

  return 0;

 err_irq:
  device_irq_source_unlink(dev, pv->irq_ep, 2);
 err_fifo:
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
 err_mem:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(efm32_usart_char_cleanup)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  /* disable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 2);

  /* disable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS);

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_usart_drv, 0, "EFM32 USART (char,swfifo)", efm32_usart_char,
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
               DRIVER_VALIO_METHODS(efm32_usart_valio),
#endif
               DRIVER_CHAR_METHODS(efm32_usart));

DRIVER_REGISTER(efm32_usart_drv);
