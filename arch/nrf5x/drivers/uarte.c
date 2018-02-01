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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2014, Nicolas Pouillon <nipo@ssji.net>
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
#include <device/class/uart.h>
#include <device/class/iomux.h>

#include <arch/nrf5x/uarte.h>
#include <arch/nrf5x/gpio.h>

#define START_RX 1
#define START_TX 2

struct nrf5x_uarte_priv
{
  uintptr_t addr;

  union {
    uint8_t byte[16];
    uint32_t word[4];
  } buffer;

  dev_request_queue_root_t rx_q;
  dev_request_queue_root_t tx_q;

  struct dev_irq_src_s irq_ep;

  bool_t has_ctsrts : 1;
  bool_t must_flush : 1;
  bool_t from_rom : 1;
};

DRIVER_PV(struct nrf5x_uarte_priv);

static void nrf5x_uarte_request_finish(struct device_s *dev,
                                       struct dev_char_rq_s *rq,
                                       size_t count)
{
  struct nrf5x_uarte_priv *pv = dev->drv_pv;

  rq->size -= count;
  rq->data += count;

  logk_trace("%s DONE %p %p %d\n", __FUNCTION__, rq, rq->data, rq->size);
  kroutine_exec(&rq->base.kr);
}

static void nrf5x_uarte_rx_start(struct device_s *dev)
{
  struct nrf5x_uarte_priv *pv = dev->drv_pv;
  struct dev_char_rq_s *rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->rx_q));

  logk_trace("%s START RX %p %p %d\n", __FUNCTION__, rq, rq->data, rq->size);

  assert(rq);

  dev->start_count |= START_RX;

  nrf_reg_set(pv->addr, NRF_UARTE_RXD_PTR, (uintptr_t)rq->data);
  nrf_reg_set(pv->addr, NRF_UARTE_RXD_MAXCNT, rq->size);

  if (rq->type & _DEV_CHAR_PARTIAL) {
    nrf_event_clear(pv->addr, NRF_UARTE_RXTO);
    nrf_it_enable(pv->addr, NRF_UARTE_RXTO);
  }

  nrf_event_clear(pv->addr, NRF_UARTE_ENDRX);
  nrf_it_enable(pv->addr, NRF_UARTE_ENDRX);
  nrf_event_clear(pv->addr, NRF_UARTE_ERROR);
  nrf_it_enable(pv->addr, NRF_UARTE_ERROR);

  nrf_task_trigger(pv->addr, NRF_UARTE_STARTRX);
}

static void nrf5x_uarte_tx_start(struct device_s *dev)
{
  struct nrf5x_uarte_priv *pv = dev->drv_pv;
  struct dev_char_rq_s *rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->tx_q));
  uintptr_t source = (uintptr_t)rq->data;
  size_t size = rq->size;

  assert(rq);

  dev->start_count |= START_TX;

  if (source < 0x20000000) {
    uintptr_t offset = source % sizeof(pv->buffer);
    uint32_t *src = (uint32_t *)(source - offset);

    for (uint8_t i = 0; i < sizeof(pv->buffer) / 4; ++i)
      pv->buffer.word[i] = src[i];

    source = (uintptr_t)pv->buffer.byte + offset;
    size = __MIN(size, sizeof(pv->buffer) - offset);
  }

  nrf_reg_set(pv->addr, NRF_UARTE_TXD_PTR, source);
  nrf_reg_set(pv->addr, NRF_UARTE_TXD_MAXCNT, size);

  logk_trace("%s START TX %p %p %d %P\n", __FUNCTION__, rq,
          source, size,
          source, size);

  if (rq->type & _DEV_CHAR_PARTIAL && pv->has_ctsrts) {
    nrf_event_clear(pv->addr, NRF_UARTE_NCTS);
    nrf_it_enable(pv->addr, NRF_UARTE_NCTS);
  } else {
    nrf_event_clear(pv->addr, NRF_UARTE_ENDTX);
    nrf_it_enable(pv->addr, NRF_UARTE_ENDTX);
    if (source != (uintptr_t)rq->data)
      pv->from_rom = 1;
  }
  nrf_task_trigger(pv->addr, NRF_UARTE_STARTTX);
}

static DEV_IRQ_SRC_PROCESS(nrf5x_uarte_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_uarte_priv *pv = dev->drv_pv;
  struct dev_char_rq_s *rq;
  size_t xfered;
  
  logk_trace("%s\n", __FUNCTION__);

  LOCK_SPIN_SCOPED(&dev->lock);

  for (;;) {
    if (nrf_it_is_enabled(pv->addr, NRF_UARTE_NCTS)
        && nrf_event_check(pv->addr, NRF_UARTE_NCTS)) {
      nrf_event_clear(pv->addr, NRF_UARTE_NCTS);
      nrf_it_disable(pv->addr, NRF_UARTE_NCTS);
      logk_trace("%s NCTS\n", __FUNCTION__);

      nrf_event_clear(pv->addr, NRF_UARTE_ENDTX);
      nrf_it_enable(pv->addr, NRF_UARTE_ENDTX);
      nrf_task_trigger(pv->addr, NRF_UARTE_STOPTX);
    }

    if (nrf_event_check(pv->addr, NRF_UARTE_ENDTX)) {
      nrf_event_clear(pv->addr, NRF_UARTE_ENDTX);
      nrf_it_disable(pv->addr, NRF_UARTE_ENDTX);
      logk_trace("%s END TX\n", __FUNCTION__);

      dev->start_count &= ~START_TX;

      xfered = nrf_reg_get(pv->addr, NRF_UARTE_TXD_AMOUNT);

      if (pv->from_rom) {
        pv->from_rom = 0;
        
        rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->tx_q));
        rq->size -= xfered;
        rq->data += xfered;

        if (rq->size) {
          nrf5x_uarte_tx_start(dev);
          continue;
        }
      }

      rq = dev_char_rq_s_cast(dev_request_queue_pop(&pv->tx_q));
      assert(rq);

      nrf5x_uarte_request_finish(dev, rq, xfered);

      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->tx_q));
      if (rq)
        nrf5x_uarte_tx_start(dev);

      continue;
    }

    if (nrf_it_is_enabled(pv->addr, NRF_UARTE_ERROR)
        && nrf_event_check(pv->addr, NRF_UARTE_ERROR)) {
      nrf_event_clear(pv->addr, NRF_UARTE_ERROR);
      nrf_it_disable(pv->addr, NRF_UARTE_ERROR);
      logk_trace("%s ERROR\n", __FUNCTION__);

      uint32_t error = nrf_reg_get(pv->addr, NRF_UARTE_ERRORSRC);
      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->rx_q));
      assert(rq);

      nrf_event_clear(pv->addr, NRF_UARTE_RXTO);
      nrf_it_enable(pv->addr, NRF_UARTE_RXTO);

      dev->start_count &= ~START_RX;

      nrf_task_trigger(pv->addr, NRF_UARTE_STOPRX);
      // Will trigger RXTO

      if (error & NRF_UARTE_ERRORSRC_PARITY)
        rq->error = -EBADDATA;
      else if (error & (NRF_UARTE_ERRORSRC_OVERRUN
                        | NRF_UARTE_ERRORSRC_FRAMING
                        | NRF_UARTE_ERRORSRC_BREAK))
        rq->error = -EPIPE;
    }

    if (nrf_it_is_enabled(pv->addr, NRF_UARTE_RXTO)
        && nrf_event_check(pv->addr, NRF_UARTE_RXTO)) {
      nrf_event_clear(pv->addr, NRF_UARTE_RXTO);
      nrf_it_disable(pv->addr, NRF_UARTE_RXTO);
      logk_trace("%s RXTO\n", __FUNCTION__);

      dev->start_count &= ~START_RX;

      nrf_task_trigger(pv->addr, NRF_UARTE_STOPRX);
      pv->must_flush = 1;
    }

    if (nrf_event_check(pv->addr, NRF_UARTE_ENDRX)) {
      nrf_event_clear(pv->addr, NRF_UARTE_ENDRX);
      nrf_event_clear(pv->addr, NRF_UARTE_RXTO);
      nrf_it_disable(pv->addr, NRF_UARTE_RXTO);
      nrf_it_disable(pv->addr, NRF_UARTE_ERROR);

      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->rx_q));
      xfered = nrf_reg_get(pv->addr, NRF_UARTE_RXD_AMOUNT);

      logk_trace("%s END RX, must flush: %d, %P\n", __FUNCTION__, pv->must_flush,
              rq->data, xfered);

      if (pv->must_flush) {
        pv->must_flush = 0;

        rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->rx_q));
        assert(rq);

        rq->size -= xfered;
        rq->data += xfered;

        nrf_reg_set(pv->addr, NRF_UARTE_RXD_PTR, (uintptr_t)rq->data);
        nrf_reg_set(pv->addr, NRF_UARTE_RXD_MAXCNT, rq->size);

        nrf_task_trigger(pv->addr, NRF_UARTE_FLUSHRX);
      } else {
        nrf_it_disable(pv->addr, NRF_UARTE_ENDRX);

        rq = dev_char_rq_s_cast(dev_request_queue_pop(&pv->rx_q));
        assert(rq);

        nrf5x_uarte_request_finish(dev, rq, xfered);

        rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->rx_q));
        if (rq)
          nrf5x_uarte_rx_start(dev);
      }

      continue;
    }

    break;
  }
}

static DEV_CHAR_REQUEST(nrf5x_uarte_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_uarte_priv *pv = dev->drv_pv;
  dev_request_queue_root_t *q = NULL;
  bool_t start;

  logk_trace("%s REQUEST %x %p %p %d\n", __FUNCTION__, rq->type, rq, rq->data, rq->size);

  switch (rq->type) {
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ:
    q = &pv->rx_q;
    break;

  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE:
    q = &pv->tx_q;
    break;

  default:
    rq->error = -ENOTSUP;
    kroutine_exec(&rq->base.kr);
    return;
  }

  assert(rq->size);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  rq->error = 0;

  start = dev_request_queue_isempty(q);
  dev_request_queue_pushback(q, &rq->base);

  if (start) {
    logk_trace("%s START\n", __FUNCTION__);

    if (q == &pv->rx_q)
      nrf5x_uarte_rx_start(dev);
    else
      nrf5x_uarte_tx_start(dev);
  }
}

static DEV_CHAR_CANCEL(nrf5x_uarte_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_uarte_priv *pv = dev->drv_pv;
  dev_request_queue_root_t *q = NULL;

  switch (rq->type) {
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ:
    q = &pv->rx_q;
    break;

  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE:
    q = &pv->tx_q;
    break;

  default:
    return -EINVAL;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (dev_request_queue_head(q) == &rq->base)
    return -EBUSY;

  dev_request_queue_remove(q, &rq->base);

  return 0;
}

static error_t nrf5x_uarte_config(struct nrf5x_uarte_priv *pv,
                                  struct dev_uart_config_s *cfg)
{
  uint32_t config = 0;
  uintptr_t baudrate = cfg->baudrate;

  if (!dev_request_queue_isempty(&pv->rx_q)
      || !dev_request_queue_isempty(&pv->tx_q))
    return -EBUSY;

  if (cfg->data_bits != 8)
    return -ENOTSUP;

  if (baudrate > 1000000)
    return -ENOTSUP;

  if (cfg->stop_bits != 1)
    return -ENOTSUP;

  switch  (cfg->parity) {
  case DEV_UART_PARITY_NONE:
    config |= NRF_UARTE_CONFIG_PARITY_DISABLED;
    break;
  case DEV_UART_PARITY_ODD:
    return -ENOTSUP;
  case DEV_UART_PARITY_EVEN:
    config |= NRF_UARTE_CONFIG_PARITY_ENABLED;
    break;
  }

  if (cfg->flow_ctrl) {
    if (!pv->has_ctsrts)
      return -ENOTSUP;

    config |= NRF_UARTE_CONFIG_CTSRTS_ENABLED;
  }

  nrf_reg_set(pv->addr, NRF_UARTE_CONFIG, config);
  nrf_reg_set(pv->addr, NRF_UARTE_BAUDRATE, NRF_UARTE_BAUDRATE_(baudrate));

  return 0;
}

#if defined(CONFIG_DEVICE_UART)

static DEV_UART_CONFIG(nrf5x_uarte_uart_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_uarte_priv *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  return nrf5x_uarte_config(pv, cfg);
}

#endif

#define nrf5x_uarte_char_use dev_use_generic

static DEV_INIT(nrf5x_uarte_char_init)
{
  struct nrf5x_uarte_priv *pv;
  iomux_io_id_t id[4];
  struct dev_uart_config_s config = {
    .baudrate = 115200,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = DEV_UART_PARITY_NONE,
    .flow_ctrl = 0,
  };

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  if (device_iomux_setup(dev, "<rx? >tx? >rts? <cts?", NULL, id, NULL))
    goto free_pv;

  dev_request_queue_init(&pv->rx_q);
  dev_request_queue_init(&pv->tx_q);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(pv->addr, NRF_UARTE_ENABLE, 0);

  // Pin config is done by iomux driver, select pins in UART.
  nrf_reg_set(
              pv->addr, NRF_UARTE_PSEL_RXD,
              id[0] != IOMUX_INVALID_ID ? id[0] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_UARTE_PSEL_TXD,
              id[1] != IOMUX_INVALID_ID ? id[1] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_UARTE_PSEL_RTS,
              id[2] != IOMUX_INVALID_ID ? id[2] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_UARTE_PSEL_CTS,
              id[3] != IOMUX_INVALID_ID ? id[3] : (uint32_t)-1);

  pv->has_ctsrts = id[2] != IOMUX_INVALID_ID && id[3] != IOMUX_INVALID_ID;
  config.flow_ctrl = pv->has_ctsrts;

  nrf_reg_set(pv->addr, NRF_UARTE_ENABLE, NRF_UARTE_ENABLE_ENABLED);

  nrf_it_disable_mask(pv->addr, -1);

  nrf_event_clear(pv->addr, NRF_UARTE_ENDRX);
  nrf_event_clear(pv->addr, NRF_UARTE_ENDTX);
  nrf_event_clear(pv->addr, NRF_UARTE_ERROR);
  nrf_event_clear(pv->addr, NRF_UARTE_RXTO);

  CPU_INTERRUPT_RESTORESTATE;

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_uarte_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_queue;

#if defined(CONFIG_DEVICE_UART)
  struct dev_resource_s *r = device_res_get(dev, DEV_RES_UART, 0);

  if (r) {
    config.baudrate    = r->u.uart.baudrate;
    config.data_bits   = r->u.uart.data_bits;
    config.stop_bits   = r->u.uart.stop_bits;
    config.parity      = r->u.uart.parity;
    config.flow_ctrl   = r->u.uart.flow_ctrl;
  }
#endif

  nrf5x_uarte_config(pv, &config);

  pv->must_flush = 0;

  return 0;

 free_queue:
  dev_request_queue_destroy(&pv->rx_q);
  dev_request_queue_destroy(&pv->tx_q);

 free_pv:
  mem_free(pv);

  return -1;
}

static DEV_CLEANUP(nrf5x_uarte_char_cleanup)
{
  struct nrf5x_uarte_priv *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->rx_q)
      || !dev_request_queue_isempty(&pv->tx_q))
    return -EBUSY;

  nrf_it_disable_mask(pv->addr, -1);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  nrf_task_trigger(pv->addr, NRF_UARTE_STOPRX);
  nrf_task_trigger(pv->addr, NRF_UARTE_STOPTX);

  nrf_reg_set(pv->addr, NRF_UARTE_PSEL_RXD, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_UARTE_PSEL_TXD, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_UARTE_PSEL_RTS, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_UARTE_PSEL_CTS, (uint32_t)-1);

  dev_request_queue_destroy(&pv->rx_q);
  dev_request_queue_destroy(&pv->tx_q);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_uarte_drv, 0, "nRF52 Serial"
#if defined(CONFIG_DEVICE_UART)
               ",UART"
#endif
               , nrf5x_uarte_char,
#if defined(CONFIG_DEVICE_UART)
               DRIVER_UART_METHODS(nrf5x_uarte_uart),
#endif
               DRIVER_CHAR_METHODS(nrf5x_uarte));

DRIVER_REGISTER(nrf5x_uarte_drv);
