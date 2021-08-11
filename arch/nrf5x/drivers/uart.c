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

    Copyright
        Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#define LOGK_MODULE_ID "nrfu"

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
#include <device/resource/uart.h>
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
# include <device/class/valio.h>
# include <device/valio/uart_config.h>
#endif
#include <device/class/iomux.h>
#include <device/clock.h>

#include <arch/nrf5x/uart.h>
#include <arch/nrf5x/gpio.h>

#include "printk.h"

#define USE_RX 1
#define USE_TX 2

#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
# include <gct_platform.h>
# include <gct/container_ring.h>

# define GCT_CONTAINER_ALGO_uart_fifo RING

GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_NRF5X_UART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, isfull, pop, pop_array, pushback, pushback_array);
#endif

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
static PRINTK_HANDLER(nrf5x_uart_printk);
#endif

DRIVER_PV(struct nrf5x_uart_priv
{
#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
  struct printk_backend_s printk;
#endif

  uintptr_t addr;

#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
  uart_fifo_root_t rx_fifo;
  uart_fifo_root_t tx_fifo;
#endif

  /* tty input request queue and char fifo */
  dev_request_queue_root_t rx_q;
  dev_request_queue_root_t tx_q;

  struct dev_irq_src_s irq_ep;

  bool_t has_ctsrts:1;
  bool_t txdrdy:1;
  bool_t rxdrdy:1;
});

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
STRUCT_COMPOSE(nrf5x_uart_priv, printk)
#endif

static void nrf5x_uart_request_finish(struct device_s *dev,
                                      struct dev_char_rq_s *rq)
{
  rq->error = 0;
  logk_trace("rq %p done", rq);
  dev_char_rq_done(rq);
}

static bool_t nrf5x_uart_tx_fifo_refill(
    struct device_s *dev,
    struct nrf5x_uart_priv *pv)
{
    struct dev_char_rq_s *rq;
    size_t count;
    bool_t processed = 0;

    while ((rq = dev_char_rq_head(&pv->tx_q))) {

        assert(rq->type == DEV_CHAR_WRITE_PARTIAL
               || rq->type == DEV_CHAR_WRITE_PARTIAL_FLUSH
               || rq->type == DEV_CHAR_WRITE_FLUSH
               || rq->type == DEV_CHAR_WRITE);

        count = uart_fifo_pushback_array(&pv->tx_fifo, rq->data, rq->size);

        logk_trace("%s %p %d", __FUNCTION__, rq, count);

        if (!count)
            break;

        rq->data += count;
        rq->size -= count;

        processed = 1;

        if (rq->size == 0
            || rq->type == DEV_CHAR_WRITE_PARTIAL
            || rq->type == DEV_CHAR_WRITE_PARTIAL_FLUSH) {
            dev_char_rq_pop(&pv->tx_q);

            nrf5x_uart_request_finish(dev, rq);
        }
    }

    return processed;
}

static bool_t nrf5x_uart_rx_fifo_flush(
    struct device_s *dev,
    struct nrf5x_uart_priv *pv)
{
    struct dev_char_rq_s *rq;
    size_t count;
    bool_t processed = 0;

    while ((rq = dev_char_rq_head(&pv->rx_q))) {

        assert(rq->type == DEV_CHAR_READ_PARTIAL || rq->type == DEV_CHAR_READ);

        count = uart_fifo_pop_array(&pv->rx_fifo, rq->data, rq->size);

        logk_trace("%s %d", __FUNCTION__, count);

        if (!count)
            break;

        rq->data += count;
        rq->size -= count;

        processed = 1;

        if (rq->size == 0 || rq->type == DEV_CHAR_READ_PARTIAL) {
            dev_char_rq_pop(&pv->rx_q);

            device_sleep_schedule(dev);

            nrf5x_uart_request_finish(dev, rq);
        }
    }

    return processed;
}

static bool_t nrf5x_io_process_one(
    struct device_s *dev,
    struct nrf5x_uart_priv *pv)
{
    bool_t processed = 0;
    size_t popped_count;
    uint8_t chr;

    logk_trace("%s %p", __FUNCTION__, pv->addr);

 rx_again:
    if (nrf_event_check(pv->addr, NRF_UART_RXDRDY)) {
      nrf_event_clear(pv->addr, NRF_UART_RXDRDY);
      pv->rxdrdy = 1;

      logk_trace("%s rxdrdy", __FUNCTION__);
    }

    while (pv->rxdrdy && !uart_fifo_isfull(&pv->rx_fifo)) {
        processed = 1;
        pv->rxdrdy = 0;

        uint8_t chr = nrf_reg_get(pv->addr, NRF_UART_RXD);
        logk_trace("%s rx 0x%02x '%c'", __FUNCTION__, chr, chr < ' ' ? '.' : chr);

        uart_fifo_pushback(&pv->rx_fifo, chr);
    }

    if (!dev_rq_queue_isempty(&pv->rx_q))
      processed |= nrf5x_uart_rx_fifo_flush(dev, pv);

    if (!dev_rq_queue_isempty(&pv->tx_q))
      processed |= nrf5x_uart_tx_fifo_refill(dev, pv);

 tx_again:
    if (nrf_event_check(pv->addr, NRF_UART_TXDRDY)) {
      nrf_event_clear(pv->addr, NRF_UART_TXDRDY);
      pv->txdrdy = 1;

      logk_trace("%s txdrdy", __FUNCTION__);
    }

    if (pv->txdrdy && !uart_fifo_isempty(&pv->tx_fifo)) {
        popped_count = uart_fifo_pop_array(&pv->tx_fifo, &chr, 1);

        assert(popped_count);

        logk_trace("%s tx 0x%02x '%c'", __FUNCTION__, chr, chr < ' ' ? '.' : chr);

        processed = 1;
        pv->txdrdy = 0;
        nrf_reg_set(pv->addr, NRF_UART_TXD, chr);

        if (uart_fifo_isempty(&pv->tx_fifo))
          device_sleep_schedule(dev);

        goto tx_again;
    }

    return processed;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_uart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_uart_priv *pv = dev->drv_pv;

  LOCK_SPIN_SCOPED(&dev->lock);

  while (nrf5x_io_process_one(dev, pv))
    ;
}

static DEV_CHAR_REQUEST(nrf5x_uart_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_uart_priv *pv = dev->drv_pv;
  dev_request_queue_root_t *q = NULL;
  uint16_t use = 0;

  logk_trace("%s REQUEST %p, type %x, use %x, %P", __FUNCTION__, rq,
          rq->type,
          dev->start_count & (USE_TX | USE_RX),
          rq->data, rq->size);

  switch (rq->type) {
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ:
    q = &pv->rx_q;
    use = USE_RX;
    break;

  case DEV_CHAR_WRITE_PARTIAL_FLUSH:
  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE:
    q = &pv->tx_q;
    use = USE_TX;
    break;

  default:
    rq->error = -ENOTSUP;
    dev_char_rq_done(rq);
    return;
  }

  assert(rq->size);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  rq->error = 0;

  dev_char_rq_pushback(q, rq);
  if (!(dev->start_count & use)) {
    if (use == USE_RX) {
      logk_trace("%s START RX", __FUNCTION__);
      nrf_task_trigger(pv->addr, NRF_UART_STARTRX);
    } else {
      logk_trace("%s START TX", __FUNCTION__);
      nrf_task_trigger(pv->addr, NRF_UART_STARTTX);
    }
    dev->start_count |= use;
  }

  nrf5x_io_process_one(dev, pv);
}

static DEV_CHAR_CANCEL(nrf5x_uart_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_uart_priv *pv = dev->drv_pv;
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

  if (dev_char_rq_head(q) == rq)
    return -EBUSY;

  dev_char_rq_remove(q, rq);
  device_sleep_schedule(dev);

  return 0;
}

static error_t nrf5x_uart_config(
    struct nrf5x_uart_priv *pv,
    struct dev_uart_config_s *cfg)
{
    uint32_t config = 0;
    uintptr_t baudrate = cfg->baudrate;

    if (cfg->data_bits != 8)
        return -ENOTSUP;

    if (baudrate > 1000000)
        return -ENOTSUP;

    if (cfg->stop_bits != 1)
        return -ENOTSUP;

    switch  (cfg->parity) {
    case DEV_UART_PARITY_NONE:
        config |= NRF_UART_CONFIG_PARITY_DISABLED;
        break;
    case DEV_UART_PARITY_ODD:
        return -ENOTSUP;
    case DEV_UART_PARITY_EVEN:
        config |= NRF_UART_CONFIG_PARITY_ENABLED;
        break;
    }

    if (cfg->flow_ctrl) {
        if (!pv->has_ctsrts)
            return -ENOTSUP;

        config |= NRF_UART_CONFIG_CTSRTS_ENABLED;
    }

    CPU_INTERRUPT_SAVESTATE_DISABLE;

    nrf_task_trigger(pv->addr, NRF_UART_STOPRX);
    nrf_task_trigger(pv->addr, NRF_UART_STOPTX);

    for (uint32_t i = 0; i < 0x1000; ++i)
        asm volatile("");

    nrf_reg_set(pv->addr, NRF_UART_CONFIG, config);
    nrf_reg_set(pv->addr, NRF_UART_BAUDRATE, NRF_UART_BAUDRATE_(baudrate));

    for (uint32_t i = 0; i < 0x1000; ++i)
        asm volatile("");

    CPU_INTERRUPT_RESTORESTATE;

    return 0;
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)

static DEV_VALIO_REQUEST(nrf5x_uart_valio_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_uart_priv *pv = dev->drv_pv;

  if (rq->type != DEVICE_VALIO_WRITE
      || rq->attribute != VALIO_UART_CONFIG) {
    rq->error = -ENOTSUP;
  } else {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    rq->error = nrf5x_uart_config(pv, rq->data);
  }

  dev_char_rq_done(rq);
}

#define nrf5x_uart_valio_cancel (dev_valio_cancel_t*)dev_driver_notsup_fcn
#endif


static DEV_USE(nrf5x_uart_char_use)
{
  switch (op)
    {
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct nrf5x_uart_priv *pv = dev->drv_pv;

      if (!dev->start_count)
        nrf_task_trigger(pv->addr, NRF_UART_STARTRX);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;

      if (!dev->start_count)
        device_sleep_schedule(dev);
      return 0;
    }

    case DEV_USE_SLEEP: {
      struct device_s *dev = param;
      struct nrf5x_uart_priv *pv = dev->drv_pv;
      uint16_t before = dev->start_count;

      if (dev_rq_queue_isempty(&pv->rx_q))
        dev->start_count &= ~USE_RX;

      if (dev_rq_queue_isempty(&pv->tx_q)
          && uart_fifo_isempty(&pv->tx_fifo)
          && pv->txdrdy)
        dev->start_count &= ~USE_TX;

      if (before & ~dev->start_count & USE_RX) {
        logk_trace("%s STOP RX", __FUNCTION__);
        nrf_task_trigger(pv->addr, NRF_UART_STOPRX);
      }

      if (before & ~dev->start_count & USE_TX) {
        logk_trace("%s STOP TX", __FUNCTION__);
        nrf_task_trigger(pv->addr, NRF_UART_STOPTX);
      }

      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(nrf5x_uart_char_init)
{
    struct nrf5x_uart_priv *pv;
    iomux_io_id_t id[4];
    struct dev_uart_config_s config = {
#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
        .baudrate = CONFIG_DRIVER_NRF5X_PRINTK_RATE,
#else
        .baudrate = 115200,
#endif
        .data_bits = 8,
        .stop_bits = 1,
        .parity = DEV_UART_PARITY_NONE,
        .flow_ctrl = 0,
    };

    uintptr_t addr;
    if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
        return -ENOENT;

    /* If there is a config resource, apply it. */
    struct dev_resource_s *r = device_res_get(dev, DEV_RES_UART, 0);

    if (r) {
        config.baudrate    = r->u.uart.baudrate;
        config.data_bits   = r->u.uart.data_bits;
        config.stop_bits   = r->u.uart.stop_bits;
        config.parity      = r->u.uart.parity;
        config.flow_ctrl   = r->u.uart.flow_ctrl;

# if defined(CONFIG_DRIVER_NRF5X_PRINTK)
        if (addr == CONFIG_MUTEK_PRINTK_ADDR &&
            config.baudrate != CONFIG_DRIVER_NRF5X_PRINTK_RATE)
          logk_warning("uart config overrides printk baudrate");
# endif
    }

    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    if (!pv)
        return -ENOMEM;

    dev->drv_pv = pv;
    memset(pv, 0, sizeof(*pv));
    pv->addr = addr;

    if (device_iomux_setup(dev, "<rx? >tx? >rts? <cts?", NULL, id, NULL))
        goto free_pv;

    CPU_INTERRUPT_SAVESTATE_DISABLE;

    nrf_reg_set(pv->addr, NRF_UART_ENABLE, 0);

    // Pin config is done by iomux driver, select pins in UART.
    nrf_reg_set(
        pv->addr, NRF_UART_PSELRXD,
        id[0] != IOMUX_INVALID_ID ? id[0] : (uint32_t)-1);
    nrf_reg_set(
        pv->addr, NRF_UART_PSELTXD,
        id[1] != IOMUX_INVALID_ID ? id[1] : (uint32_t)-1);
    nrf_reg_set(
        pv->addr, NRF_UART_PSELRTS,
        id[2] != IOMUX_INVALID_ID ? id[2] : (uint32_t)-1);
    nrf_reg_set(
        pv->addr, NRF_UART_PSELCTS,
        id[3] != IOMUX_INVALID_ID ? id[3] : (uint32_t)-1);

    pv->has_ctsrts = id[2] != IOMUX_INVALID_ID && id[3] != IOMUX_INVALID_ID;
    config.flow_ctrl = pv->has_ctsrts;

    nrf_reg_set(pv->addr, NRF_UART_ENABLE, NRF_UART_ENABLE_ENABLED);

    nrf_event_clear(pv->addr, NRF_UART_RXDRDY);
    nrf_event_clear(pv->addr, NRF_UART_TXDRDY);
    nrf_event_clear(pv->addr, NRF_UART_ERROR);
    nrf_event_clear(pv->addr, NRF_UART_RXTO);

    CPU_INTERRUPT_RESTORESTATE;

    dev_rq_queue_init(&pv->rx_q);
    dev_rq_queue_init(&pv->tx_q);

#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_init(&pv->rx_fifo);
#endif

# if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_init(&pv->tx_fifo);
# endif

    nrf_it_disable_mask(pv->addr, -1);

    device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_uart_irq);

    if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
        goto free_queue;

    nrf5x_uart_config(pv, &config);

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
    if (pv->addr == CONFIG_MUTEK_PRINTK_ADDR)
      {
        nrf5x_printk_cleanup();
        printk_register(&pv->printk, nrf5x_uart_printk);
      }
#endif

    pv->txdrdy = 1;
    pv->rxdrdy = 0;
    nrf_it_enable(pv->addr, NRF_UART_TXDRDY);
    nrf_it_enable(pv->addr, NRF_UART_RXDRDY);

    return 0;

  free_queue:
#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_destroy(&pv->tx_fifo);
    uart_fifo_destroy(&pv->rx_fifo);
#endif

    dev_rq_queue_destroy(&pv->rx_q);
    dev_rq_queue_destroy(&pv->tx_q);

  free_pv:
    mem_free(pv);

    return -1;
}

static DEV_CLEANUP(nrf5x_uart_char_cleanup)
{
    struct nrf5x_uart_priv *pv = dev->drv_pv;
    uintptr_t addr = pv->addr;

    if (!dev_rq_queue_isempty(&pv->rx_q)
        || !dev_rq_queue_isempty(&pv->tx_q))
      return -EBUSY;

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
    if (addr == CONFIG_MUTEK_PRINTK_ADDR)
      printk_unregister(&pv->printk);
#endif

    nrf_it_disable_mask(addr, -1);

    device_irq_source_unlink(dev, &pv->irq_ep, 1);

    nrf_task_trigger(addr, NRF_UART_STOPRX);
    nrf_task_trigger(addr, NRF_UART_STOPTX);

    nrf_reg_set(addr, NRF_UART_PSELRXD, (uint32_t)-1);
    nrf_reg_set(addr, NRF_UART_PSELTXD, (uint32_t)-1);
    nrf_reg_set(addr, NRF_UART_PSELRTS, (uint32_t)-1);
    nrf_reg_set(addr, NRF_UART_PSELCTS, (uint32_t)-1);

#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_destroy(&pv->tx_fifo);
    uart_fifo_destroy(&pv->rx_fifo);
#endif

    dev_rq_queue_destroy(&pv->rx_q);
    dev_rq_queue_destroy(&pv->tx_q);

    device_iomux_cleanup(dev);
    mem_free(pv);

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
    if (addr == CONFIG_MUTEK_PRINTK_ADDR)
      nrf5x_printk_init();
#endif

    return 0;
}

DRIVER_DECLARE(nrf5x_uart_drv, 0, "nRF5x Serial"
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
               ",UART"
#endif
               , nrf5x_uart_char,
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
               DRIVER_VALIO_METHODS(nrf5x_uart_valio),
#endif
               DRIVER_CHAR_METHODS(nrf5x_uart));

DRIVER_REGISTER(nrf5x_uart_drv);

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)

static PRINTK_HANDLER(nrf5x_uart_printk)
{
  struct nrf5x_uart_priv *pv = nrf5x_uart_priv_from_printk(backend);
  struct device_s *dev = pv->irq_ep.base.dev;

  if (!len)
    return;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  bool_t enabled = dev->start_count & USE_TX;

  if (enabled) {
    if (pv->txdrdy) {
      nrf_it_disable(pv->addr, NRF_UART_TXDRDY);
    } else {
      while (!nrf_event_check(pv->addr, NRF_UART_TXDRDY))
        ;
    }
  } else {
    nrf_it_disable(pv->addr, NRF_UART_TXDRDY);
    nrf_task_trigger(pv->addr, NRF_UART_STARTTX);
  }

  nrf5x_printk_out(pv->addr, str, len);

  pv->txdrdy = 1;

  if (!enabled)
    nrf_task_trigger(pv->addr, NRF_UART_STOPTX);

  nrf_it_enable(pv->addr, NRF_UART_TXDRDY);

  CPU_INTERRUPT_RESTORESTATE;
}

#endif
