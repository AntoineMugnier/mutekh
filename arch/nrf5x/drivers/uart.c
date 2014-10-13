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
#include <device/class/clock.h>

#include <arch/nrf5x/uart.h>
#include <arch/nrf5x/gpio.h>

#if !defined(CONFIG_DRIVER_NRF5X_PRINTK) && 0
# define dprintk printk
#else
# define dprintk(...) do{}while(0)
#endif


#if defined(CONFIG_DRIVER_NRF5X_UART)
#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
# include <gct_platform.h>
# include <gct/container_ring.h>

# define GCT_CONTAINER_ALGO_uart_fifo RING

GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_NRF5X_UART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, isfull, pop, pop_array, pushback, pushback_array);
#endif

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
static PRINTF_OUTPUT_FUNC(nrf5x_printk_out);
PRINTF_OUTPUT_FUNC(nrf5x_printk_out_nodrv);
#endif

struct nrf5x_uart_priv
{
    uintptr_t addr;

#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_root_t rx_fifo;
    uart_fifo_root_t tx_fifo;
#endif

    /* tty input request queue and char fifo */
    dev_request_queue_root_t rx_q;
    dev_request_queue_root_t tx_q;

    struct dev_irq_src_s irq_ep;

    bool_t callbacking:1;
    bool_t has_ctsrts:1;
    bool_t txdrdy:1;
    bool_t rxdrdy:1;
};

static void nrf5x_uart_request_finish(
    struct device_s *dev,
    struct dev_char_rq_s *rq)
{
  struct nrf5x_uart_priv *pv = dev->drv_pv;

  pv->callbacking = 1;
  lock_release(&dev->lock);

  rq->error = 0;
  dprintk("rq %p done\n", rq);
  kroutine_exec(&rq->base.kr);

  lock_spin(&dev->lock);
  pv->callbacking = 0;
}

static bool_t nrf5x_uart_tx_fifo_refill(
    struct device_s *dev,
    struct nrf5x_uart_priv *pv)
{
    struct dev_char_rq_s *rq;
    struct dev_request_s *drq;
    size_t count;
    bool_t processed = 0;

    while ((drq = dev_request_queue_head(&pv->tx_q))) {
        rq = dev_char_rq_s_cast(drq);

        assert(rq->type == DEV_CHAR_WRITE_PARTIAL || rq->type == DEV_CHAR_WRITE);

        count = uart_fifo_pushback_array(&pv->tx_fifo, rq->data, rq->size);

        dprintk("%s %p %d\n", __FUNCTION__, rq, count);

        if (!count)
            break;

        rq->data += count;
        rq->size -= count;

        processed = 1;

        if (rq->size == 0 || rq->type == DEV_CHAR_WRITE_PARTIAL) {
            dev_request_queue_pop(&pv->tx_q);

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
    struct dev_request_s *drq;
    size_t count;
    bool_t processed = 0;

    while ((drq = dev_request_queue_head(&pv->rx_q))) {
        rq = dev_char_rq_s_cast(drq);

        assert(rq->type == DEV_CHAR_READ_PARTIAL || rq->type == DEV_CHAR_READ);

        count = uart_fifo_pop_array(&pv->rx_fifo, rq->data, rq->size);

        dprintk("%s %d\n", __FUNCTION__, count);

        if (!count)
            break;

        rq->data += count;
        rq->size -= count;

        processed = 1;

        if (rq->size == 0 || rq->type == DEV_CHAR_READ_PARTIAL) {
            dev_request_queue_pop(&pv->rx_q);

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

    dprintk("%s %p\n", __FUNCTION__, pv->addr);

 rx_again:
    if (nrf_event_check(pv->addr, NRF_UART_RXDRDY)) {
      nrf_event_clear(pv->addr, NRF_UART_RXDRDY);
      pv->rxdrdy = 1;

      dprintk("%s rxdrdy\n", __FUNCTION__);
    }

    while (pv->rxdrdy && !uart_fifo_isfull(&pv->rx_fifo)) {
        processed = 1;
        pv->rxdrdy = 0;

        uint8_t chr = nrf_reg_get(pv->addr, NRF_UART_RXD);
        dprintk("%s rx 0x%02x '%c'\n", __FUNCTION__, chr, chr < ' ' ? '.' : chr);

        uart_fifo_pushback(&pv->rx_fifo, chr);
    }

    if (!dev_request_queue_isempty(&pv->rx_q))
      processed |= nrf5x_uart_rx_fifo_flush(dev, pv);

    if (!dev_request_queue_isempty(&pv->tx_q))
      processed |= nrf5x_uart_tx_fifo_refill(dev, pv);

 tx_again:
    if (nrf_event_check(pv->addr, NRF_UART_TXDRDY)) {
      nrf_event_clear(pv->addr, NRF_UART_TXDRDY);
      pv->txdrdy = 1;
    }

    if (pv->txdrdy && !uart_fifo_isempty(&pv->tx_fifo)) {
        popped_count = uart_fifo_pop_array(&pv->tx_fifo, &chr, 1);

        assert(popped_count);

        dprintk("%s tx 0x%02x '%c'\n", __FUNCTION__, chr, chr < ' ' ? '.' : chr);

        processed = 1;
        pv->txdrdy = 0;
        nrf_reg_set(pv->addr, NRF_UART_TXD, chr);

        goto tx_again;
    }

    if (dev_request_queue_isempty(&pv->rx_q)) {
      dprintk("%s STOP RX\n", __FUNCTION__);
      nrf_task_trigger(pv->addr, NRF_UART_STOPRX);
    }

    if (dev_request_queue_isempty(&pv->tx_q)
        && uart_fifo_isempty(&pv->tx_fifo)
        && pv->txdrdy) {
      dprintk("%s STOP TX\n", __FUNCTION__);
      nrf_task_trigger(pv->addr, NRF_UART_STOPTX);
    }

    return processed;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_uart_irq)
{
    struct device_s *dev = ep->base.dev;
    struct nrf5x_uart_priv *pv = dev->drv_pv;

    lock_spin(&dev->lock);

    while (nrf5x_io_process_one(dev, pv))
        ;

    lock_release(&dev->lock);
}

static DEV_CHAR_REQUEST(nrf5x_uart_request)
{
    struct device_s *dev = accessor->dev;
    struct nrf5x_uart_priv *pv = dev->drv_pv;
    dev_request_queue_root_t *q = NULL;
    bool_t empty;

    assert(rq->size);

    dprintk("%s %p %d %d %d\n", __FUNCTION__, rq, dev_request_queue_isempty(&pv->rx_q),
            rq->type, rq->size);

    switch (rq->type) {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      q = &pv->rx_q;
      break;

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
      q = &pv->tx_q;
      break;
    }

    if (!q)
      return -ENOTSUP;

    LOCK_SPIN_IRQ(&dev->lock);

    empty = dev_request_queue_isempty(q);
    dev_request_queue_pushback(q, &rq->base);

    if (empty) {
      switch (rq->type) {
      case DEV_CHAR_READ_PARTIAL:
      case DEV_CHAR_READ:
        dprintk("%s START RX\n", __FUNCTION__);
        nrf_task_trigger(pv->addr, NRF_UART_STARTRX);
        break;

      case DEV_CHAR_WRITE_PARTIAL:
      case DEV_CHAR_WRITE:
        dprintk("%s START TX\n", __FUNCTION__);
        nrf_task_trigger(pv->addr, NRF_UART_STARTTX);
        break;
      }

      if (!pv->callbacking)
        nrf5x_io_process_one(dev, pv);
    }

    LOCK_RELEASE_IRQ(&dev->lock);
}

static error_t nrf5x_uart_config(
    struct nrf5x_uart_priv *pv,
    struct dev_uart_config_s *cfg)
{
    uint32_t config = 0;
    uintptr_t baudrate = cfg->baudrate;

    CPU_INTERRUPT_SAVESTATE_DISABLE;

    if (cfg->data_bits != 8)
        return -ENOTSUP;

    if (baudrate > 1000000)
        return -ENOTSUP;

    if (cfg->stop_bits != 1)
        return -ENOTSUP;

    if (cfg->half_duplex)
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

#if defined(CONFIG_DEVICE_UART)

static DEV_UART_CONFIG(nrf5x_uart_uart_config)
{
    struct device_s *dev = accessor->dev;
    struct nrf5x_uart_priv *pv = dev->drv_pv;

    error_t err = nrf5x_uart_config(pv, cfg);

    return err;
}

#endif

static DEV_INIT(nrf5x_uart_char_init);
static DEV_CLEANUP(nrf5x_uart_char_cleanup);

#define nrf5x_uart_char_use dev_use_generic

DRIVER_DECLARE(nrf5x_uart_drv, 0, "nRF5x Serial"
#if defined(CONFIG_DEVICE_UART)
               ",UART"
#endif
               , nrf5x_uart_char,
#if defined(CONFIG_DEVICE_UART)
               DRIVER_UART_METHODS(nrf5x_uart_uart),
#endif
               DRIVER_CHAR_METHODS(nrf5x_uart));

DRIVER_REGISTER(nrf5x_uart_drv);

static DEV_INIT(nrf5x_uart_char_init)
{
    struct nrf5x_uart_priv *pv;
    iomux_io_id_t id[4];
    struct dev_uart_config_s config = {
        .baudrate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = DEV_UART_PARITY_NONE,
        .flow_ctrl = 0,
        .half_duplex = 0,
    };

    dev->status = DEVICE_DRIVER_INIT_FAILED;

    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    dev->drv_pv = pv;

    memset(pv, 0, sizeof(*pv));

    if (!pv)
        return -ENOMEM;

    if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
        goto free_pv;

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

    dev_request_queue_init(&pv->rx_q);
    dev_request_queue_init(&pv->tx_q);

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

# if defined(CONFIG_DRIVER_NRF5X_PRINTK)
    config.baudrate = CONFIG_DRIVER_NRF5X_PRINTK_RATE;
# endif

#if defined(CONFIG_DEVICE_UART)
    /* If there is a config resource, apply it. */
    struct dev_resource_s *r = device_res_get(dev, DEV_RES_UART, 0);

    if (r) {
        config.baudrate    = r->u.uart.baudrate;
        config.data_bits   = r->u.uart.data_bits;
        config.stop_bits   = r->u.uart.stop_bits;
        config.parity      = r->u.uart.parity;
        config.flow_ctrl   = r->u.uart.flow_ctrl;
        config.half_duplex = r->u.uart.half_duplex;
    }
#endif

    nrf5x_uart_config(pv, &config);

    dev->drv = &nrf5x_uart_drv;
    dev->status = DEVICE_DRIVER_INIT_DONE;

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)
    if (pv->addr == CONFIG_MUTEK_PRINTK_ADDR)
      printk_set_output(nrf5x_printk_out, pv);
#endif

    pv->txdrdy = 1;
    pv->rxdrdy = 0;
    pv->callbacking = 0;
    nrf_it_enable(pv->addr, NRF_UART_TXDRDY);
    nrf_it_enable(pv->addr, NRF_UART_RXDRDY);

    return 0;

  free_queue:
#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_destroy(&pv->tx_fifo);
    uart_fifo_destroy(&pv->rx_fifo);
#endif

    dev_request_queue_destroy(&pv->rx_q);
    dev_request_queue_destroy(&pv->tx_q);

  free_pv:
    mem_free(pv);

    return -1;
}

DEV_CLEANUP(nrf5x_uart_char_cleanup)
{
    struct nrf5x_uart_priv *pv = dev->drv_pv;

    nrf_it_disable_mask(pv->addr, -1);

    device_irq_source_unlink(dev, &pv->irq_ep, 1);

    nrf_task_trigger(pv->addr, NRF_UART_STOPRX);
    nrf_task_trigger(pv->addr, NRF_UART_STOPTX);

    nrf_reg_set(pv->addr, NRF_UART_PSELRXD, (uint32_t)-1);
    nrf_reg_set(pv->addr, NRF_UART_PSELTXD, (uint32_t)-1);
    nrf_reg_set(pv->addr, NRF_UART_PSELRTS, (uint32_t)-1);
    nrf_reg_set(pv->addr, NRF_UART_PSELCTS, (uint32_t)-1);

#if CONFIG_DRIVER_NRF5X_UART_SWFIFO > 0
    uart_fifo_destroy(&pv->tx_fifo);
    uart_fifo_destroy(&pv->rx_fifo);
#endif

    dev_request_queue_destroy(&pv->rx_q);
    dev_request_queue_destroy(&pv->tx_q);

    mem_free(pv);
}

#if defined(CONFIG_DRIVER_NRF5X_PRINTK)

void nrf5x_printk_out_char(void *addr, char c);

static PRINTF_OUTPUT_FUNC(nrf5x_printk_out)
{
    struct nrf5x_uart_priv *pv = ctx;

    if (!len)
      return;

    dprintk("printk %d\n", len);

    CPU_INTERRUPT_SAVESTATE_DISABLE;

    bool_t enabled = !dev_request_queue_isempty(&pv->tx_q);

    if (enabled) {
      if (pv->txdrdy) {
        pv->txdrdy = 0;
      } else {
        while (!nrf_event_check(pv->addr, NRF_UART_TXDRDY))
          ;
      }
    } else {
      nrf_it_disable(pv->addr, NRF_UART_TXDRDY);
      nrf_task_trigger(pv->addr, NRF_UART_STARTTX);
    }

    nrf5x_printk_out_nodrv((void *)pv->addr, str, offset, len);

    if (!enabled) {
      nrf_event_clear(pv->addr, NRF_UART_TXDRDY);
      nrf_task_trigger(pv->addr, NRF_UART_STOPTX);
      nrf_it_enable(pv->addr, NRF_UART_TXDRDY);
    }

    CPU_INTERRUPT_RESTORESTATE;
}

#endif

#endif /* CONFIG_DRIVER_NRF5X_UART */
