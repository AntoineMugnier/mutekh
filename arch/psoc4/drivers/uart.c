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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
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
#include <device/clock.h>

#include <arch/psoc4/scb.h>

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#define GCT_CONTAINER_ALGO_uart_fifo RING

#include <gct_platform.h>
#include <gct/container_ring.h>

GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_NRF5X_UART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, isfull, pop, pop_array, pushback, pushback_array);

#define HW_FIFO_DEPTH 16
#define USE_HAS_REQ 1
#define OVS 8

struct psoc4_uart_pv_s
{
  uintptr_t addr;

  uart_fifo_root_t rx_fifo;
  uart_fifo_root_t tx_fifo;

  dev_request_queue_root_t rx_q;
  dev_request_queue_root_t tx_q;

  struct dev_irq_src_s irq_ep;

  struct dev_freq_s freq;
  struct dev_clock_sink_ep_s clock_sink;

  uint32_t baudrate;
  bool_t has_ctsrts;
};

static void psoc4_uart_rx_irq_disable(uintptr_t scb)
{
  dprintk("%s\n", __FUNCTION__);
  dwritek("r", 1);

  cpu_mem_write_32(scb + SCB_INTR_RX_MASK_ADDR, 0);
}

static void psoc4_uart_tx_irq_disable(uintptr_t scb)
{
  dprintk("%s\n", __FUNCTION__);
  dwritek("t", 1);

  cpu_mem_write_32(scb + SCB_INTR_TX_MASK_ADDR, 0);
}

static void psoc4_uart_rx_irq_enable(uintptr_t scb)
{
  dprintk("%s\n", __FUNCTION__);
  dwritek("R", 1);

  cpu_mem_write_32(scb + SCB_INTR_RX_MASK_ADDR, 0
                   | SCB_INTR_RX_MASK_NOT_EMPTY
                   );
}

static void psoc4_uart_tx_flush_irq_enable(uintptr_t scb)
{
  dprintk("%s\n", __FUNCTION__);
  dwritek("F", 1);

  cpu_mem_write_32(scb + SCB_INTR_TX_MASK_ADDR, 0
                   | SCB_INTR_TX_MASK_UART_DONE);
}

static void psoc4_uart_tx_irq_enable(uintptr_t scb)
{
  dprintk("%s\n", __FUNCTION__);
  dwritek("T", 1);

  cpu_mem_write_32(scb + SCB_INTR_TX_MASK_ADDR, 0
                   | SCB_INTR_TX_MASK_TRIGGER
                   );
}

static void psoc4_uart_start(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;

  if (dev->start_count)
    return;

  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_CLOCK);

  psoc4_uart_rx_irq_enable(pv->addr);
  cpu_mem_write_32(pv->addr + SCB_CTRL_ADDR,
                   cpu_mem_read_32(pv->addr + SCB_CTRL_ADDR) | SCB_CTRL_ENABLED);
}

static void psoc4_uart_stop(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;

  if (dev->start_count)
    return;

  cpu_mem_write_32(pv->addr + SCB_CTRL_ADDR,
                   cpu_mem_read_32(pv->addr + SCB_CTRL_ADDR) & ~SCB_CTRL_ENABLED);
  psoc4_uart_rx_irq_disable(pv->addr);
  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_NONE);
}

static bool_t psoc4_uart_tx_hwfifo_isfull(uintptr_t scb)
{
  uint32_t status = cpu_mem_read_32(scb + SCB_TX_FIFO_STATUS_ADDR);

  return SCB_TX_FIFO_STATUS_USED_GET(status) == HW_FIFO_DEPTH;
}

static bool_t psoc4_uart_rx_hwfifo_isempty(uintptr_t scb)
{
  uint32_t status = cpu_mem_read_32(scb + SCB_RX_FIFO_STATUS_ADDR);

  return SCB_RX_FIFO_STATUS_USED_GET(status) == 0;
}

static bool_t psoc4_uart_rx_hwfifo_isfull(uintptr_t scb)
{
  uint32_t status = cpu_mem_read_32(scb + SCB_RX_FIFO_STATUS_ADDR);

  return SCB_RX_FIFO_STATUS_USED_GET(status) == HW_FIFO_DEPTH;
}

static bool_t psoc4_uart_tx_hwfifo_isempty(uintptr_t scb)
{
  uint32_t status = cpu_mem_read_32(scb + SCB_TX_FIFO_STATUS_ADDR);

  return SCB_TX_FIFO_STATUS_USED_GET(status) == 0;
}

static bool_t psoc4_uart_tx_fifo_refill(struct device_s* dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  struct dev_char_rq_s *rq;
  struct dev_request_s *drq;
  size_t count;
  bool_t processed = 0;

  while ((drq = dev_request_queue_head(&pv->tx_q))) {
    rq = dev_char_rq_s_cast(drq);

    assert(rq->type == DEV_CHAR_WRITE_PARTIAL
           || rq->type == DEV_CHAR_WRITE_PARTIAL_FLUSH
           || rq->type == DEV_CHAR_WRITE_FLUSH
           || rq->type == DEV_CHAR_WRITE);

    count = uart_fifo_pushback_array(&pv->tx_fifo, rq->data, rq->size);

    dprintk("%s %p %d\n", __FUNCTION__, rq, count);

    if (!count)
      break;

    rq->data += count;
    rq->size -= count;

    processed = 1;

    if (rq->size == 0
        || rq->type == DEV_CHAR_WRITE_PARTIAL
        || rq->type == DEV_CHAR_WRITE_PARTIAL_FLUSH) {
      dev_request_queue_pop(&pv->tx_q);

      dwritek("(", 1);
      kroutine_exec(&rq->base.kr);
    }
  }

  return processed;
}

static bool_t psoc4_uart_rx_fifo_flush(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
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

      dwritek(")", 1);
      kroutine_exec(&rq->base.kr);
    }
  }

  return processed;
}

static void psoc4_uart_rx(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;

  dprintk("%s rx hwfifo %s, fifo %s\n", __FUNCTION__,
          psoc4_uart_rx_hwfifo_isempty(pv->addr) ? "empty" : "!empty",
          uart_fifo_isfull(&pv->rx_fifo) ? "full" : "!full");

  while (!psoc4_uart_rx_hwfifo_isempty(pv->addr)
         && !uart_fifo_isfull(&pv->rx_fifo)) {
    uint32_t chr = cpu_mem_read_32(pv->addr + SCB_RX_FIFO_RD_ADDR);

    dprintk("%s %02x\n", __FUNCTION__, chr);

    uart_fifo_pushback(&pv->rx_fifo, chr);
  }
}

static void psoc4_uart_tx(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;

  dprintk("%s tx hwfifo %s, fifo %s\n", __FUNCTION__,
          psoc4_uart_tx_hwfifo_isfull(pv->addr) ? "full" : "!full",
          uart_fifo_isempty(&pv->tx_fifo) ? "empty" : "!empty");

  while (!psoc4_uart_tx_hwfifo_isfull(pv->addr)
         && !uart_fifo_isempty(&pv->tx_fifo)) {
    uint32_t chr = uart_fifo_pop(&pv->tx_fifo);

    dprintk("%s %02x\n", __FUNCTION__, chr);

    cpu_mem_write_32(pv->addr + SCB_TX_FIFO_WR_ADDR, chr);
  }
}

static void psoc4_uart_process_one(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  uint32_t rx_cause;
  uint32_t tx_cause;
  bool_t changed;

 again:
  changed = 0;

  rx_cause = cpu_mem_read_32(pv->addr + SCB_INTR_RX_ADDR) & (0
                   | SCB_INTR_RX_MASK_NOT_EMPTY
                   );
  tx_cause = cpu_mem_read_32(pv->addr + SCB_INTR_TX_ADDR) & (0
                   | SCB_INTR_TX_MASK_UART_DONE
                   | SCB_INTR_TX_MASK_TRIGGER
                   | SCB_INTR_TX_MASK_EMPTY
                   );

  dprintk("%s tx cause: %x rx cause: %x\n", __FUNCTION__, tx_cause, rx_cause);

  // Ack everything
  cpu_mem_write_32(pv->addr + SCB_INTR_TX_ADDR, tx_cause);
  cpu_mem_write_32(pv->addr + SCB_INTR_RX_ADDR, rx_cause);

  psoc4_uart_tx(dev);
  psoc4_uart_rx(dev);

  changed |= psoc4_uart_tx_fifo_refill(dev);
  changed |= psoc4_uart_rx_fifo_flush(dev);

  dprintk("%s tx q %s, fifo %s, hwfifo %s\n", __FUNCTION__,
          dev_request_queue_isempty(&pv->tx_q) ? "empty" : "!empty",
          uart_fifo_isempty(&pv->tx_fifo) ? "empty" : "!empty",
          psoc4_uart_tx_hwfifo_isempty(pv->addr) ? "empty" : "!empty");

  if (dev_request_queue_isempty(&pv->tx_q) && uart_fifo_isempty(&pv->tx_fifo)) {
    if (psoc4_uart_tx_hwfifo_isempty(pv->addr)) {
      psoc4_uart_tx_irq_disable(pv->addr);
      device_sleep_schedule(dev);
    } else {
      psoc4_uart_tx_flush_irq_enable(pv->addr);
    }
  } else {
    psoc4_uart_tx_irq_enable(pv->addr);
  }

  dprintk("%s rx fifo %s, rx hwfifo %s\n", __FUNCTION__,
          uart_fifo_isfull(&pv->rx_fifo) ? "full" : "!full",
          psoc4_uart_rx_hwfifo_isfull(pv->addr) ? "full" : "!full");

  dwritek(psoc4_uart_rx_hwfifo_isfull(pv->addr) ? "[rxhF]" : "[rxhf]", 6);

  dprintk("[r%d]", SCB_RX_FIFO_STATUS_USED_GET(cpu_mem_read_32(pv->addr + SCB_RX_FIFO_STATUS_ADDR)));

  if (uart_fifo_isfull(&pv->rx_fifo)) {
    psoc4_uart_rx_irq_disable(pv->addr);
    if (psoc4_uart_rx_hwfifo_isfull(pv->addr))
      device_sleep_schedule(dev);
  } else {
    psoc4_uart_rx_irq_enable(pv->addr);
  }

  if (changed)
    goto again;
}

static DEV_IRQ_SRC_PROCESS(psoc4_uart_irq)
{
  struct device_s *dev = ep->base.dev;

  lock_spin(&dev->lock);
  psoc4_uart_process_one(dev);
  lock_release(&dev->lock);
}

static DEV_CHAR_REQUEST(psoc4_uart_request)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  dev_request_queue_root_t *q = NULL;

  dprintk("%s REQUEST %p, type %x, use %x, %P\n", __FUNCTION__, rq,
          rq->type,
          dev->start_count,
          rq->data, rq->size);

  switch (rq->type) {
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ:
    q = &pv->rx_q;
    dwritek(">", 1);
    break;

  case DEV_CHAR_WRITE_PARTIAL_FLUSH:
  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE:
    q = &pv->tx_q;
    dwritek("<", 1);
    break;

  default:
    rq->error = -ENOTSUP;
    kroutine_exec(&rq->base.kr);
    return;
  }

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  psoc4_uart_start(dev);

  dev->start_count |= USE_HAS_REQ;

  rq->error = 0;
  dev_request_queue_pushback(q, &rq->base);
  psoc4_uart_process_one(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_CHAR_CANCEL(psoc4_uart_cancel)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  dev_request_queue_root_t *q = NULL;
  error_t err;

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

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev_request_queue_head(q) == &rq->base) {
    err = -EBUSY;
  } else {
    dev_request_queue_remove(q, &rq->base);
    err = 0;
  }

  psoc4_uart_process_one(dev);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static void psoc4_uart_ratio_compute(struct device_s *dev,
                                     struct dev_freq_ratio_s *ratio)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  ratio->num = pv->freq.denom * OVS;
  ratio->denom = pv->freq.num / pv->baudrate;

  dprintk("%s freq %d/%d baudrate %d ratio %d/%d\n",
         __FUNCTION__, (uint32_t)pv->freq.num, (uint32_t)pv->freq.denom,
         pv->baudrate, (uint32_t)ratio->num, (uint32_t)ratio->denom);
}

static error_t psoc4_uart_config(struct device_s *dev,
                                 struct dev_uart_config_s *cfg)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  uintptr_t scb = pv->addr;

  uint32_t scb_ctrl;
  uint32_t uart_ctrl;
  uint32_t uart_tx_ctrl;
  uint32_t uart_rx_ctrl;
  uint32_t tx_ctrl;
  uint32_t rx_ctrl;
  uint32_t uart_flow_ctrl;
  uint32_t old_ctrl;

  if (cfg->data_bits < 3 || cfg->data_bits > 8)
    return -ENOTSUP;

  if (cfg->baudrate > 1000000)
    return -ENOTSUP;

  if (cfg->half_duplex)
    return -ENOTSUP;

  if (cfg->flow_ctrl && !pv->has_ctsrts)
    return -ENOTSUP;

  if (cfg->baudrate * OVS * pv->freq.denom > pv->freq.num)
    return -ENOTSUP;

  scb_ctrl = 0
    | SCB_CTRL_MODE(UART)
    | SCB_CTRL_BYTE_MODE
    | SCB_CTRL_OVS(OVS - 1)
    ;

  uart_ctrl = 0
    | SCB_UART_CTRL_MODE(STD)
    ;

  uart_flow_ctrl = 0
    ;

  uart_tx_ctrl = 0
    | SCB_UART_TX_CTRL_STOP_BITS(cfg->stop_bits * 2 - 1)
    ;

  uart_rx_ctrl = 0
    | SCB_UART_RX_CTRL_STOP_BITS(cfg->stop_bits * 2 - 1)
    | SCB_UART_RX_CTRL_BREAK_WIDTH(10 * 2 - 1)
    ;

  tx_ctrl = 0
    | SCB_TX_CTRL_DATA_WIDTH(cfg->data_bits - 1)
    ;

  rx_ctrl = 0
    | SCB_RX_CTRL_DATA_WIDTH(cfg->data_bits - 1)
    | (OVS == 16 ? SCB_RX_CTRL_MEDIAN : 0)
    ;

  switch (cfg->parity) {
  case DEV_UART_PARITY_NONE:
    break;
  case DEV_UART_PARITY_ODD:
    uart_rx_ctrl |= 0
      | SCB_UART_RX_CTRL_PARITY_EN
      | SCB_UART_RX_CTRL_PARITY(ODD);
    uart_tx_ctrl |= 0
      | SCB_UART_TX_CTRL_PARITY_EN
      | SCB_UART_TX_CTRL_PARITY(ODD);
    break;
  case DEV_UART_PARITY_EVEN:
    uart_rx_ctrl |= 0
      | SCB_UART_RX_CTRL_PARITY_EN
      | SCB_UART_RX_CTRL_PARITY(EVEN);
    uart_tx_ctrl |= 0
      | SCB_UART_TX_CTRL_PARITY_EN
      | SCB_UART_TX_CTRL_PARITY(EVEN);
    break;
  }

  if (cfg->flow_ctrl)
    uart_flow_ctrl |= 0
      | SCB_UART_FLOW_CTRL_CTS_EN
      | SCB_UART_FLOW_CTRL_TRIGGER_LEVEL(HW_FIFO_DEPTH * 3 / 4);

  pv->baudrate = cfg->baudrate;

  struct dev_freq_ratio_s ratio;
  psoc4_uart_ratio_compute(dev, &ratio);
  dev_clock_sink_scaler_set(&pv->clock_sink, &ratio);

  old_ctrl = cpu_mem_read_32(scb + SCB_CTRL_ADDR);

  cpu_mem_write_32(scb + SCB_CTRL_ADDR, old_ctrl & ~SCB_CTRL_ENABLED);
  cpu_mem_write_32(scb + SCB_CTRL_ADDR, scb_ctrl & ~SCB_CTRL_ENABLED);
  cpu_mem_write_32(scb + SCB_UART_CTRL_ADDR, uart_ctrl);
  cpu_mem_write_32(scb + SCB_TX_CTRL_ADDR, tx_ctrl);
  cpu_mem_write_32(scb + SCB_RX_CTRL_ADDR, rx_ctrl);
  cpu_mem_write_32(scb + SCB_UART_TX_CTRL_ADDR, uart_tx_ctrl);
  cpu_mem_write_32(scb + SCB_UART_RX_CTRL_ADDR, uart_rx_ctrl);
  cpu_mem_write_32(scb + SCB_UART_FLOW_CTRL_ADDR, uart_flow_ctrl);
  cpu_mem_write_32(scb + SCB_TX_FIFO_CTRL_ADDR, 0
                   | SCB_TX_FIFO_CTRL_TRIGGER_LEVEL(HW_FIFO_DEPTH / 4)
                   );

  cpu_mem_write_32(scb + SCB_RX_FIFO_CTRL_ADDR, 0
                   );
  cpu_mem_write_32(scb + SCB_CTRL_ADDR, scb_ctrl | (old_ctrl & SCB_CTRL_ENABLED));

  return 0;
}

#if defined(CONFIG_DEVICE_UART)
static DEV_UART_CONFIG(psoc4_uart_uart_config)
{
  struct device_s *dev = accessor->dev;
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);
  err = psoc4_uart_config(dev, cfg);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}
#endif

static DEV_USE(psoc4_uart_char_use)
{
  switch (op) {
  case DEV_USE_START: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct psoc4_uart_pv_s *pv = dev->drv_pv;

    dprintk("UART start cnt %d fifo full %d\n",
           dev->start_count, uart_fifo_isfull(&pv->rx_fifo));

    if (!dev->start_count && !uart_fifo_isfull(&pv->rx_fifo))
      psoc4_uart_start(dev);
      
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
    struct psoc4_uart_pv_s *pv = dev->drv_pv;

    if (dev_request_queue_isempty(&pv->rx_q)
        && dev_request_queue_isempty(&pv->tx_q)) {
      dev->start_count &= ~USE_HAS_REQ;
      psoc4_uart_stop(dev);
    }

    return 0;
  }

  case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
    struct dev_clock_notify_s *notify = param;
    struct dev_clock_sink_ep_s *sink = notify->sink;
    struct device_s *dev = sink->dev;
    struct psoc4_uart_pv_s *pv = dev->drv_pv;
    struct dev_freq_ratio_s ratio;

    dprintk("UART clock notify %d/%d\n",
           (uint32_t)notify->freq.num, (uint32_t)notify->freq.denom);

    pv->freq = notify->freq;

    psoc4_uart_ratio_compute(dev, &ratio);
    dev_clock_notify_scaler_set(notify, &ratio);

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static void psoc4_uart_scb_init(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  uintptr_t scb = pv->addr;

  // Enable UART mode, OVS x oversampling
  cpu_mem_write_32(scb + SCB_CTRL_ADDR, 0
                   | SCB_CTRL_MODE(UART)
                   | SCB_CTRL_BYTE_MODE
                   | SCB_CTRL_OVS(OVS - 1)
                   );

  // STD Submode
  cpu_mem_write_32(scb + SCB_UART_CTRL_ADDR, 0
                   | SCB_UART_CTRL_MODE(STD)
                   );
}

static void psoc4_uart_scb_cleanup(struct device_s *dev)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;
  uintptr_t scb = pv->addr;

  // Disable UART
  cpu_mem_write_32(scb + SCB_CTRL_ADDR, 0);
}

static DEV_INIT(psoc4_uart_char_init)
{
  struct psoc4_uart_pv_s *pv;
  iomux_io_id_t id[4];
  struct dev_uart_config_s config = {
    .baudrate = 1000000,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = DEV_UART_PARITY_NONE,
    .flow_ctrl = 0,
    .half_duplex = 0,
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

  pv->has_ctsrts = id[2] != IOMUX_INVALID_ID && id[3] != IOMUX_INVALID_ID;
  config.flow_ctrl = pv->has_ctsrts;

  device_irq_source_init(dev, &pv->irq_ep, 1, &psoc4_uart_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_pv;

  if (dev_drv_clock_init(dev, &pv->clock_sink, 0,
                         DEV_CLOCK_EP_GATING_SYNC | DEV_CLOCK_EP_FREQ_NOTIFY | DEV_CLOCK_EP_VARFREQ,
                         &pv->freq))
    goto unlink_irq;


  dprintk("UART clock init freq %d/%d\n",
         (uint32_t)pv->freq.num, (uint32_t)pv->freq.denom);

  dev_request_queue_init(&pv->rx_q);
  dev_request_queue_init(&pv->tx_q);

  uart_fifo_init(&pv->rx_fifo);
  uart_fifo_init(&pv->tx_fifo);

#if defined(CONFIG_DEVICE_UART)
  /* If there is a config resource, apply it. */
  struct dev_resource_s *r = device_res_get(dev, DEV_RES_UART, 0);

  if (r) {
    config.baudrate    = r->u.uart.baudrate;
    config.data_bits   = r->u.uart.data_bits;
    config.stop_bits   = r->u.uart.stop_bits;
    config.parity      = r->u.uart.parity;
    config.flow_ctrl   = r->u.uart.flow_ctrl && pv->has_ctsrts;
    config.half_duplex = r->u.uart.half_duplex;
  }
#endif

  psoc4_uart_scb_init(dev);

  assert(psoc4_uart_config(dev, &config) == 0);

  return 0;

 free_queue:
  uart_fifo_destroy(&pv->tx_fifo);
  uart_fifo_destroy(&pv->rx_fifo);

  dev_request_queue_destroy(&pv->rx_q);
  dev_request_queue_destroy(&pv->tx_q);

 unlink_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

 free_pv:
  mem_free(pv);

  return -1;
}

static DEV_CLEANUP(psoc4_uart_char_cleanup)
{
  struct psoc4_uart_pv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->rx_q)
      || !dev_request_queue_isempty(&pv->tx_q))
    return -EBUSY;

  psoc4_uart_scb_cleanup(dev);

  psoc4_uart_tx_irq_disable(pv->addr);
  psoc4_uart_rx_irq_disable(pv->addr);

  cpu_mem_write_32(pv->addr + SCB_CTRL_ADDR, 0);

  dev_drv_clock_cleanup(dev, &pv->clock_sink);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  uart_fifo_destroy(&pv->tx_fifo);
  uart_fifo_destroy(&pv->rx_fifo);

  dev_request_queue_destroy(&pv->rx_q);
  dev_request_queue_destroy(&pv->tx_q);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(psoc4_uart_drv, 0, "SCB Serial"
#if defined(CONFIG_DEVICE_UART)
               "+UART"
#endif
               , psoc4_uart_char,
#if defined(CONFIG_DEVICE_UART)
               DRIVER_UART_METHODS(psoc4_uart_uart),
#endif
               DRIVER_CHAR_METHODS(psoc4_uart));

DRIVER_REGISTER(psoc4_uart_drv);
