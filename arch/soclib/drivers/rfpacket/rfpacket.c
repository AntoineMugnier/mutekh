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

   Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

 */

#include <stdbool.h>
#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/class/timer.h>
#include <device/class/rfpacket.h>

#include "rfpacket_regs.h"

//#define SOCLIB_RFP_DEBUG
#define SOCLIB_RFP_THRESHOLD 8

#define SOCLIB_TRANSCEIVER_TX_IRQ_MASK (SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_DONE | \
    SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_TIMEOUT | \
    SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_UDF | \
    SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_CCA)

#define SOCLIB_TRANSCEIVER_RX_IRQ_MASK (SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_DONE |\
    SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_OVF | \
    SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_TIMEOUT | \
    SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_CRC | \
    SOCLIB_TRANSCEIVER_IRQ_STATUS_JAMMING)

#ifdef SOCLIB_RFP_DEBUG
# define soclib_rfp_printk(...) do { printk(__VA_ARGS__); } while(0)
#else
# define soclib_rfp_printk(...) do { } while(0)
#endif


DRIVER_PV(struct rfp_soclib_ctx_s
{
  struct device_s               *dev;
  /* Timer queue */
  dev_request_pqueue_root_t     timer_queue;
  /* queue for RX buffers */
  uintptr_t                     addr;
  struct dev_irq_src_s          irq_ep;
  struct dev_freq_s             timer_freq;
  int16_t                       curr_chan;
  uint32_t                      curr_freq;
  /* Control struct */
  struct dev_rfpacket_ctx_s gctx;
  const struct dev_rfpacket_pk_cfg_s  *pk_cfg;
  const struct dev_rfpacket_rf_cfg_s  *rf_cfg;
  enum dev_timer_capabilities_e cap;
  /* Driver flags */
  uint8_t flags;
  /* Request kroutine */
  struct kroutine_s             kr;
});

STRUCT_COMPOSE(rfp_soclib_ctx_s, kr);

#define SOCLIB_RFP_FLAGS_RX_TX_OK  0x01 // rx during tx_lbt is possible

/* Functions prototypes */
static inline dev_timer_value_t soclib_rfp_get_timer(struct rfp_soclib_ctx_s *pv);
static error_t soclib_rfp_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value);

static inline dev_timer_value_t soclib_rfp_get_timestamp(struct rfp_soclib_ctx_s *pv);
static inline void soclib_rfp_set_status(struct rfp_soclib_ctx_s *pv, enum dev_rfpacket_status_s status);
static void soclib_rfp_req_done(struct rfp_soclib_ctx_s *pv);
static void soclib_rfp_read_packet(struct rfp_soclib_ctx_s *pv);
static void soclib_rfp_cancel_done(struct rfp_soclib_ctx_s *pv);

static error_t soclib_rfp_dynamic_rf_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static error_t soclib_rfp_dynamic_pk_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static error_t soclib_rfp_build_rf_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static error_t soclib_rfp_build_pk_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static void soclib_rfp_tx_irq(struct rfp_soclib_ctx_s *pv, uint32_t irq);
static void soclib_rfp_rx_irq(struct rfp_soclib_ctx_s *pv, uint32_t irq);

static error_t soclib_rfp_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
static void soclib_rfp_start_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void soclib_rfp_start_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void soclib_rfp_cancel_rxc(struct dev_rfpacket_ctx_s *gpv);
static bool_t soclib_rfp_wakeup(struct dev_rfpacket_ctx_s *gpv);
static bool_t soclib_rfp_sleep(struct dev_rfpacket_ctx_s *gpv);
static void soclib_rfp_idle(struct dev_rfpacket_ctx_s *gpv);


static const struct dev_rfpacket_driver_interface_s soclib_rfp_itfc = {
  soclib_rfp_get_time,
  soclib_rfp_check_config,
  soclib_rfp_start_rx,
  soclib_rfp_start_tx,
  soclib_rfp_cancel_rxc,
  soclib_rfp_wakeup,
  soclib_rfp_sleep,
  soclib_rfp_idle,
};

/**************************** TIMER PART ********************************/

static inline dev_timer_value_t soclib_rfp_get_timer(struct rfp_soclib_ctx_s *pv)
{
  soclib_rfp_printk("drv: Get timer value\n");

  dev_timer_value_t v = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_TIMER_CNT_MSB_ADDR));
  v <<= 32;
  return (v | endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_TIMER_CNT_LSB_ADDR)));
}

static void soclib_rfp_set_timer(struct rfp_soclib_ctx_s *pv, dev_timer_value_t value)
{
  soclib_rfp_printk("drv: Set timer value %lld\n", value);

  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_TIMER_DEADLINE_MSB_ADDR,
                   endian_le32(value >> 32));

  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_TIMER_DEADLINE_LSB_ADDR,
                   endian_le32(value));
}

static void soclib_rfp_timer_process_queue(struct rfp_soclib_ctx_s *pv)
{
  while (1)
    {
      uint64_t value = soclib_rfp_get_timer(pv);
      struct dev_timer_rq_s *rq = dev_timer_rq_head(&pv->timer_queue);
      if (!rq)
        return;

      if (rq->deadline <= value)
        {
          dev_timer_rq_remove(&pv->timer_queue, rq);
          rq->base.drvdata = NULL;
          dev_timer_rq_done(rq);
        }
      else
        {
          soclib_rfp_set_timer(pv, rq->deadline);
          break;
        }
    }
}

static DEV_TIMER_CANCEL(rfpacket_soclib_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == pv)
    {
      bool_t first = (dev_timer_rq_prev(&pv->timer_queue, rq) == NULL);

      dev_timer_rq_remove(&pv->timer_queue, rq);
      rq->base.drvdata = NULL;

      if (first)
      /* Start next request if removed request is the first queue element */
        soclib_rfp_timer_process_queue(pv);

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

static DEV_TIMER_REQUEST(rfpacket_soclib_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != 1)
    err = -EAGAIN;
  else
    {
      uint64_t value = soclib_rfp_get_timer(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          dev->start_count |= 1;
          dev_timer_rq_insert(&pv->timer_queue, rq);
          rq->base.drvdata = pv;

          if (dev_timer_rq_prev(&pv->timer_queue, rq) == NULL)
            soclib_rfp_set_timer(pv, rq->deadline);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

static DEV_TIMER_GET_VALUE(rfpacket_soclib_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = soclib_rfp_get_timer(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(rfpacket_soclib_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;
  uint32_t r;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  cfg->freq = pv->timer_freq;

  if (res && res != 1)
    err = -ERANGE;
  else
    r = 1;

  if (cfg)
    {
      cfg->rev = 1;
      cfg->res = r;
      cfg->cap = pv->cap;
      cfg->max = 0xffffffffffffffffULL;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static error_t soclib_rfp_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value) {
  struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  *value = soclib_rfp_get_timer(pv);
  return 0;
}

/**************************** RFPACKET PART ********************************/

static inline dev_timer_value_t soclib_rfp_get_timestamp(struct rfp_soclib_ctx_s *pv)
{
  dev_timer_value_t v = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_TIMESTAMP_MSB_ADDR));
  v <<= 32;
  return (v | endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_TIMESTAMP_LSB_ADDR)));
}

static inline void soclib_rfp_set_status(struct rfp_soclib_ctx_s *pv, enum dev_rfpacket_status_s status) {
  pv->gctx.status = status;
}

static KROUTINE_EXEC(soclib_rfp_end_rq)
{
  struct rfp_soclib_ctx_s *pv = rfp_soclib_ctx_s_from_kr(kr);

  soclib_rfp_printk("req done\n");
  dev_rfpacket_req_done(&pv->gctx);
}

static void soclib_rfp_req_done(struct rfp_soclib_ctx_s *pv)
{
  /* Be sure to use soclib_rfp_set_status before calling this function */
  kroutine_init_deferred(&pv->kr, &soclib_rfp_end_rq);
  kroutine_exec(&pv->kr);
}

static void soclib_rfp_read_packet(struct rfp_soclib_ctx_s *pv)
{
  struct dev_rfpacket_rx_s *rx = pv->gctx.rxrq;
  uint8_t *rxbuf = (uint8_t *)rx->buf;

  soclib_rfp_printk("drv: RX size %d\n", rx->size);

  /* Set rx info */
  rx->frequency = pv->curr_freq;
  rx->rssi = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_RSSI_ADDR));
  rx->snr = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_SNR_ADDR));
  rx->carrier = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_CARRIER_ADDR));

  /* Read packet */
  for (uint16_t i = 0; i < rx->size; i++)
    rxbuf[i] = cpu_mem_read_8(pv->addr + SOCLIB_TRANSCEIVER_FIFO_ADDR);

  /* End request */
  soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_RX_DONE);
  soclib_rfp_req_done(pv);
}

static KROUTINE_EXEC(soclib_rfp_rx_kr)
{
  struct rfp_soclib_ctx_s *pv = rfp_soclib_ctx_s_from_kr(kr);
  soclib_rfp_read_packet(pv);
}

static void soclib_rfp_cancel_done(struct rfp_soclib_ctx_s *pv)
{
  soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_MISC);
  soclib_rfp_req_done(pv);
}



static error_t soclib_rfp_dynamic_rf_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  /* Retrieve data struct */
  const struct dev_rfpacket_rf_cfg_fairtx_s *fairtx = NULL;
  const struct dev_rfpacket_rf_cfg_std_s *common = NULL;
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = NULL;
  const struct dev_rfpacket_rf_cfg_ask_s *cask = NULL;

  switch (cfg->mod)
  {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      cfsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(cfg);
      common = &cfsk->common;
      fairtx = &cfsk->fairtx;
    break;

    case DEV_RFPACKET_ASK:
      cask = const_dev_rfpacket_rf_cfg_ask_s_cast(cfg);
      common = &cask->common;
      fairtx = &cask->fairtx;
    break;

    default:
      return -ENOTSUP;
    break;
  }
  /* Get common parameters */
  assert(common);
  uint32_t frequency = common->frequency + common->chan_spacing * rq->channel;
  pv->curr_freq = frequency;
  uint32_t rx_bw = common->rx_bw;

  /* Set specific parameters */
  if (cfsk != NULL)
  {
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_RF_MOD0_ADDR, endian_le32(cfsk->symbols));
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_RF_MOD1_ADDR, endian_le32(cfsk->deviation));

    if (rx_bw == 0)
      rx_bw = common->drate + cfsk->deviation * 2;
  }
  else if (cask != NULL)
  {
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_RF_MOD0_ADDR, endian_le32(cask->symbols));

    if (rx_bw == 0)
      rx_bw = common->drate * 2;
  }

  /* Set fair tx pramaters */
  if (fairtx != NULL)
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_RSSI_ADDR, endian_le32(fairtx->lbt.rssi));

  /* Set common parameters */
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_DATA_RATE_ADDR, endian_le32(common->drate));
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_BANDWITH_ADDR, endian_le32(rx_bw));
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_RSSI_JAM_ADDR, endian_le32(common->jam_rssi));
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_FREQUENCY_ADDR, endian_le32(frequency));

  return 0;
}

static error_t soclib_rfp_dynamic_pk_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{

  const struct dev_rfpacket_pk_cfg_s *cfg = rq->pk_cfg;

  /* Retrieve data struct */
  const struct dev_rfpacket_pk_cfg_basic_s *cbasic = NULL;

  switch (cfg->format) {
    case DEV_RFPACKET_FMT_SLPC:
      cbasic = const_dev_rfpacket_pk_cfg_basic_s_cast(cfg);
    break;

    default:
      return -ENOTSUP;
    break;
  }

  /* Set parameters */
  if (cbasic != NULL)
  {
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_FILTER_MSB_ADDR, 0);
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_FILTER_LSB_ADDR, endian_le32(cbasic->sw_value));
  }

  return 0;
}

static error_t soclib_rfp_build_rf_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  soclib_rfp_printk("RF configuration\n");

  switch (cfg->mod)
  {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
    case DEV_RFPACKET_ASK:
      return soclib_rfp_dynamic_rf_config(pv, rq);
#endif

    case DEV_RFPACKET_MOD_STATIC:
    case DEV_RFPACKET_MOD_EXTERN:
    default:
      return -ENOTSUP;
  }
}

static error_t soclib_rfp_build_pk_config(struct rfp_soclib_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_pk_cfg_s *cfg = rq->pk_cfg;

  soclib_rfp_printk("PKT configuration\n");

  switch (cfg->format)
  {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
    case DEV_RFPACKET_FMT_SLPC:
      return soclib_rfp_dynamic_pk_config(pv, rq);
#endif

    case DEV_RFPACKET_FMT_STATIC:
    case DEV_RFPACKET_FMT_EXTERN:
    default:
      return -ENOTSUP;
  }
}


static void soclib_rfp_tx_irq(struct rfp_soclib_ctx_s *pv, uint32_t irq)
{
  /* Packet sent */
  if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_DONE)
    soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_TX_DONE);
  /* Tx error */
  else
    {
      soclib_rfp_printk("tx failed: %d\n", irq);

      /* Flush fifo */
      uint32_t x = endian_le32(SOCLIB_TRANSCEIVER_CTRL_FTX);
      cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, x);

      /* Set error status */
      if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_TIMEOUT)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_TX_TIMEOUT);
      else if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_CCA)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_OTHER_ERR);
      else if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_TX_UDF)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_OTHER_ERR);
      else
        UNREACHABLE();
    }

  /* End request */
  soclib_rfp_req_done(pv);
}

static void soclib_rfp_rx_irq(struct rfp_soclib_ctx_s *pv, uint32_t irq)
{
  /* Packet received */
  if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_DONE)
    {
      /* Retrieve size */
      uint32_t size = cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_LEN_ADDR);

      /* Ask for buffer allocation */
      pv->gctx.size = size;
      uintptr_t p = dev_rfpacket_alloc(&pv->gctx);

      /* Allocation fail */
      if (p == 0)
        {
          soclib_rfp_printk("rx alloc fail\n");

          /* FLush rx fifo  */
          uint32_t x = endian_le32(SOCLIB_TRANSCEIVER_CTRL_FRX);
          cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, x);

          /* End request */
          soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_OTHER_ERR);
          soclib_rfp_req_done(pv);
          return;
        }

      /* Warn of incoming packet */
      dev_rfpacket_packet_incoming(&pv->gctx);

      /* Ask to read outside of irq */
      kroutine_init_deferred(&pv->kr, &soclib_rfp_rx_kr);
      kroutine_exec(&pv->kr);
    }
  /* Rx error */
  else
    {
      soclib_rfp_printk("rx fail: %d\n", irq);

      /* Flush rx fifo */
      uint32_t x = endian_le32(SOCLIB_TRANSCEIVER_CTRL_FRX);
      cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, x);

      /* Set error status */
      if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_JAMMING)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_JAMMING_ERR);
      else if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_TIMEOUT)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_RX_TIMEOUT);
      else if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_CRC)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_CRC_ERR);
      else if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_RX_OVF)
        soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_OTHER_ERR);
      else
        UNREACHABLE();

      /* End request */
      soclib_rfp_req_done(pv);
    }
}




static error_t soclib_rfp_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq)
{
  struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  error_t err;
  bool config_changed = false;

  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR)
  {
    /* Set rx_tx flags */
    pv->flags &= ~SOCLIB_RFP_FLAGS_RX_TX_OK;
    /* Test if RX is allowed during TX */
    if (dev_rfpacket_can_rxtx(&pv->gctx, rq))
      pv->flags |= SOCLIB_RFP_FLAGS_RX_TX_OK;
  }
  if ((rq->rf_cfg != pv->rf_cfg) || rq->rf_cfg->cache.dirty || (rq->channel != pv->curr_chan))
  {
    err = soclib_rfp_build_rf_config(pv, rq);
    if (err != 0)
      return err;
    /* Clear cache flag WARNING MAKE SURE rq not in read-only memory or segfault! */
    ((struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg)->cache.dirty = 0;
    /* Update rf cfg values */
    pv->rf_cfg = rq->rf_cfg;
    pv->curr_chan = rq->channel;
    config_changed = true;

  }
  if ((rq->pk_cfg != pv->pk_cfg) || rq->pk_cfg->cache.dirty)
  {

    err = soclib_rfp_build_pk_config(pv, rq);
    if (err != 0)
      return err;
    /* Clear cache flag WARNING MAKE SURE rq not in read-only memory or segfault! */
    ((struct dev_rfpacket_pk_cfg_s *)rq->pk_cfg)->cache.dirty = 0;
    /* Update pk cfg values */
    pv->pk_cfg = rq->pk_cfg;
    config_changed = true;
  }
  /* Start up configuration */
  if (config_changed)
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, endian_le32(SOCLIB_TRANSCEIVER_CTRL_START_CFG));

  return 0;
}

static void soclib_rfp_start_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry)
{
  soclib_rfp_printk("drv: Write %d bytes to TX fifo\n", rq->tx_size);
  struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  const uint8_t *txbuf = rq->tx_buf;

  if (isRetry)
  {
    uint32_t x;

    switch (rq->type)
    {
      case DEV_RFPACKET_RQ_TX_FAIR:
      /* Clear rx fifo */
      x = endian_le32(SOCLIB_TRANSCEIVER_CTRL_FRX);
      cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, x);
      break;

      default:
        UNREACHABLE();
    }
  }
  else
  {
    switch (rq->type)
      {
        case DEV_RFPACKET_RQ_TX:
          cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_TIMEOUT_ADDR, endian_le32(0));
          break;
        case DEV_RFPACKET_RQ_TX_FAIR:
          cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_TIMEOUT_ADDR, endian_le32(rq->lifetime));
          break;
        default:
          UNREACHABLE();
      }
    /* Fill TX fifo */
    for(uint16_t i = 0; i < rq->tx_size; i++)
      cpu_mem_write_8(pv->addr + SOCLIB_TRANSCEIVER_FIFO_ADDR, txbuf[i]);

    /* Set size */
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_LEN_ADDR, endian_le32(rq->tx_size));

    /* Set deadline */
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_DEADLINE_MSB_ADDR, endian_le32(rq->deadline >> 32));
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_DEADLINE_LSB_ADDR, endian_le32(rq->deadline));

    /* Set power */
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_PWR_ADDR, endian_le32(rq->tx_pwr));
  }
  /* Start tx */
  uint32_t x = SOCLIB_TRANSCEIVER_CTRL_START_TX;
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, endian_le32(x));
}

static void soclib_rfp_start_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry)
{
  soclib_rfp_printk("drv: Start RX with timeout: %d\n", rq->lifetime);
  struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  dev_timer_value_t t = soclib_rfp_get_timer(pv);
  dev_timer_delay_t lifetime = 0;
  dev_timer_value_t deadline = 0;

  /* Init rx params if not retry */
  if (!isRetry)
  {
    switch (rq->type)
    {
      case DEV_RFPACKET_RQ_RX_TIMEOUT:
        assert(rq->deadline > t);
        lifetime = rq->deadline - t;
      break;

      case DEV_RFPACKET_RQ_RX_CONT:
      break;

      case DEV_RFPACKET_RQ_RX:
        assert(rq->lifetime);
        lifetime = rq->lifetime;
      break;

      default:
        UNREACHABLE();
      break;
    }

    /* Set time values */
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_TIMEOUT_ADDR, endian_le32(lifetime));
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_DEADLINE_MSB_ADDR, endian_le32(deadline >> 32));
    cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_DEADLINE_LSB_ADDR, endian_le32(deadline));
  }

  /* Start Rx */
  uint32_t x = endian_le32(SOCLIB_TRANSCEIVER_CTRL_START_RX);
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, x);
}

static void soclib_rfp_cancel_rxc(struct dev_rfpacket_ctx_s *gpv)
{
  struct rfp_soclib_ctx_s *pv = gpv->pvdata;

  /* Start cancel, wait for irq */
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_CTRL_ADDR, endian_le32(SOCLIB_TRANSCEIVER_CTRL_CANCEL));
}

static bool_t soclib_rfp_wakeup(struct dev_rfpacket_ctx_s *gpv)
{
  // TODO
  //struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  return false;
}

static bool_t soclib_rfp_sleep(struct dev_rfpacket_ctx_s *gpv)
{
  // TODO
  //struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  return false;
}

static void soclib_rfp_idle(struct dev_rfpacket_ctx_s *gpv)
{
  // TODO
  //struct rfp_soclib_ctx_s *pv = gpv->pvdata;
  return;
}





static DEV_IRQ_SRC_PROCESS(rfpacket_soclib_irq)
{
  struct device_s *dev = ep->base.dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  /* Set timestamp */
  pv->gctx.timestamp = soclib_rfp_get_timestamp(pv);

  while(1)
    {
       uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_IRQ_STATUS_ADDR));
       irq &= endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_TRANSCEIVER_IEN_ADDR));

       soclib_rfp_printk("drv: irq: 0x%x\n", irq);

       if (!irq)
         break;

       if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_TIMER)
          soclib_rfp_timer_process_queue(pv);

       if (irq & SOCLIB_TRANSCEIVER_IRQ_STATUS_CANCEL)
         soclib_rfp_cancel_done(pv);

       if (irq & SOCLIB_TRANSCEIVER_RX_IRQ_MASK)
         soclib_rfp_rx_irq(pv, irq);

       if (irq & SOCLIB_TRANSCEIVER_TX_IRQ_MASK)
         soclib_rfp_tx_irq(pv, irq);
    }

  lock_release(&dev->lock);
}

static DEV_RFPACKET_REQUEST(rfpacket_soclib_request)
{
  struct device_s *dev = accessor->dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  va_list vl;
  va_start(vl, accessor);

  while(1)
  {
    struct dev_rfpacket_rq_s *rq = va_arg(vl, struct dev_rfpacket_rq_s *);

    if (rq == NULL)
      break;

    rq->error = 0;
    dev_rfpacket_request(&pv->gctx, rq);
  }
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_RFPACKET_CANCEL(rfpacket_soclib_cancel)
{
  struct device_s *dev = accessor->dev;
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;
  error_t err = 0;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);
  err = dev_rfpacket_cancel(&pv->gctx, rq);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define rfpacket_soclib_use dev_use_generic
#define rfpacket_soclib_stats (dev_rfpacket_stats_t*)&dev_driver_notsup_fcn

static DEV_INIT(rfpacket_soclib_init);
static DEV_CLEANUP(rfpacket_soclib_cleanup);

DRIVER_DECLARE(rfpacket_soclib_drv, 0, "SoCLib RFpacket", rfpacket_soclib,
    DRIVER_RFPACKET_METHODS(rfpacket_soclib),
    DRIVER_TIMER_METHODS(rfpacket_soclib_timer));

DRIVER_REGISTER(rfpacket_soclib_drv,
    DEV_ENUM_FDTNAME_ENTRY("soclib:rfpacket"));

static DEV_INIT(rfpacket_soclib_init)
{
  struct rfp_soclib_ctx_s	*pv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  dev->drv_pv = pv;
  pv->dev = dev;

  /* Timer info */
  pv->cap = DEV_TIMER_CAP_HIGHRES |
            DEV_TIMER_CAP_REQUEST |
            DEV_TIMER_CAP_KEEPVALUE;


  pv->timer_freq.num = 1000000;
  pv->timer_freq.denom = 1;

  device_get_res_freq(dev, &pv->timer_freq, 0);

  assert(pv->timer_freq.denom == 1);

  soclib_rfp_printk("freq: %lld\n", pv->timer_freq.num);

  /* Period in us */
  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_PERIOD_ADDR, endian_le32(1000000000/pv->timer_freq.num));

  /* Enable interrupt on device */
  uint32_t x =  SOCLIB_TRANSCEIVER_IEN_TX_DONE  |
                SOCLIB_TRANSCEIVER_IEN_TX_TIMEOUT |
                SOCLIB_TRANSCEIVER_IEN_RX_DONE  |
                SOCLIB_TRANSCEIVER_IEN_CANCEL  |
                SOCLIB_TRANSCEIVER_IEN_RX_TIMEOUT |
                SOCLIB_TRANSCEIVER_IEN_JAMMING |
                SOCLIB_TRANSCEIVER_IEN_TIMER;


  cpu_mem_write_32(pv->addr + SOCLIB_TRANSCEIVER_IEN_ADDR, endian_le32(x));

  dev_rq_pqueue_init(&pv->timer_queue);

  device_irq_source_init(dev, &pv->irq_ep, 1,
      &rfpacket_soclib_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;

  /* Init generic context */
  pv->gctx.pvdata = pv;
  pv->gctx.drv = &soclib_rfp_itfc;
  dev_rfpacket_init(&pv->gctx);

  /* Indicate that we finished init */
  soclib_rfp_printk("init end\n");
  soclib_rfp_set_status(pv, DEV_RFPACKET_STATUS_MISC);
  soclib_rfp_req_done(pv);

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(rfpacket_soclib_cleanup)
{
  struct rfp_soclib_ctx_s *pv = dev->drv_pv;
  error_t err = dev_rfpacket_clean_check(&pv->gctx);

  if (!err)
  {
    device_irq_source_unlink(dev, &pv->irq_ep, 1);
    dev_rq_pqueue_destroy(&pv->timer_queue);
    dev_rfpacket_clean(&pv->gctx);
    mem_free(pv);
  }
  return err;
}