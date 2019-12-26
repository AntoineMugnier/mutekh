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

   Copyright (c) 2017 Sebastien Cerdan <sebcerdan@gmail.com>

 */

#include <stdbool.h>
#include "common.h"
#include "protimer.h"

/* LBT parameters */
#define EFR32_ETSI_LBT_TIME              5000ULL     /*us*/
#define EFR32_MAX_LBT_RETRY_COUNT        4

/* Protimer Radio class parameters */
#define EFR32_PROTIMER_RX_START_CHANNEL  1
#define EFR32_PROTIMER_RX_STOP_CHANNEL   2
#define EFR32_PROTIMER_TX_CHANNEL        3

#define EFR32_RADIO_RFP_BUFFER_SIZE 256

DRIVER_PV(struct radio_efr32_rfp_ctx_s
{
  struct radio_efr32_ctx_s pv;
  uint8_t sg_buffer[EFR32_RADIO_RFP_BUFFER_SIZE];
  // Timer device struct
  struct device_timer_s timer_s;
  // Generic rfpacket context struct
  struct dev_rfpacket_ctx_s gctx;
  // Current config
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;
});

STRUCT_COMPOSE(radio_efr32_rfp_ctx_s, pv);

static void efr32_rfp_timer_irq(struct device_s *dev);
static error_t efr32_radio_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value);
static inline error_t efr32_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static inline error_t efr32_pkt_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static inline void  efr32_rfp_fill_status(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status);
static void efr32_rfp_req_done(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_set_cca_threshold(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static void efr32_rfp_start_tx_lbt(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static void efr32_rfp_disable(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_start_rx_scheduled(struct radio_efr32_rfp_ctx_s *ctx, dev_timer_value_t t);
static void efr32_rfp_read_packet(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_rx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq);
static inline void efr32_rfp_tx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq);
static error_t efr32_rfp_init(struct radio_efr32_rfp_ctx_s *ctx);
// Lib device rfpacket interface functions
static error_t efr32_radio_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
static void efr32_radio_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void efr32_radio_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void efr32_radio_cancel_rxc(struct dev_rfpacket_ctx_s *gpv);
static bool_t efr32_radio_wakeup(struct dev_rfpacket_ctx_s *gpv);
static bool_t efr32_radio_sleep(struct dev_rfpacket_ctx_s *gpv);
static void efr32_radio_idle(struct dev_rfpacket_ctx_s *gpv);

static const struct dev_rfpacket_driver_interface_s efr32_radio_itfc = {
  efr32_radio_get_time,
  efr32_radio_check_config,
  efr32_radio_rx,
  efr32_radio_tx,
  efr32_radio_cancel_rxc,
  efr32_radio_wakeup,
  efr32_radio_sleep,
  efr32_radio_idle,
};

#define efr32_radio_use dev_use_generic
#define efr32_radio_stats (dev_rfpacket_stats_t *)&dev_driver_notsup_fcn

/**************************** TIMER PART ********************************/

static DEV_TIMER_CANCEL(efr32_rfpacket_timer_cancel) {
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);
  if (rq->base.drvdata == ctx) {
    struct dev_timer_rq_s *rqnext = NULL;
    bool_t first = (dev_timer_rq_prev(&pv->pti.queue, rq) == NULL);

    if (first) {
      rqnext = dev_timer_rq_next(&pv->pti.queue, rq);
    }
    dev_timer_rq_remove(&pv->pti.queue, rq);
    rq->base.drvdata = NULL;

    if (first) {
      efr32_protimer_disable_compare(&pv->pti, EFR32_PROTIMER_CHANNEL);

      if (rqnext != NULL) {
        dev_timer_value_t value = efr32_protimer_get_value(&pv->pti);
        // start next request, raise irq on race condition
        efr32_protimer_request_start(&pv->pti, value, rqnext->deadline, EFR32_PROTIMER_CHANNEL);
      }
    }
    err = 0;
  }
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_REQUEST(efr32_rfpacket_timer_request) {
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct efr32_protimer_s *pti = &pv->pti;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  if (rq->rev && rq->rev != pti->rev) {
    err = -EAGAIN;
  } else {
    // Start timer if needed
    if (dev->start_count == 0) {
      efr32_protimer_start_counter(pti);
    }
    dev_timer_value_t value = efr32_protimer_get_value(pti);

    if (rq->delay) {
      rq->deadline = value + rq->delay;
    }
    if (rq->deadline <= value) {
      err = -ETIMEDOUT;
    } else {
      dev->start_count |= 1;
      dev_timer_rq_insert(&pti->queue, rq);
      rq->base.drvdata = ctx;
      // Start request, raise irq on race condition
      if (dev_timer_rq_prev(&pti->queue, rq) == NULL) {
        efr32_protimer_request_start(pti, value, rq->deadline, EFR32_PROTIMER_CHANNEL);
      }
    }
  }
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(efr32_rfpacket_timer_get_value) {
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  LOCK_SPIN_IRQ(&dev->lock);
  *value = efr32_protimer_get_value(&pv->pti);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(efr32_rfpacket_timer_config) {
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  uint32_t r = 1;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  cfg->freq = pv->freq;

  if (res) {
    if (dev->start_count) {
      err = -EBUSY;
      r = res;
    } else {
      // Resolution can not be modified
      if (res != 1) {
        err = -ERANGE;
      }
      pv->pti.rev += 2;
    }
  }
  if (cfg) {
    cfg->rev = pv->pti.rev;
    cfg->res = r;
    cfg->cap = pv->pti.cap;
    cfg->max = 0xffffffffffffffffULL;
  }
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static void efr32_rfp_timer_irq(struct device_s *dev) {
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct efr32_protimer_s *pti = &pv->pti;

  while (1) {
    uint32_t irq = endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IF_ADDR));
    irq &= endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR));

    if (!irq) {
      break;
    }
    cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFC_ADDR, endian_le32(irq));

    uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
    x = EFR32_RAC_STATUS_STATE_GET(x);
    // Tx timeout test
    if (irq & EFR32_PROTIMER_IF_LBTFAILURE) {
      // Check RAC state
      if (x != EFR32_RAC_STATUS_STATE_OFF) {
        efr32_rfp_disable(ctx);
      }
      efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_TX_TIMEOUT);
      efr32_rfp_req_done(ctx);
    }
    // Compare channel Rx end interrupt
    if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL)) {
      // Rx timeout
      efr32_protimer_disable_compare(pti, EFR32_PROTIMER_RX_STOP_CHANNEL);
      // Check RAC state
      if (x != EFR32_RAC_STATUS_STATE_OFF) {
        efr32_rfp_disable(ctx);
      }
      efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_RX_TIMEOUT);
      efr32_rfp_req_done(ctx);
    }
    // Timer class interrupts
    if (!(irq & (EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL) | EFR32_PROTIMER_IF_WRAPCNTOF))) {
      break;
    }
#if EFR32_PROTIMER_HW_WIDTH < 64
    // Compare channel interrupt
    if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL)) {
      efr32_protimer_disable_compare(pti, EFR32_PROTIMER_CHANNEL);
    }
    // Update the software part of the counter
    if (irq & EFR32_PROTIMER_IF_WRAPCNTOF) {
      pti->swvalue++;
    }
#endif
    // CC channel irq
    while (1) {
      struct dev_timer_rq_s *rq = dev_timer_rq_head(&pti->queue);
      if (rq == NULL) {
        break;
      }
      dev_timer_value_t value = efr32_protimer_get_value(pti);
      // Setup compare for first request
      if (rq->deadline > value) {
        if (!efr32_protimer_request_start(pti, value, rq->deadline, EFR32_PROTIMER_CHANNEL)) {
          break;
        }
      }
      dev_timer_rq_remove(&pti->queue, rq);
      efr32_protimer_disable_compare(pti, EFR32_PROTIMER_CHANNEL);
      rq->base.drvdata = NULL;

      lock_release(&dev->lock);
      dev_timer_rq_done(rq);
      lock_spin(&dev->lock);
    }
  }
}

static error_t efr32_radio_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  *value = efr32_protimer_get_value(&pv->pti);
  return 0;
}




/**************************** RFPACKET PART ********************************/

static inline error_t efr32_rf_config(struct radio_efr32_rfp_ctx_s *ctx,
                                      struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

  // Test if new RF configuration or previous configuration modified
  if ((rfcfg != ctx->rf_cfg) || rfcfg->cache.dirty) {
    if (rfcfg->drate != 38400) {
     return -ENOTSUP;
    }
    ctx->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)rfcfg;
    uint32_t div = cpu_mem_read_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR);
    uint64_t f = ((uint64_t)(rfcfg->frequency) * div) << 19;
    f /= EFR32_RADIO_HFXO_CLK;
    cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_FREQ_ADDR, (uint32_t)f);
    // FIXME FILL TIME BYTE INFO
    // dev_timer_delay_t tb = 8000000 / rfcfg->drate;
    // dev_timer_init_sec(ctx->gctx.timer, &(ctx->gctx.time_byte), 0, tb, 1000000);
  }
  return 0;
}

static inline error_t efr32_pkt_config(struct radio_efr32_rfp_ctx_s *ctx,
                                       struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;
  // Test if new Packet configuration or previous configuration modified
  if ((pkcfg == ctx->pk_cfg) && !pkcfg->cache.dirty) {
    return 0;
  }
  ctx->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pkcfg;
  if (rq->pk_cfg->format != DEV_RFPACKET_FMT_SLPC) {
    return -ENOTSUP;
  }
  const struct dev_rfpacket_pk_cfg_basic_s *cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);
  // Configure Sync Word
  uint8_t sw = cfg->sw_len + 1;
  if ((sw >> 5) || (sw % 4)) {
    return -ENOTSUP;
  }
  uint32_t x = EFR32_MODEM_CTRL1_SYNCBITS(cfg->sw_len);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL1_ADDR, x);

  x = cfg->sw_value;
  x = ((x & 0x55555555) << 1) | ((0xAAAAAAAA & x) >> 1);
  x = ((x & 0x33333333) << 2) | ((0xCCCCCCCC & x) >> 2);
  x = ((x & 0x0F0F0F0F) << 4) | ((0xF0F0F0F0 & x) >> 4);
  x = ((x & 0xFF00FF00) >> 8) | ((0x00FF00FF & x) << 8);
  x = ((x & 0xFFFF0000) >> 16) | ((0x0000FFFF & x) << 16);
  x = x >> (32 - sw);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SYNC0_ADDR, x);
  // Configure Preamble
  if ((cfg->pb_pattern_len + 1) >> 4) {
    return -ENOTSUP;
  }
  x = EFR32_MODEM_PRE_BASE(cfg->pb_pattern_len);
  uint32_t msk = (1 << (cfg->pb_pattern_len + 1)) - 1;
  uint32_t preamble = cfg->pb_pattern & msk;

  if (preamble == (0xAAAAAAAA & msk)) {
    EFR32_MODEM_PRE_BASEBITS_SET(x, 1); // TYPE 1010
  } else if (preamble == (0x55555555 & msk)) {
    EFR32_MODEM_PRE_BASEBITS_SET(x, 0); // TYPE 0101
  } else {
    return -ENOTSUP;
  }
  EFR32_MODEM_PRE_TXBASES_SET(x, cfg->tx_pb_len/(cfg->pb_pattern_len + 1));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_PRE_ADDR, x);
  // Configure CRC
  x =  EFR32_CRC_CTRL_BITSPERWORD(7) |
       EFR32_CRC_CTRL_INPUTBITORDER(MSB) |
       EFR32_CRC_CTRL_BITREVERSE(LSB);

  uint32_t w;

  switch (cfg->crc) {
    case 0:
    case 0x07:
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH8;
     break;

    case 0x8005:
    case 0x1021:
    case 0x3d65:
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH16;
    break;

    case 0x65b:
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH24;
    break;

    case 0x814141ab:
    case 0x04c11db7:
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH32;
    break;

    default:
      return -ENOTSUP;
  }
  x |= (w << EFR32_CRC_CTRL_CRCWIDTH_IDX);
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_CTRL_ADDR, x);
  // Reverse bits
  uint32_t v = cfg->crc;
  x = 0;

  for (uint8_t i = 0; i < ((w + 1) << 3); i++) {
      x <<= 1;
      x = v & 1 ? x | 1 : x;
      v >>= 1;
  }
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_POLY_ADDR, x);
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_INIT_ADDR, EFR32_CRC_INIT_INIT(cfg->crc_seed));
  return 0;
}

#ifdef CONFIG_DRIVER_EFR32_DEBUG
static void efr32_rfp_cfg_prs_dbg(struct radio_efr32_rfp_ctx_s *ctx) {
  uint32_t x;
  // Set PC9 to PC11 in output
  uint32_t a = 0x4000a008 + 2 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x444 << 4));
  // Configure PRS channel 9/10/11 on PC9/10/11
  x = EFR32_PRS_ROUTEPEN_CHPEN(9)  |
      EFR32_PRS_ROUTEPEN_CHPEN(10) |
      EFR32_PRS_ROUTEPEN_CHPEN(11);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTEPEN_ADDR, x);

  x = EFR32_PRS_ROUTELOC2_CH9LOC(14) |
      EFR32_PRS_ROUTELOC2_CH10LOC(4) |
      EFR32_PRS_ROUTELOC2_CH11LOC(4);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTELOC2_ADDR, x);
  // PC9
  x = EFR32_PRS_CH_CTRL_SOURCESEL(PROTIMERH);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 7); //PROTIMERCC1
  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(9), x);
  // PC10
  x = EFR32_PRS_CH_CTRL_SOURCESEL(PROTIMERH);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 2); //PROTIMERCC2
  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(10), x);
  // PC11
  x = EFR32_PRS_CH_CTRL_SOURCESEL(PROTIMERL);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 0); //PROTIMERPOF
  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(11), x);
}
#endif







static inline void  efr32_rfp_fill_status(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status) {
  ctx->gctx.status = status;
}

static KROUTINE_EXEC(efr32_radio_rq_done) {
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_radio_printk("req done\n");
  dev_rfpacket_req_done(pv->dev, &ctx->gctx);
}

static void efr32_rfp_req_done(struct radio_efr32_rfp_ctx_s *ctx) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  // Be sure to use efr32_rfp_fill_status before calling this function
  kroutine_init_deferred(&pv->kr, &efr32_radio_rq_done);
  kroutine_exec(&pv->kr);
}

static void efr32_rfp_set_cca_threshold(struct radio_efr32_rfp_ctx_s *ctx,
                                        struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_fsk_s * c = const_dev_rfpacket_rf_cfg_fsk_s_cast(rq->rf_cfg);
  int16_t r = c->fairtx.lbt.rssi >> 3;
  int8_t v = (r & 0x7F) | (r < 0 ? 0x80 : 0);

  // Saturate rssi threshold
  if (r < -128) {
    v = -128;
  }
  if (r > 127) {
    v = 127;
  }
  uint32_t x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_CCATHRSH_SET(x, v & 0xFF);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);
}

static void efr32_rfp_start_tx_lbt(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TOUTCNTTOP_ADDR(0), EFR32_PROTIMER_TOUTCNTTOP_PCNTTOP(4));
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TOUTCOMP_ADDR(0), EFR32_PROTIMER_TOUTCOMP_PCNTCOMP(1));

  // Time granularity is based on T0UF
  uint32_t x = // Time between 2 retries
               EFR32_PROTIMER_LBTCTRL_FIXEDBACKOFF |
               EFR32_PROTIMER_LBTCTRL_STARTEXP(1) |
               EFR32_PROTIMER_LBTCTRL_MAXEXP(1) |
               // Number of consecutive successful CCA before generating LBT success
               EFR32_PROTIMER_LBTCTRL_CCAREPEAT(1) |
               // Time between 2 CCA measures
               EFR32_PROTIMER_LBTCTRL_CCADELAY(1) |
               // Number of CCA failure before generating LBTFAILURE
               EFR32_PROTIMER_LBTCTRL_RETRYLIMIT(EFR32_MAX_LBT_RETRY_COUNT - 1);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_LBTCTRL_ADDR, x);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RANDOM_ADDR, 1);

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_LBTSTATE1_ADDR, EFR32_PROTIMER_LBTSTATE1_EXP(1));
#endif

  x = EFR32_PROTIMER_RXCTRL_RXSETEVENT(0, ALWAYS) |
      EFR32_PROTIMER_RXCTRL_RXCLREVENT(0, ALWAYS) |

      EFR32_PROTIMER_RXCTRL_RXSETEVENT(1, TOUT0MATCHLBT) |
      EFR32_PROTIMER_RXCTRL_RXCLREVENT(1, CCAACK);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RXCTRL_ADDR, x);

  x = EFR32_PROTIMER_TXCTRL_TXSETEVENT(0, ALWAYS) |
      EFR32_PROTIMER_TXCTRL_TXSETEVENT(1, LBTSUCCESS);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TXCTRL_ADDR, x);

  uint32_t nbsym = (EFR32_ETSI_LBT_TIME * rq->rf_cfg->drate)/1000000;
  nbsym = bit_ctz(pow2_up(nbsym));

  assert(nbsym < 15);
  // Configure AGC
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR);
  EFR32_AGC_CTRL0_MODE_SET(x, CONT);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR, x);

  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_RSSIPERIOD_SET(x, nbsym);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);

  efr32_rfp_set_cca_threshold(ctx, rq);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, EFR32_PROTIMER_CMD_LBTSTART);
}

static void efr32_rfp_disable(struct radio_efr32_rfp_ctx_s *ctx) {
  // Disable RX
  uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
  x &= ~0xF;

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, 0);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CMD_ADDR, EFR32_FRC_CMD_RXABORT);
  // Disable TX
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXDIS);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_FORCESTATE_ADDR, EFR32_RAC_STATUS_STATE_OFF);
  // Check RAC state
  while(1) {
    // FIXME Use state change interrupt here or schedule a timer interrupt
    x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
    if (EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF) {
      break;
    }
  }
  // Clear irq
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR, 0);
}

static void efr32_rfp_start_rx_scheduled(struct radio_efr32_rfp_ctx_s *ctx, dev_timer_value_t t) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;

#if EFR32_PROTIMER_HW_WIDTH < 64
  assert(0);
#endif
  dev_timer_value_t start = ctx->gctx.deadline;
  dev_timer_value_t end = ctx->gctx.timeout;

  uint32_t x = EFR32_PROTIMER_RXCTRL_RXSETEVENT(0, ALWAYS) |
               EFR32_PROTIMER_RXCTRL_RXCLREVENT(0, ALWAYS) |
               EFR32_PROTIMER_RXCTRL_RXSETEVENT(1, CC1)    |
               EFR32_PROTIMER_RXCTRL_RXCLREVENT(1, CC2);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RXCTRL_ADDR, x);
  // RX scheduled looks not to work as expected without minimal delay
  if ((start <= t) || (start - t) < 128) {
    start = t + 128;
  }
  efr32_protimer_request_start(&pv->pti, t, start, EFR32_PROTIMER_RX_START_CHANNEL);
  efr32_protimer_request_start(&pv->pti, t, end, EFR32_PROTIMER_RX_STOP_CHANNEL);
}





static void efr32_rfp_read_packet(struct radio_efr32_rfp_ctx_s *ctx) {
  struct dev_rfpacket_rx_s *rx = ctx->gctx.rxrq;
  uint8_t *p = (uint8_t *)rx->buf;

  rx->snr = 0;
  rx->carrier = 0;
  // Read packet
  for (uint16_t i = 0; i < rx->size; i++) {
    p[i] = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));
  }
  rx->rssi = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));
  // Call rfpacket to process request completion
  efr32_rfp_req_done(ctx);
}

static KROUTINE_EXEC(efr32_rfp_rx) {
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_rfp_read_packet(ctx);
}

static void efr32_rfp_rx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  x = EFR32_RAC_STATUS_STATE_GET(x);

  if (irq != EFR32_FRC_IF_RXDONE) {
    // Flush RX fifo
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);
    efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_OTHER_ERR); // FIXME SCAN ERRORS, CRC?
    efr32_rfp_req_done(ctx);
    return;
  }
  uint16_t size = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);

  uint32_t status = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_STATUS_ADDR(1));
  status = EFR32_BUFC_STATUS_BYTES_GET(status);

  assert(EFR32_BUFC_STATUS_BYTES_GET(status) >= size);
  // Asking for buffer allocation
  ctx->gctx.size = size;
  uintptr_t p = dev_rfpacket_alloc(&ctx->gctx);

  if (p != 0) {
    // Flush RX fifo
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);
    efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_OTHER_ERR);
    efr32_rfp_req_done(ctx);
    return;
  }
  // Status ok
  efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_RX_DONE);
  // Ask to read outside of irq
  kroutine_init_deferred(&pv->kr, &efr32_rfp_rx);
  kroutine_exec(&pv->kr);
}

static inline void efr32_rfp_tx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq) {
  // If tx is over
  if (irq & EFR32_FRC_IF_TXDONE) {
    // Set status
    efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_TX_DONE);
  } else {
    // Clear buffer
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR);
    efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_OTHER_ERR);
  }
  // Call rfpacket to process request completion
  efr32_rfp_req_done(ctx);
}





static error_t efr32_radio_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;

  if (efr32_pkt_config(ctx, rq) || efr32_rf_config(ctx, rq)) {
    // Unsupported configuration
    return -ENOTSUP;
  } else {
    return 0;
  }
}

static void efr32_radio_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  cpu_mem_write_32(EFR32_SEQ_DEADLINE_ADDR, 0);
  // Check RAC state
  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  assert(EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF);
  // Clear buffer
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);

  dev_timer_value_t t = efr32_protimer_get_value(&pv->pti);

  if (ctx->gctx.timeout < t) {
    // Deadline already reached
    efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_RX_TIMEOUT);
    efr32_rfp_req_done(ctx);
    return;
  }
  // Retry or not
  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX_CONT:
      // Enable RX
      x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
      x |= 0x2;
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
    break;

    case DEV_RFPACKET_RQ_RX:
      // Scheduled RX
      efr32_rfp_start_rx_scheduled(ctx, t);
    break;
    // TODO RQ RX TIMEOUT support
    default:
      assert(1);
  }
}

static void efr32_radio_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;

  cpu_mem_write_32(EFR32_SEQ_DEADLINE_ADDR, 0);
  // Check retry
  if (isRetry) {
    // TODO LBT FEATURES + LBT TIMESTAMP
    efr32_rfp_start_tx_lbt(ctx, rq);
  }
  // Check RAC state
  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  assert(EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF);
  // Clear buffer
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR);
  // Fill buffer
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), rq->tx_size);
  uint8_t *p = (uint8_t *)rq->tx_buf;
  for(uint16_t i = 0; i < rq->tx_size; i++) {
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), (uint32_t)p[i]);
  }
  // Set channel
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, rq->channel);
  // Enable TX
  switch (rq->type) {
    case DEV_RFPACKET_RQ_TX:
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXEN);
    break;

    case DEV_RFPACKET_RQ_TX_FAIR:
      efr32_rfp_start_tx_lbt(ctx, rq);
    break;

    default:
      abort();
  }
}

static void efr32_radio_cancel_rxc(struct dev_rfpacket_ctx_s *gpv) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  efr32_rfp_disable(ctx);
}

static bool_t efr32_radio_wakeup(struct dev_rfpacket_ctx_s *gpv) {
  //struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  return false; // TODO SUPPORT SLEEP
}

static bool_t efr32_radio_sleep(struct dev_rfpacket_ctx_s *gpv) {
  //struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  return false; // TODO SUPPORT SLEEP
}

static void efr32_radio_idle(struct dev_rfpacket_ctx_s *gpv) {
  //struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  return; // TODO SUPPORT SLEEP
}





static DEV_RFPACKET_REQUEST(efr32_radio_request) {
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  va_list vl;
  va_start(vl, accessor);

  while(1) {
    struct dev_rfpacket_rq_s *rq = va_arg(vl, struct dev_rfpacket_rq_s *);

    if (rq == NULL) {
      break;
    }
    efr32_radio_printk("R %d\n", rq->type);
    rq->error = 0;
    dev_rfpacket_request(&ctx->gctx, rq);
  }
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_RFPACKET_CANCEL(efr32_radio_cancel) {
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  error_t err = 0;

  efr32_radio_printk("cancel");
  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);
  err = dev_rfpacket_cancel(&ctx->gctx, rq);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_IRQ_SRC_PROCESS(efr32_radio_irq) {
  struct device_s *dev = ep->base.dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  uint32_t irq = 0;

  lock_spin(&dev->lock);
  // Set timestamp
  ctx->gctx.timestamp = efr32_protimer_get_value(&ctx->pv.pti);
  // Process irq
  switch (ep - ctx->pv.irq_ep) {
    case 0:
      irq = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IEN_ADDR);
      cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_IFC_ADDR, irq);
      efr32_radio_printk("modem irq: 0x%x\n", irq);
    break;

    case 1:
    case 2:
      irq = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_IEN_ADDR);
      efr32_radio_printk("rac irq: 0x%x\n", irq);
    break;

    case 3:
      irq = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IEN_ADDR);
      efr32_radio_printk("bufc irq: 0x%x\n", irq);
    break;

    case 7:
      efr32_rfp_timer_irq(dev);
    break;

    case 8:
      irq = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);

      if (irq & EFR32_TX_IRQ_FRC_MSK) {
        efr32_rfp_tx_irq(ctx, irq);
      } else if (irq & EFR32_RX_IRQ_FRC_MSK) {
        efr32_rfp_rx_irq(ctx, irq);
      }
      efr32_radio_printk("frc irq: 0x%x\n", irq);
      // Clear irq
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, irq);
    break;

    default:
      efr32_radio_printk("irq: %d\n", ep - ctx->pv.irq_ep);
      abort();
    break;
  }
  lock_release(&dev->lock);
}

static DEV_INIT(efr32_radio_init);
static DEV_CLEANUP(efr32_radio_cleanup);

DRIVER_DECLARE(efr32_radio_drv, 0, "EFR32 RFP radio", efr32_radio,
    DRIVER_RFPACKET_METHODS(efr32_radio),
    DRIVER_TIMER_METHODS(efr32_rfpacket_timer));

DRIVER_REGISTER(efr32_radio_drv,
    DEV_ENUM_FDTNAME_ENTRY("efr32:radio"));

static DEV_INIT(efr32_radio_init) {
  struct radio_efr32_rfp_ctx_s	*ctx;

  // Allocate private driver data */
  ctx = mem_alloc(sizeof(*ctx), (mem_scope_sys));
  if (!ctx) {
    return -ENOMEM;
  }
  memset(ctx, 0, sizeof(*ctx));
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  dev->drv_pv = ctx;
  pv->dev = dev;

  if (efr32_rfp_init(ctx)) {
    goto err_mem;
  }
  uint32_t x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR);

  x |=  EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL) |
        EFR32_PROTIMER_IF_LBTFAILURE;

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR, x);

  // TC0 synchronized on PRECNTOF, prescaler decremented on PRECNTOF
  x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR);
  x |= EFR32_PROTIMER_CTRL_TOUT_SRC(0, PRECNTOF);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR, x);

  pv->freq.num = EFR32_RADIO_HFXO_CLK;
  pv->freq.denom = 1;

  device_get_res_freq(dev, &pv->freq, 0);
  assert(pv->freq.denom == 1);
  // Queue init
  dev_rq_pqueue_init(&pv->pti.queue);
  // Interrupt init
  device_irq_source_init(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT, &efr32_radio_irq);

  if (device_irq_source_link(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT, -1)) {
    goto err_mem;
  }
  dev->start_count |= 1;
  efr32_protimer_start_counter(&pv->pti);

#ifdef CONFIG_DRIVER_EFR32_DEBUG
  efr32_rfp_cfg_prs_dbg(ctx);
  efr32_radio_debug_init(pv);
#endif
  // Note pvdata and interface into generic context
  ctx->gctx.pvdata = ctx;
  ctx->gctx.drv = &efr32_radio_itfc;
  // Init generic context
  dev_rfpacket_init(&ctx->gctx);
  // Indicate that we finished init
  efr32_rfp_fill_status(ctx, DEV_RFPACKET_STATUS_MISC);
  efr32_rfp_req_done(ctx);
  return 0;

err_mem:
  mem_free(ctx);
  return -1;
}

static DEV_CLEANUP(efr32_radio_cleanup) {
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  error_t err = dev_rfpacket_clean_check(&ctx->gctx);

  if (!err) {
    device_irq_source_unlink(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT);
    dev_rq_pqueue_destroy(&pv->pti.queue);
    dev_rfpacket_clean(&ctx->gctx);
    mem_free(ctx);
  }
  return err;
}

static error_t efr32_rfp_init(struct radio_efr32_rfp_ctx_s *ctx) {
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_RXCTRL_ADDR, EFR32_FRC_RXCTRL_BUFRESTOREFRAMEERROR |
                                                           EFR32_FRC_RXCTRL_BUFRESTORERXABORTED);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_TRAILRXDATA_ADDR, EFR32_FRC_TRAILRXDATA_RSSI);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(0), EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(1), EFR32_FRC_FCD_WORDS(255) |
                                                           EFR32_FRC_FCD_INCLUDECRC |
                                                           EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(2), EFR32_FRC_FCD_BUFFER(1) |
                                                           EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(3), EFR32_FRC_FCD_WORDS(255) |
                                                           EFR32_FRC_FCD_BUFFER(1) |
                                                           EFR32_FRC_FCD_INCLUDECRC |
                                                           EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CTRL_ADDR, EFR32_SYNTH_CTRL_DITHERDSMINPUT |
                                                           EFR32_SYNTH_CTRL_DITHERDSMOUTPUT(7) |
                                                           EFR32_SYNTH_CTRL_DITHERDAC(3) |
                                                           EFR32_SYNTH_CTRL_LOCKTHRESHOLD(3) |
                                                           EFR32_SYNTH_CTRL_AUXLOCKTHRESHOLD(5) |
                                                           EFR32_SYNTH_CTRL_PRSMUX0(DISABLED) |
                                                           EFR32_SYNTH_CTRL_PRSMUX1(DISABLED) |
                                                           EFR32_SYNTH_CTRL_DEMMODE);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CALCTRL_ADDR, EFR32_SYNTH_CALCTRL_NUMCYCLES(1) |
                                                           EFR32_SYNTH_CALCTRL_CAPCALCYCLEWAIT(CYCLES1) |
                                                           EFR32_SYNTH_CALCTRL_STARTUPTIMING(10) |
                                                           EFR32_SYNTH_CALCTRL_AUXCALCYCLES(4) |
                                                           EFR32_SYNTH_CALCTRL_AUXCALCYCLEWAIT(CYCLES64));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCDACCTRL_ADDR, EFR32_SYNTH_VCDACCTRL_VCDACVAL(35));
  // Frequency 868 MHz
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_FREQ_ADDR, EFR32_SYNTH_FREQ_FREQ(35553280));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_IFFREQ_ADDR, EFR32_SYNTH_IFFREQ_IFFREQ(16384) |
                                                               EFR32_SYNTH_IFFREQ_LOSIDE);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR, EFR32_SYNTH_DIVCTRL_LODIVFREQCTRL(LODIV3) |
                                                           EFR32_SYNTH_DIVCTRL_AUXLODIVFREQCTRL(LODIV1));
  // Channel spacing
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHSP_ADDR, EFR32_SYNTH_CHSP_CHSP(4096));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOTUNING_ADDR, EFR32_SYNTH_VCOTUNING_VCOTUNING(125));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOGAIN_ADDR, EFR32_SYNTH_VCOGAIN_VCOGAIN(35));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CAPCALCYCLECNT_ADDR, EFR32_SYNTH_CAPCALCYCLECNT_CAPCALCYCLECNT(127));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CTRL_ADDR, EFR32_RAC_CTRL_ACTIVEPOL |
                                                         EFR32_RAC_CTRL_PAENPOL |
                                                         EFR32_RAC_CTRL_LNAENPOL);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSCTRL_ADDR, EFR32_RAC_LVDSCTRL_LVDSCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSIDLESEQ_ADDR, EFR32_RAC_LVDSIDLESEQ_LVDSIDLESEQ(188));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_HFXORETIMECTRL_ADDR, EFR32_RAC_HFXORETIMECTRL_LIMITH(4) |
                                                           EFR32_RAC_HFXORETIMECTRL_LIMITL(5));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_WAITSNSH_ADDR, EFR32_RAC_WAITSNSH_WAITSNSH(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PRESC_ADDR, EFR32_RAC_PRESC_STIMER(7));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHENCTRL_ADDR, EFR32_RAC_SYNTHENCTRL_VCOEN |
                                                                EFR32_RAC_SYNTHENCTRL_LODIVEN |
                                                                EFR32_RAC_SYNTHENCTRL_CHPEN |
                                                                EFR32_RAC_SYNTHENCTRL_LPFEN |
                                                                EFR32_RAC_SYNTHENCTRL_SYNTHCLKEN |
                                                                EFR32_RAC_SYNTHENCTRL_SYNTHSTARTREQ |
                                                                EFR32_RAC_SYNTHENCTRL_CHPLDOEN |
                                                                EFR32_RAC_SYNTHENCTRL_LODIVSYNCCLKEN |
                                                                EFR32_RAC_SYNTHENCTRL_MMDLDOEN |
                                                                EFR32_RAC_SYNTHENCTRL_VCOLDOEN |
                                                                EFR32_RAC_SYNTHENCTRL_VCODIVEN);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHREGCTRL_ADDR, EFR32_RAC_SYNTHREGCTRL_MMDLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_MMDLDOVREFTRIM(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_VCOLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_VCOLDOVREFTRIM(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_CHPLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_CHPLDOVREFTRIM(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_VCOCTRL_ADDR, EFR32_RAC_VCOCTRL_VCOAMPLITUDE(10) |
                                                            EFR32_RAC_VCOCTRL_VCODETAMPLITUDE(10) |
                                                            EFR32_RAC_VCOCTRL_VCODETEN |
                                                            EFR32_RAC_VCOCTRL_VCODETMODE |
                                                            EFR32_RAC_VCOCTRL_VCOAREGCURR(1) |
                                                            EFR32_RAC_VCOCTRL_VCOCREGCURR(2) |
                                                            EFR32_RAC_VCOCTRL_VCODIVCURR(15));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL_ADDR, EFR32_RAC_MMDCTRL_MMDDIVDCDC(123) |
                                                            EFR32_RAC_MMDCTRL_MMDDIVRSDCDC(1) |
                                                            EFR32_RAC_MMDCTRL_MMDDIVRSDIG(1) |
                                                            EFR32_RAC_MMDCTRL_MMDENDCDC |
                                                            EFR32_RAC_MMDCTRL_MMDENRSDCDC |
                                                            EFR32_RAC_MMDCTRL_MMDENRSDIG);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL_ADDR, EFR32_RAC_CHPCTRL_CHPBIAS(6) |
                                                           EFR32_RAC_CHPCTRL_CHPCURR(5));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCAL_ADDR, EFR32_RAC_CHPCAL_PSRC(4) |
                                                           EFR32_RAC_CHPCAL_NSRC(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LPFCTRL_ADDR, EFR32_RAC_LPFCTRL_LPFINPUTCAP(3) |
                                                           EFR32_RAC_LPFCTRL_LPFSWITCHINGEN |
                                                           EFR32_RAC_LPFCTRL_LPFGNDSWITCHINGEN |
                                                           EFR32_RAC_LPFCTRL_LPFBWTX(2));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_AUXCTRL_ADDR, EFR32_RAC_AUXCTRL_CHPCURR(3) |
                                                           EFR32_RAC_AUXCTRL_RXAMP(16) |
                                                           EFR32_RAC_AUXCTRL_LDOAMPCURR(4) |
                                                           EFR32_RAC_AUXCTRL_LDOVREFTRIM(6) |
                                                           EFR32_RAC_AUXCTRL_LPFRES(1));

  // Sub-G PA configuration
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGRFENCTRL0_ADDR, EFR32_RAC_SGRFENCTRL0_LNAMIXBIASEN |
                                                           EFR32_RAC_SGRFENCTRL0_LNAMIXLOBIASEN |
                                                           EFR32_RAC_SGRFENCTRL0_LNAMIXRFBIASEN);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGLNAMIXCTRL_ADDR, EFR32_RAC_SGLNAMIXCTRL_CASCODEBIAS(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_LOBIAS(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_VREG(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_RFBIAS(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_SGREGAMPCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPACTRL0_ADDR, EFR32_RAC_SGPACTRL0_CASCODE(EN3SLICES) |
                                                           EFR32_RAC_SGPACTRL0_SLICE(EN3SLICES) |
                                                           EFR32_RAC_SGPACTRL0_STRIPE(8) |
                                                           EFR32_RAC_SGPACTRL0_DACGLITCHCTRL);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPAPKDCTRL_ADDR, EFR32_RAC_SGPAPKDCTRL_VTHSEL(23) |
                                                           EFR32_RAC_SGPAPKDCTRL_CAPSEL(3) |
                                                           EFR32_RAC_SGPAPKDCTRL_I2VCM(2) |
                                                           EFR32_RAC_SGPAPKDCTRL_PKDBIASTH(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL0_ADDR, EFR32_RAC_SGPABIASCTRL0_LDOBIAS |
                                                           EFR32_RAC_SGPABIASCTRL0_PABIAS(1) |
                                                           EFR32_RAC_SGPABIASCTRL0_BUF0BIAS(1) |
                                                           EFR32_RAC_SGPABIASCTRL0_BUF12BIAS(1) |
                                                           EFR32_RAC_SGPABIASCTRL0_SGDACFILTBANDWIDTH(7));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL1_ADDR, EFR32_RAC_SGPABIASCTRL1_VLDO(3) |
                                                           EFR32_RAC_SGPABIASCTRL1_VLDOFB(2) |
                                                           EFR32_RAC_SGPABIASCTRL1_VCASCODEHV(5) |
                                                           EFR32_RAC_SGPABIASCTRL1_VCASCODELV(4) |
                                                           EFR32_RAC_SGPABIASCTRL1_SGVBATDETTHRESHOLD(2));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCTRL_ADDR, EFR32_RAC_RFBIASCTRL_LDOVREF(4) |
                                                           EFR32_RAC_RFBIASCTRL_LDOAMPCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCAL_ADDR, EFR32_RAC_RFBIASCAL_VREF(22) |
                                                           EFR32_RAC_RFBIASCAL_BIAS(28) |
                                                           EFR32_RAC_RFBIASCAL_TEMPCO(48));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LNAMIXCTRL1_ADDR, EFR32_RAC_LNAMIXCTRL1_TRIMAUXPLLCLK(E0) |
                                                           EFR32_RAC_LNAMIXCTRL1_TRIMVCASLDO |
                                                           EFR32_RAC_LNAMIXCTRL1_TRIMVREGMIN(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACTRL_ADDR, EFR32_RAC_IFPGACTRL_VLDO(3) |
                                                           EFR32_RAC_IFPGACTRL_BANDSEL(SG) |
                                                           EFR32_RAC_IFPGACTRL_CASCBIAS(7) |
                                                           EFR32_RAC_IFPGACTRL_TRIMVCASLDO |
                                                           EFR32_RAC_IFPGACTRL_TRIMVCM(3) |
                                                           EFR32_RAC_IFPGACTRL_TRIMVREGMIN(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACAL_ADDR, EFR32_RAC_IFPGACAL_IRAMP(2) |
                                                           EFR32_RAC_IFPGACAL_IRPHASE(10) |
                                                           EFR32_RAC_IFPGACAL_OFFSETI(64));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFFILTCTRL_ADDR, EFR32_RAC_IFFILTCTRL_CENTFREQ(7) |
                                                           EFR32_RAC_IFFILTCTRL_VCM(2) |
                                                           EFR32_RAC_IFFILTCTRL_VREG(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFADCCTRL_ADDR, EFR32_RAC_IFADCCTRL_REALMODE |
                                                           EFR32_RAC_IFADCCTRL_VLDOSERIES(3) |
                                                           EFR32_RAC_IFADCCTRL_VLDOSERIESCURR(3) |
                                                           EFR32_RAC_IFADCCTRL_VLDOSHUNT(2) |
                                                           EFR32_RAC_IFADCCTRL_VLDOCLKGEN(3) |
                                                           EFR32_RAC_IFADCCTRL_VCM(E2) |
                                                           EFR32_RAC_IFADCCTRL_OTA1CURRENT(2) |
                                                           EFR32_RAC_IFADCCTRL_OTA2CURRENT(2) |
                                                           EFR32_RAC_IFADCCTRL_OTA3CURRENT(2) |
                                                           EFR32_RAC_IFADCCTRL_REGENCLKDELAY(3) |
                                                           EFR32_RAC_IFADCCTRL_ENABLECLK |
                                                           EFR32_RAC_IFADCCTRL_INVERTCLK);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RCTUNE_ADDR, EFR32_RAC_RCTUNE_IFADCRCTUNE(34) |
                                                           EFR32_RAC_RCTUNE_IFFILT(34));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_APC_ADDR, EFR32_RAC_APC_AMPCONTROLLIMITSW(255));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL1_ADDR, EFR32_RAC_CHPCTRL1_BYPREPLDORX |
                                                           EFR32_RAC_CHPCTRL1_TRIMREPLDO(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL1_ADDR, EFR32_RAC_MMDCTRL1_BYPREPLDORX |
                                                           EFR32_RAC_MMDCTRL1_TRIMREPLDO(1));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_FREQOFFEST_ADDR, EFR32_MODEM_FREQOFFEST_FREQOFFEST(9) |
                                                           EFR32_MODEM_FREQOFFEST_CORRVAL(115) |
                                                           EFR32_MODEM_FREQOFFEST_SOFTVAL(221));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MIXCTRL_ADDR, EFR32_MODEM_MIXCTRL_MODE(NORMAL) |
                                                           EFR32_MODEM_MIXCTRL_DIGIQSWAPEN);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL0_ADDR, EFR32_MODEM_CTRL0_MAPFSK(MAP0) |
                                                           EFR32_MODEM_CTRL0_CODING(NRZ) |
                                                           EFR32_MODEM_CTRL0_MODFORMAT(FSK2) |
                                                           EFR32_MODEM_CTRL0_DSSSSHIFTS(NOSHIFT) |
                                                           EFR32_MODEM_CTRL0_DSSSDOUBLE(DIS) |
                                                           EFR32_MODEM_CTRL0_DIFFENCMODE(DIS) |
                                                           EFR32_MODEM_CTRL0_SHAPING(EVENLENGTH) |
                                                           EFR32_MODEM_CTRL0_DEMODRAWDATASEL(DIS) |
                                                           EFR32_MODEM_CTRL0_FRAMEDETDEL(DEL0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL2_ADDR, EFR32_MODEM_CTRL2_SQITHRESH(200) |
                                                           EFR32_MODEM_CTRL2_TXPINMODE(OFF) |
                                                           EFR32_MODEM_CTRL2_DATAFILTER(LEN7) |
                                                           EFR32_MODEM_CTRL2_RATESELMODE(NOCHANGE) |
                                                           EFR32_MODEM_CTRL2_DMASEL(SOFT));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL3_ADDR, EFR32_MODEM_CTRL3_PRSDINSEL(PRSCH0) |
                                                           EFR32_MODEM_CTRL3_ANTDIVMODE(ANTENNA0) |
                                                           EFR32_MODEM_CTRL3_TSAMPMODE(ON) |
                                                           EFR32_MODEM_CTRL3_TSAMPDEL(3) |
                                                           EFR32_MODEM_CTRL3_TSAMPLIM(8));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL4_ADDR, EFR32_MODEM_CTRL4_ADCSATLEVEL(CONS64));
  // Baudrate 38400 bit/s
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TXBR_ADDR, EFR32_MODEM_TXBR_TXBRNUM(31875) |
                                                             EFR32_MODEM_TXBR_TXBRDEN(255));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RXBR_ADDR, EFR32_MODEM_RXBR_RXBRNUM(19) |
                                                           EFR32_MODEM_RXBR_RXBRDEN(27) |
                                                           EFR32_MODEM_RXBR_RXBRINT(3));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CF_ADDR, EFR32_MODEM_CF_DEC0(DF3) |
                                                           EFR32_MODEM_CF_DEC1(44) |
                                                           EFR32_MODEM_CF_CFOSR(CF7) |
                                                           EFR32_MODEM_CF_DEC1GAIN(ADD0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TIMING_ADDR, EFR32_MODEM_TIMING_TIMTHRESH(27) |
                                                           EFR32_MODEM_TIMING_TIMINGBASES(7) |
                                                           EFR32_MODEM_TIMING_OFFSUBNUM(13) |
                                                           EFR32_MODEM_TIMING_OFFSUBDEN(8) |
                                                           EFR32_MODEM_TIMING_FASTRESYNC(DIS));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MODINDEX_ADDR, EFR32_MODEM_MODINDEX_MODINDEXM(20) |
                                                           EFR32_MODEM_MODINDEX_MODINDEXE(27) |
                                                           EFR32_MODEM_MODINDEX_FREQGAINE(3) |
                                                           EFR32_MODEM_MODINDEX_FREQGAINM(7));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING0_ADDR, EFR32_MODEM_SHAPING0_COEFF0(4) |
                                                           EFR32_MODEM_SHAPING0_COEFF1(10) |
                                                           EFR32_MODEM_SHAPING0_COEFF2(20) |
                                                           EFR32_MODEM_SHAPING0_COEFF3(34));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING1_ADDR, EFR32_MODEM_SHAPING1_COEFF4(50) |
                                                           EFR32_MODEM_SHAPING1_COEFF5(65) |
                                                           EFR32_MODEM_SHAPING1_COEFF6(74) |
                                                           EFR32_MODEM_SHAPING1_COEFF7(79));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPCTRL_ADDR, EFR32_MODEM_RAMPCTRL_RAMPRATE2(6));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPLEV_ADDR, EFR32_MODEM_RAMPLEV_RAMPLEV2(150));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCCOMP_ADDR, EFR32_MODEM_DCCOMP_DCESTIEN |
                                                           EFR32_MODEM_DCCOMP_DCCOMPEN |
                                                           EFR32_MODEM_DCCOMP_DCCOMPGEAR(3) |
                                                           EFR32_MODEM_DCCOMP_DCLIMIT(FULLSCALE));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCESTI_ADDR, EFR32_MODEM_DCESTI_DCCOMPESTIVALI(51) |
                                                               EFR32_MODEM_DCESTI_DCCOMPESTIVALQ(32679));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SRCCHF_ADDR, EFR32_MODEM_SRCCHF_SRCRATIO1(128) |
                                                               EFR32_MODEM_SRCCHF_SRCRATIO2(1024));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD0_ADDR, EFR32_MODEM_DSATHD0_SPIKETHD(100) |
                                                           EFR32_MODEM_DSATHD0_UNMODTHD(4) |
                                                           EFR32_MODEM_DSATHD0_FDEVMINTHD(12) |
                                                           EFR32_MODEM_DSATHD0_FDEVMAXTHD(120));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD1_ADDR, EFR32_MODEM_DSATHD1_POWABSTHD(5000) |
                                                           EFR32_MODEM_DSATHD1_POWRELTHD(DISABLED) |
                                                           EFR32_MODEM_DSATHD1_DSARSTCNT(2) |
                                                           EFR32_MODEM_DSATHD1_RSSIJMPTHD(6) |
                                                           EFR32_MODEM_DSATHD1_FREQLATDLY(1) |
                                                           EFR32_MODEM_DSATHD1_PWRFLTBYP |
                                                           EFR32_MODEM_DSATHD1_AMPFLTBYP |
                                                           EFR32_MODEM_DSATHD1_PWRDETDIS);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSACTRL_ADDR, EFR32_MODEM_DSACTRL_DSAMODE(DISABLED) |
                                                           EFR32_MODEM_DSACTRL_ARRTHD(7) |
                                                           EFR32_MODEM_DSACTRL_ARRTOLERTHD0(2) |
                                                           EFR32_MODEM_DSACTRL_ARRTOLERTHD1(4) |
                                                           EFR32_MODEM_DSACTRL_FREQAVGSYM |
                                                           EFR32_MODEM_DSACTRL_DSARSTON);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VITERBIDEMOD_ADDR, EFR32_MODEM_VITERBIDEMOD_VITERBIKSI1(64) |
                                                           EFR32_MODEM_VITERBIDEMOD_VITERBIKSI2(48) |
                                                           EFR32_MODEM_VITERBIDEMOD_VITERBIKSI3(32));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG0_ADDR, EFR32_MODEM_VTCORRCFG0_EXPECTPATT(349879) |
                                                           EFR32_MODEM_VTCORRCFG0_EXPSYNCLEN(8) |
                                                           EFR32_MODEM_VTCORRCFG0_BUFFHEAD(2));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DIGMIXCTRL_ADDR, EFR32_MODEM_DIGMIXCTRL_DIGMIXFREQ(32768) |
                                                           EFR32_MODEM_DIGMIXCTRL_DIGMIXMODE);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG1_ADDR, EFR32_MODEM_VTCORRCFG1_CORRSHFTLEN(32) |
                                                           EFR32_MODEM_VTCORRCFG1_VTFRQLIM(192));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTTRACK_ADDR, EFR32_MODEM_VTTRACK_FREQTRACKMODE(DISABLED) |
                                                           EFR32_MODEM_VTTRACK_TIMTRACKTHD(2) |
                                                           EFR32_MODEM_VTTRACK_TIMEACQUTHD(238) |
                                                           EFR32_MODEM_VTTRACK_TIMEOUTMODE |
                                                           EFR32_MODEM_VTTRACK_TIMGEAR(GEAR0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_BREST_ADDR, EFR32_MODEM_BREST_BRESTINT(7) |
                                                           EFR32_MODEM_BREST_BRESTNUM(11));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_POE_ADDR, EFR32_MODEM_POE_POEI(511));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_RSSI_ADDR, EFR32_AGC_RSSI_RSSIFRAC(3) |
                                                           EFR32_AGC_RSSI_RSSIINT(149));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_FRAMERSSI_ADDR, EFR32_AGC_FRAMERSSI_FRAMERSSIINT(128));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, EFR32_AGC_CTRL1_RSSIPERIOD(3));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL2_ADDR, EFR32_AGC_CTRL2_HYST(3) |
                                                           EFR32_AGC_CTRL2_FASTLOOPDEL(10) |
                                                           EFR32_AGC_CTRL2_CFLOOPDEL(26) |
                                                           EFR32_AGC_CTRL2_ADCRSTFASTLOOP(GAINREDUCTION) |
                                                           EFR32_AGC_CTRL2_ADCRSTSTARTUP);

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_IFPEAKDET_ADDR, EFR32_AGC_IFPEAKDET_PKDTHRESH1(2) |
                                                           EFR32_AGC_IFPEAKDET_PKDTHRESH2(8));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_RFPEAKDET_ADDR, EFR32_AGC_RFPEAKDET_RFPKDTHRESH1(5) |
                                                           EFR32_AGC_RFPEAKDET_RFPKDTHRESH2(13));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAINRANGE_ADDR, EFR32_AGC_GAINRANGE_MAXGAIN(62) |
                                                           EFR32_AGC_GAINRANGE_MINGAIN(112));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAININDEX_ADDR, EFR32_AGC_GAININDEX_NUMINDEXPGA(12) |
                                                           EFR32_AGC_GAININDEX_NUMINDEXDEGEN(3) |
                                                           EFR32_AGC_GAININDEX_NUMINDEXSLICES(6) |
                                                           EFR32_AGC_GAININDEX_NUMINDEXATTEN(18));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_SLICECODE_ADDR, EFR32_AGC_SLICECODE_SLICECODEINDEX0(3) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX1(4) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX2(5) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX3(6) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX4(8) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX5(10) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX6(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE1_ADDR, EFR32_AGC_ATTENCODE1_ATTENCODEINDEX1(1) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX2(2) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX3(3) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX4(4) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX5(5) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX6(6));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE2_ADDR, EFR32_AGC_ATTENCODE2_ATTENCODEINDEX7(7) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX8(8) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX9(9) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX10(10) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX11(11) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX12(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE3_ADDR, EFR32_AGC_ATTENCODE3_ATTENCODEINDEX13(13) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX14(14) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX15(15) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX16(16) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX17(17) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX18(18));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAINSTEPLIM_ADDR, EFR32_AGC_GAINSTEPLIM_FASTSTEPDOWN(5) |
                                                           EFR32_AGC_GAINSTEPLIM_FASTSTEPUP(2) |
                                                           EFR32_AGC_GAINSTEPLIM_CFLOOPSTEPMAX(8) |
                                                           EFR32_AGC_GAINSTEPLIM_SLOWDECAYCNT(1) |
                                                           EFR32_AGC_GAINSTEPLIM_ADCATTENMODE(DISABLE));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_LOOPDEL_ADDR, EFR32_AGC_LOOPDEL_PKDWAIT(25) |
                                                           EFR32_AGC_LOOPDEL_IFPGADEL(12) |
                                                           EFR32_AGC_LOOPDEL_LNASLICESDEL(16));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_MININDEX_ADDR, EFR32_AGC_MININDEX_INDEXMINSLICES(18) |
                                                           EFR32_AGC_MININDEX_INDEXMINDEGEN(24) |
                                                           EFR32_AGC_MININDEX_INDEXMINPGA(27));
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  // Sequencer code initialization
  efr32_radio_seq_init(pv, seqcode, 4 * seqcode_size);
  // Turn off radio
  efr32_rfp_disable(ctx);
  // Timer init
  efr32_protimer_init(&pv->pti);
  uint32_t x = bit_ctz32(EFR32_RADIO_RFP_BUFFER_SIZE) - 6;
  // TX/RX buffers initialization
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(0), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(0), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(1), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(1), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(2), (uint32_t)ctx->pv.rx_length_buffer);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(2), 0);
  // Configure variable length mode
  x = EFR32_FRC_CTRL_RXFCDMODE(FCDMODE2) |
      EFR32_FRC_CTRL_TXFCDMODE(FCDMODE2) |
      EFR32_FRC_CTRL_BITORDER(MSB) |
      EFR32_FRC_CTRL_BITSPERWORD(7);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);

  x = EFR32_FRC_DFLCTRL_DFLMODE(SINGLEBYTE) |
      EFR32_FRC_DFLCTRL_MINLENGTH(1) |
      EFR32_FRC_DFLCTRL_DFLBITS(8);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, x);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MAXLENGTH_ADDR, EFR32_RADIO_RFP_BUFFER_SIZE - 1);
  // Clear buffer
  for (uint8_t i = 0; i < 4; i++) {
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(i), EFR32_BUFC_CMD_CLEAR);
  }
  // Enable irq
  x = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);
  x |= EFR32_TX_IRQ_FRC_MSK | EFR32_RX_IRQ_FRC_MSK;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, x);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR, x);
  return 0;
}

