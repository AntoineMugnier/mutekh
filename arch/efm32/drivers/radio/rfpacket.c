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
#include "radio.h"

/* LBT parameters */
#define EFR32_ETSI_LBT_TIME              5000ULL     /*us*/
#define EFR32_MAX_LBT_RETRY_COUNT        4

/* Protimer Radio class parameters */
#define EFR32_PROTIMER_RX_START_CHANNEL  1
#define EFR32_PROTIMER_RX_STOP_CHANNEL   2
#define EFR32_PROTIMER_TX_CHANNEL        3

#define EFR32_RX_IRQ_FRC_MSK      (EFR32_FRC_IF_RXDONE     |      \
                                   EFR32_FRC_IF_RXOF       |      \
                                   EFR32_FRC_IF_BLOCKERROR |      \
                                   EFR32_FRC_IF_FRAMEERROR)

// LDC values
#define EFR32_LDC_TX_RESTART_NS 5000000 // Time before restarting rx after tx
#define EFR32_LDC_RX_START_NS 760000 // Measured empirically, 80µs rx warm, 880µs rx off (25µs/sink gate, 452µs hfxo)
#define EFR32_LDC_RX_THRESH 0x24 // Number of symbols to detect preambule (Abritrary value)

static void efr32_rfp_timer_irq(struct device_s *dev);
static error_t efr32_radio_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value);
static inline void  efr32_rfp_set_status_timeout(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status);
static void efr32_rfp_req_timeout(struct radio_efr32_rfp_ctx_s *ctx);
static inline void  efr32_rfp_set_status_done(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status);
static void efr32_rfp_req_done(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_req_done_direct(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status);
static void efr32_rfp_start_tx_lbt(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static bool_t efr32_rfp_disable(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_start_rx_scheduled(struct radio_efr32_rfp_ctx_s *ctx);
static int16_t efr32_rfp_get_rssi(struct radio_efr32_rfp_ctx_s *ctx, uintptr_t a);
static void efr32_rfp_read_packet(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_rx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq);
static void efr32_rfp_tx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq);
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


// static void efr32_debug_cpu_mem_write_32(uint32_t addr, uint32_t data) {
//   printk("0x%02x, 0x%02x,\n", addr, data);
//   cpu_mem_write_32(addr, data);
// }

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
  memcpy(&cfg->freq, &pv->freq, sizeof(struct dev_freq_s));

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

    if (irq & EFR32_PROTIMER_IF_LBTFAILURE) {
      // Tx LBT timeout
      efr32_radio_printk("tx timeout\n");
      efr32_rfp_set_status_timeout(ctx, DEV_RFPACKET_STATUS_TX_TIMEOUT);
      // Check if packet incoming
      if (!dev_rfpacket_is_packet_incoming(&ctx->gctx)) {
        // No packet incoming, try to disable radio
        if (efr32_rfp_disable(ctx)) {
          // Disable already done, call req timeout
          efr32_rfp_req_timeout(ctx);
        } else {
          // Disable is pending irq
          ctx->rac_irq_type = EFR32_RAC_IRQ_TIMEOUT;
        }
      } else {
        // Call req timeout
        efr32_rfp_req_timeout(ctx);
      }
    } else if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL)) {
      // Rx timeout
      efr32_protimer_disable_compare(pti, EFR32_PROTIMER_RX_STOP_CHANNEL);
      efr32_radio_printk("rx timeout\n");
      efr32_rfp_set_status_timeout(ctx, DEV_RFPACKET_STATUS_RX_TIMEOUT);
      // Check if packet incoming
      if (!dev_rfpacket_is_packet_incoming(&ctx->gctx)) {
        // No packet incoming, try to disable radio
        if (efr32_rfp_disable(ctx)) {
          // Disable already done, call req timeout
          efr32_rfp_req_timeout(ctx);
        } else {
          // Disable is pending irq
          ctx->rac_irq_type = EFR32_RAC_IRQ_TIMEOUT;
        }
      } else {
        // Call req timeout
        efr32_rfp_req_timeout(ctx);
      }
    }
    if (!(irq & (EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL) | EFR32_PROTIMER_IF_WRAPCNTOF))) {
      // Timer class interrupts
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

static error_t efr32_radio_timer_req(struct radio_efr32_rfp_ctx_s *ctx, struct dev_timer_rq_s *rq) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct efr32_protimer_s *pti = &pv->pti;

  dev_timer_value_t value = efr32_protimer_get_value(pti);

  if (rq->delay) {
    rq->deadline = value + rq->delay;
  }
  if (rq->deadline <= value) {
    return -ETIMEDOUT;
  } else {
    dev_timer_rq_insert(&pti->queue, rq);
    rq->base.drvdata = ctx;
    // Start request, raise irq on race condition
    if (dev_timer_rq_prev(&pti->queue, rq) == NULL) {
      efr32_protimer_request_start(pti, value, rq->deadline, EFR32_PROTIMER_CHANNEL);
    }
    return 0;
  }
}

static error_t efr32_radio_timer_cancel(struct radio_efr32_rfp_ctx_s *ctx, struct dev_timer_rq_s *rq) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  error_t err = -ETIMEDOUT;

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
  return err;
}






/**************************** RFPACKET PART ********************************/
#ifdef CONFIG_DRIVER_EFR32_DEBUG
static void efr32_rfp_cfg_rac_dbg(struct radio_efr32_rfp_ctx_s *ctx) {
  uint32_t x;
  /* Set PC9 to PC11 in output */
  uint32_t a = 0x4000a008 + 2 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x444 << 4));
  /* Configure PRS channel 9/10/11 on PC9/10/11 */
  x = EFR32_PRS_ROUTEPEN_CHPEN(9)  |
      EFR32_PRS_ROUTEPEN_CHPEN(10) |
      EFR32_PRS_ROUTEPEN_CHPEN(11);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTEPEN_ADDR, x);

  x = EFR32_PRS_ROUTELOC2_CH9LOC(14) |
      EFR32_PRS_ROUTELOC2_CH10LOC(4) |
      EFR32_PRS_ROUTELOC2_CH11LOC(4);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTELOC2_ADDR, x);
  /* PC9 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(RAC);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 1); //RAC TX

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(9), x);
  /* PC10 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(RAC);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 2); //RAC RX

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(10), x);
  /* PC11 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(MODEML);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 0); //MODEM FRAME DET

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(11), x);
}

static void efr32_rfp_cfg_protimer_dbg(struct radio_efr32_rfp_ctx_s *ctx) {
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




#ifdef CONFIG_DRIVER_EFR32_RFPACKET_LDC
static uint32_t efr32_rfp_get_ldc_time(void) {
  return cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR);
}

static void efr32_rfp_calc_ldc(struct radio_efr32_rfp_ctx_s *ctx) {
  // Time bit in ns
  uint32_t time_bit = 1000000000 / ctx->curr_drate;
  uint32_t time_preamb = (ctx->curr_rx_pb_len) * time_bit;
  uint32_t time_rx = EFR32_LDC_RX_THRESH * time_bit + EFR32_LDC_RX_START_NS;
  assert(time_preamb > time_rx);
  uint32_t time_sleep = time_preamb - time_rx;
  // Convert in 30us step with rounding
  ctx->ldc_rx_start = (time_sleep + 15000) / 30000;
  ctx->ldc_rx_end = (time_preamb + 15000) / 30000;
  //printk("values: rx start %d rx end %d\n", ctx->ldc_rx_start, ctx->ldc_rx_end);
}

static void efr32_rfp_schedule_next_wup(struct radio_efr32_rfp_ctx_s *ctx, uint32_t rx_start, uint32_t rx_end) {
  // Get current time
  uint32_t curr_time = efr32_rfp_get_ldc_time();
  // Set reg values
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CCV_ADDR(1), curr_time + ctx->ldc_rx_start);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CCV_ADDR(2), curr_time + ctx->ldc_rx_end);
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_SLEEP
  // Stop all clock except for RTCC
  for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT - 1; i++) {
    dev_clock_sink_gate(&ctx->pv.clk_ep[i], DEV_CLOCK_EP_POWER);
  }
  ctx->sleep = true;
#endif
}

static void efr32_rfp_set_wup(struct radio_efr32_rfp_ctx_s *ctx) {
  uint32_t rx_start, rx_end;
  // Get current time
  uint32_t curr_time = efr32_rfp_get_ldc_time();
  // Calc start and end time
  rx_start = curr_time + EFR32_LDC_TX_RESTART_NS;
  rx_end = curr_time + EFR32_LDC_TX_RESTART_NS + ctx->ldc_rx_end - ctx->ldc_rx_start;
  // Schedule new wup
  efr32_rfp_schedule_next_wup(ctx, rx_start, rx_end);
}

static void efr32_rfp_start_rx_ldc(struct radio_efr32_rfp_ctx_s *ctx) {
  // Clear flag
  ctx->ldc_canceled = false;
  // Set rtcc config
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CTRL_ADDR(1), EFM32_RTCC_CC_CTRL_MODE_SHIFT_VAL(OUTPUTCOMPARE));
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CTRL_ADDR(2), EFM32_RTCC_CC_CTRL_MODE_SHIFT_VAL(OUTPUTCOMPARE));
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, endian_le32(EFM32_RTCC_IFC_MASK));
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR, EFM32_RTCC_IEN_CC(2) | EFM32_RTCC_IEN_CC(1));
  // Set AFC
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_AFC_ADDR, 0x019591F9);
  // Config ldc timer irq
  efr32_rfp_set_wup(ctx);
  // Start Counter
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, endian_le32(EFM32_RTCC_CTRL_ENABLE));
}

static KROUTINE_EXEC(efr32_rfp_ldc) {
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  if (!ctx->ldc_canceled) {
    // Get current time
    uint32_t curr_time = efr32_rfp_get_ldc_time();
    // Set values
    uint32_t rx_start = curr_time + ctx->ldc_rx_start;
    uint32_t rx_end = curr_time + ctx->ldc_rx_end;
    // Schedule new wup
    efr32_rfp_schedule_next_wup(ctx, rx_start, rx_end);
  }
}

static void efr32_rfp_ldc_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq) {
  // Start Rx event
  if (irq & EFM32_RTCC_IEN_CC(1)) {
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_SLEEP
    // Wakeup clocks if sleeping
    if (ctx->sleep) {
      for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT - 2; i++) {
        dev_clock_sink_gate(&ctx->pv.clk_ep[i], DEV_CLOCK_EP_POWER_CLOCK);
      }
      ctx->sleep = false;
    }
#endif
    // Enable rx
    uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
    x |= 0x2;
    cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
  // End rx event
  } else if (irq & EFM32_RTCC_IEN_CC(2)) {
    // Get rac state and modem status
    uint32_t rac_state = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
    rac_state = EFR32_RAC_STATUS_STATE_GET(rac_state);
    uint32_t modem_status = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IF_ADDR);
    // Ignore if tx
    if ((rac_state == EFR32_RAC_STATUS_STATE_TX) || (rac_state == EFR32_RAC_STATUS_STATE_TXWARM))  {
    // Don't stop rx if rx incoming or preambule detected
    } else if ((rac_state != EFR32_RAC_STATUS_STATE_RXFRAME) && ((modem_status & EFR32_MODEM_IF_RXPREDET) == 0))  {
      // Disable Rx
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, 0);
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_RXDIS);
      // Call end ldc kroutine
      kroutine_init_deferred(&ctx->pv.kr, &efr32_rfp_ldc);
      kroutine_exec(&ctx->pv.kr);
    } else {
      // Clear predet modem irq
      cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_IFC_ADDR, EFR32_MODEM_IF_RXPREDET);
      // Set-up another event in case rx fail
      uint32_t curr_time = efr32_rfp_get_ldc_time();
      cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CCV_ADDR(2), curr_time + ctx->ldc_rx_end);
    }
  }
}
#endif

static inline void  efr32_rfp_set_status_timeout(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status) {
  ctx->timeout_status = status;
}

static KROUTINE_EXEC(efr32_radio_rq_timeout) {
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_kr(kr);

  efr32_radio_printk("req timeout\n");
  ctx->gctx.status = ctx->timeout_status;
  dev_rfpacket_rxtx_timeout(&ctx->gctx);
}

static void efr32_rfp_req_timeout(struct radio_efr32_rfp_ctx_s *ctx) {
  // Be sure to use efr32_rfp_set_status_timeout before calling this function
  kroutine_init_deferred(&ctx->kr, &efr32_radio_rq_timeout);
  kroutine_exec(&ctx->kr);
}

static inline void  efr32_rfp_set_status_done(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status) {
  ctx->done_status = status;
}

static KROUTINE_EXEC(efr32_radio_rq_done) {
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_radio_printk("req done\n");
  ctx->gctx.status = ctx->done_status;
  dev_rfpacket_req_done(&ctx->gctx);
}

static void efr32_rfp_req_done(struct radio_efr32_rfp_ctx_s *ctx) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  // Be sure to use efr32_rfp_set_status_done before calling this function
  kroutine_init_deferred(&pv->kr, &efr32_radio_rq_done);
  kroutine_exec(&pv->kr);
}

static void efr32_rfp_req_done_direct(struct radio_efr32_rfp_ctx_s *ctx, enum dev_rfpacket_status_s status) {
  efr32_radio_printk("req directly done\n");
  ctx->gctx.status = status;
  dev_rfpacket_req_done(&ctx->gctx);
}

static void efr32_rfp_start_tx_lbt(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;

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

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) || \
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG13) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
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

  // Calc nbsym
  uint32_t drate = ctx->curr_drate;
  uint32_t nbsym = (EFR32_ETSI_LBT_TIME * drate)/1000000;
  nbsym = bit_ctz(pow2_up(nbsym));
  assert(nbsym < 15);

  // Configure rssi period
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_RSSIPERIOD_SET(x, nbsym);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);

  // Configure AGC
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR);
  EFR32_AGC_CTRL0_MODE_SET(x, CONT);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR, x);

  // FIXME Timestamp between rx and tx
  ctx->gctx.lbt_timestamp = efr32_protimer_get_value(&pv->pti);

  efr32_rfp_set_cca_threshold(ctx, rq);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, EFR32_PROTIMER_CMD_LBTSTART);
}

static bool_t efr32_rfp_disable(struct radio_efr32_rfp_ctx_s *ctx) {
  // Disable RX
  uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
  x &= ~0xF;
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, 0);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CMD_ADDR, EFR32_FRC_CMD_RXABORT);
  // Disable Tx
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXDIS);
  // Turn RAX off
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_FORCESTATE_ADDR, EFR32_RAC_STATUS_STATE_OFF);
  // Clear irq
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR, 0);
  // Check RAC state
  x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  if (EFR32_RAC_STATUS_STATE_GET(x) != EFR32_RAC_STATUS_STATE_OFF) {
    // Set RAC interrupt
    cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFC_ADDR, EFR32_RAC_IF_STATECHANGE);
    cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IEN_ADDR, EFR32_RAC_IF_STATECHANGE);
    //cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFS_ADDR, EFR32_RAC_IF_STATECHANGE);
    return false;
  }
  return true;
}

static void efr32_rfp_start_rx_scheduled(struct radio_efr32_rfp_ctx_s *ctx) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;

#if EFR32_PROTIMER_HW_WIDTH < 64
  assert(0);
#endif
  dev_timer_value_t start = ctx->gctx.deadline;
  dev_timer_value_t end = ctx->gctx.timeout;
  dev_timer_value_t t = efr32_protimer_get_value(&pv->pti);

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

static int16_t efr32_rfp_get_rssi(struct radio_efr32_rfp_ctx_s *ctx, uintptr_t a) {
  int16_t x = cpu_mem_read_32(EFR32_AGC_ADDR + a);
  x >>= EFR32_AGC_RSSI_RSSIFRAC_IDX;

  if ((x >> 9) & 1) {
    // Negative value
    uint16_t msk = EFR32_AGC_RSSI_MASK >> EFR32_AGC_RSSI_RSSIFRAC_IDX;
    x |= 0xFFFF ^ msk;
    x -= 1;
    x = -(~x & msk);
  }
  return  (x << 3) >> 2;
}





static void efr32_rfp_read_packet(struct radio_efr32_rfp_ctx_s *ctx) {
  struct dev_rfpacket_rx_s *rx = ctx->gctx.rxrq;
  uint8_t *p = (uint8_t *)rx->buf;
  // Read freq offset
  int16_t offset = (int16_t)cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_AFCADJRX_ADDR);
  rx->frequency = ctx->curr_freq + (ctx->synth_ratio * offset) / 128;
  rx->snr = 0;
  rx->carrier = 0;
  // Read packet
  for (uint16_t i = 0; i < rx->size; i++) {
    p[i] = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));
  }
  rx->rssi = efr32_rfp_get_rssi(ctx, EFR32_AGC_FRAMERSSI_ADDR);
  // Call rfpacket to process request completion
  efr32_radio_printk("read packet\n");
  if (efr32_rfp_disable(ctx)) {
    efr32_rfp_req_done_direct(ctx, DEV_RFPACKET_STATUS_RX_DONE);
  } else {
    efr32_rfp_set_status_done(ctx, DEV_RFPACKET_STATUS_RX_DONE);
    ctx->rac_irq_type = EFR32_RAC_IRQ_TXRX;
  }
}

static KROUTINE_EXEC(efr32_rfp_rx) {
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_rfp_read_packet(ctx);
}

static void efr32_rfp_rx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  if (irq != EFR32_FRC_IF_RXDONE) {
    // Flush RX fifo
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
    efr32_radio_printk("crc err\n");
    efr32_rfp_set_status_done(ctx, DEV_RFPACKET_STATUS_CRC_ERR);
    if (efr32_rfp_disable(ctx)) {
      efr32_rfp_req_done(ctx);
    } else {
      ctx->rac_irq_type = EFR32_RAC_IRQ_TXRX;
    }
    return;
  }
  uint16_t size = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);

  uint32_t status = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_STATUS_ADDR(1));
  status = EFR32_BUFC_STATUS_BYTES_GET(status);

  assert(status >= size);
  // Asking for buffer allocation
  ctx->gctx.size = size;
  uintptr_t p = dev_rfpacket_alloc(&ctx->gctx);

  if (p == 0) {
    // Failed to allocate, flush RX fifo
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);
    efr32_radio_printk("alloc err\n");
    efr32_rfp_set_status_done(ctx, DEV_RFPACKET_STATUS_OTHER_ERR);
    if (efr32_rfp_disable(ctx)) {
      efr32_rfp_req_done(ctx);
    } else {
      ctx->rac_irq_type = EFR32_RAC_IRQ_TXRX;
    }
    return;
  }
  // Warn of incoming packet
  dev_rfpacket_packet_incoming(&ctx->gctx);
  // Ask to read outside of irq
  kroutine_init_deferred(&pv->kr, &efr32_rfp_rx);
  kroutine_exec(&pv->kr);
}

static void efr32_rfp_tx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq) {
  // If tx is over
  if (irq & EFR32_FRC_IF_TXDONE) {
    // Set status
    efr32_rfp_set_status_done(ctx, DEV_RFPACKET_STATUS_TX_DONE);
  } else {
    // Clear buffer
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR);
    efr32_rfp_set_status_done(ctx, DEV_RFPACKET_STATUS_OTHER_ERR);
  }
  efr32_radio_printk("tx packet\n");
  if (efr32_rfp_disable(ctx)) {
    // Call rfpacket to process request completion
    efr32_rfp_req_done(ctx);
  } else {
    // Wait for rac irq
    ctx->rac_irq_type = EFR32_RAC_IRQ_TXRX;
  }
}

static void efr32_rfp_rac_irq(struct radio_efr32_rfp_ctx_s *ctx) {
  switch(ctx->rac_irq_type) {
    case EFR32_RAC_IRQ_TIMEOUT:
      efr32_rfp_req_timeout(ctx);
    break;

    case EFR32_RAC_IRQ_TXRX:
      efr32_rfp_req_done(ctx);
    break;

    default:
      UNREACHABLE();
    break;
  }
}





static error_t efr32_radio_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  error_t err;
  bool config_changed = false;

  if ((rq->rf_cfg != ctx->rf_cfg) || rq->rf_cfg->cache.dirty) {
    err = efr32_build_rf_config(ctx, rq);
    if (err != 0) {
      return err;
    }
    // Update rf cfg values
    ((struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg)->cache.dirty = 0;
    ctx->rf_cfg = rq->rf_cfg;
    config_changed = true;
  }
  if ((rq->pk_cfg != ctx->pk_cfg) || rq->pk_cfg->cache.dirty) {
    err = efr32_build_pkt_config(ctx, rq);
    if (err != 0) {
      return err;
    }
    // Update pk cfg values
    ((struct dev_rfpacket_pk_cfg_s *)rq->pk_cfg)->cache.dirty = 0;
    ctx->pk_cfg = rq->pk_cfg;
    config_changed = true;
  }
  // Calc params if config changed
  if (config_changed) {
  // Update synth ratio
  ctx->synth_ratio = efr32_calc_synth_ratio(ctx->curr_freq);
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_LDC
  // Update ldc values
    efr32_rfp_calc_ldc(ctx);
#endif
  }
  return 0;
}

static KROUTINE_EXEC(efr32_rfp_rxc_timeout_cb) {
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  struct radio_efr32_rfp_ctx_s *ctx = trq->pvdata;
  assert(ctx);
  efr32_radio_printk("rxc timeout\n");
  // Warn libdevice that we reached a rxc timeout
  dev_rfpacket_rxc_timeout(&ctx->gctx);
}

static void efr32_radio_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;

  cpu_mem_write_32(EFR32_SEQ_DEADLINE_ADDR, 0);
  // Check RAC state
  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  if (!isRetry) {
    assert(EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF);
  }
  // Configure AGC
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_RSSIPERIOD_SET(x, 5);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR);
  EFR32_AGC_CTRL0_MODE_SET(x, LOCKPREDET);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR, x);
  // Clear buffer
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
  // Set channel
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, rq->channel);
  // Check rq type
  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      // Cancel timeout irq before creating another one
      efr32_radio_timer_cancel(ctx, &ctx->rx_cont_trq);
      // Set timeout irq
      ctx->rx_cont_trq.deadline = rq->deadline;
      ctx->rx_cont_trq.rev = 0;
      ctx->rx_cont_trq.pvdata = ctx;
      dev_timer_rq_init(&ctx->rx_cont_trq, efr32_rfp_rxc_timeout_cb);
      if (efr32_radio_timer_req(ctx, &ctx->rx_cont_trq) == -ETIMEDOUT) {
          // Deadline already passed
          efr32_radio_printk("rxc timeout early\n");
          efr32_rfp_req_done_direct(ctx, DEV_RFPACKET_STATUS_RX_TIMEOUT);
          return;
      }
    case DEV_RFPACKET_RQ_RX_CONT:
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_LDC
      efr32_rfp_start_rx_ldc(ctx);
#else
      // Enable RX
      x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
      x |= 0x2;
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
#endif
    break;

    case DEV_RFPACKET_RQ_RX:
      // Scheduled RX
      efr32_rfp_start_rx_scheduled(ctx);
    break;

    default:
      UNREACHABLE();
    break;
  }
}

static void efr32_radio_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;

  cpu_mem_write_32(EFR32_SEQ_DEADLINE_ADDR, 0);
  // Check RAC state
  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  assert(EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF);
  // Check retry
  if (isRetry) {
    // Clear buffer
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
    // Restart tx lbt
    efr32_rfp_start_tx_lbt(ctx, rq);
    // TODO LBT FEATURES (no rx, lbt norm)
  }
  // Clear buffer
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
  // Write length when required
  if (ctx->isSigfox) {
    cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WCNTCMP0_ADDR, rq->tx_size - 1);
  } else {
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), rq->tx_size);
  }
  // Fill payload
  uint8_t *p = (uint8_t *)rq->tx_buf;
  for(uint16_t i = 0; i < rq->tx_size; i++) {
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), (uint32_t)p[i]);
  }
  // Set channel
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, rq->channel);
  // Set power
  efr32_set_tx_power(ctx, rq);
  // Enable TX
  switch (rq->type) {
    case DEV_RFPACKET_RQ_TX:
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXEN);
    break;

    case DEV_RFPACKET_RQ_TX_FAIR:
      efr32_rfp_start_tx_lbt(ctx, rq);
    break;

    default:
      UNREACHABLE();
    break;
  }
}

static void efr32_radio_cancel_rxc(struct dev_rfpacket_ctx_s *gpv) {
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  efr32_radio_printk("rxc canceled\n");
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_LDC
  // Stop rtcc
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, 0);
  // Flag cancel
  ctx->ldc_canceled = true;
  // Deactivate AFC
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_AFC_ADDR, 0x00000000);
#endif
  if (efr32_rfp_disable(ctx)) {
    // Disable already done, call req done
    efr32_rfp_req_done_direct(ctx, DEV_RFPACKET_STATUS_MISC);
  } else {
    // Disable is pending irq
    efr32_rfp_set_status_done(ctx, DEV_RFPACKET_STATUS_MISC);
    ctx->rac_irq_type = EFR32_RAC_IRQ_TXRX;
  }
}

static bool_t efr32_radio_wakeup(struct dev_rfpacket_ctx_s *gpv) {
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_SLEEP
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  // Wakeup radio if sleeping
  if (ctx->sleep) {
    for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++) {
      dev_clock_sink_gate(&pv->clk_ep[i], DEV_CLOCK_EP_POWER_CLOCK);
    }
    ctx->sleep = false;
  }
  return true;
  efr32_rfp_req_done_direct(ctx, DEV_RFPACKET_STATUS_MISC);
#else
  return false;
#endif
}

static bool_t efr32_radio_sleep(struct dev_rfpacket_ctx_s *gpv) {
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_SLEEP
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  // Put radio to sleep if awake
  if (!ctx->sleep) {
    for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++) {
       dev_clock_sink_gate(&pv->clk_ep[i], DEV_CLOCK_EP_POWER);
    }
    ctx->sleep = true;
  }
  return true;
#else
  return false;
#endif
}

static void efr32_radio_idle(struct dev_rfpacket_ctx_s *gpv) {
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_SLEEP
  struct radio_efr32_rfp_ctx_s *ctx = gpv->pvdata;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  // Delayed clock disable
  device_sleep_schedule(pv->dev);
#endif
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
  uint32_t idx = ep - ctx->pv.irq_ep;

  lock_spin(&dev->lock);
  // Set timestamp
  ctx->gctx.timestamp = efr32_protimer_get_value(&ctx->pv.pti);
  efr32_radio_printk("irq: %d\n", idx);
  // Process irq
  switch (idx) {
    case 0:
      irq = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IEN_ADDR);
      cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_IFC_ADDR, irq);
      efr32_radio_printk("modem irq: 0x%x\n", irq);
    break;

    case 1:
    case 2:
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IEN_ADDR, 0);
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFC_ADDR, EFR32_RAC_IF_STATECHANGE);
      efr32_radio_printk("rac irq: 0x%x\n", irq);
      efr32_rfp_rac_irq(ctx);
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

#ifdef CONFIG_DRIVER_EFR32_RFPACKET_LDC
    case 9:
      // Read rtcc irq reg
      irq = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_IF_ADDR);
      irq &= cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR);
      efr32_radio_printk("rtcc irq: 0x%x\n", irq);
      // Clear irq
      cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, endian_le32(EFM32_RTCC_IFC_MASK));
      // Process irq
      efr32_rfp_ldc_irq(ctx, irq);
      break;
#endif

    default:
      UNREACHABLE();
    break;
  }
  lock_release(&dev->lock);
}

static DEV_USE(efr32_radio_use) {
  struct device_s *dev = param;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;

  return dev_rfpacket_use(param, op, &ctx->gctx);
}

static DEV_RFPACKET_STATS(efr32_radio_stats) {
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  memcpy(stats, &ctx->gctx.stats, sizeof(*stats));
  return 0;
#else
  return -ENOTSUP;
#endif
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

  // Clock init
  if (dev_drv_clock_init(dev, &pv->pti.clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL)) {
    goto err_clk;
  }
  for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++) {
    if (dev_drv_clock_init(dev, &pv->clk_ep[i], i, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL)) {
      goto err_clk;
    }
  }
  pv->freq.num = CONFIG_DRIVER_EFR32_RADIO_HFXO_CLK;
  pv->freq.denom = 1;

  device_get_res_freq(dev, &pv->freq, 0);
  assert(pv->freq.denom == 1);
  // Queue init
  dev_rq_pqueue_init(&pv->pti.queue);
  // Sequencer code initialisaton
  efr32_radio_seq_init(pv, seqcode, 4 * seqcode_size);
  // Turn off radio
  efr32_rfp_disable(ctx);
  // Timer init
  efr32_protimer_init(&pv->pti);
  // Add specific rfpacket protimer initialisation
  uint32_t x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR);
  x |=  EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL) |
        EFR32_PROTIMER_IF_LBTFAILURE;
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR, x);
  // TC0 synchronized on PRECNTOF, prescaler decremented on PRECNTOF
  x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR);
  x |= EFR32_PROTIMER_CTRL_TOUT_SRC(0, PRECNTOF);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR, x);
  // TX/RX buffers initialization
  x = bit_ctz32(EFR32_RADIO_RFP_BUFFER_SIZE) - 6;

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(0), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(0), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(1), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(1), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(2), 0);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(2), (uint32_t)pv->rx_length_buffer);
  // Clear buffer
  for (uint8_t i = 0; i < 4; i++) {
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(i), EFR32_BUFC_CMD_CLEAR);
  }
  // Enable irq
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, EFR32_FRC_IF_MASK);
  x = EFR32_TX_IRQ_FRC_MSK | EFR32_RX_IRQ_FRC_MSK;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR, x);
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV
  // Setup pinmux
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, ">sel? >nsel?", loc, NULL, NULL))
    goto err_mem;

  x = EFR32_MODEM_ROUTEPEN_ANT0PEN | EFR32_MODEM_ROUTEPEN_ANT1PEN;
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_ROUTEPEN_ADDR, x);
  x = EFR32_MODEM_ROUTELOC1_ANT0LOC(loc[0]) | EFR32_MODEM_ROUTELOC1_ANT1LOC(loc[1]);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_ROUTELOC1_ADDR, x);
#endif
  // Interrupt init
  device_irq_source_init(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT, &efr32_radio_irq);
  if (device_irq_source_link(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT, -1)) {
    goto err_mem;
  }
  // Start timer
  dev->start_count |= 1;
  efr32_protimer_start_counter(&pv->pti);
#ifdef CONFIG_DRIVER_EFR32_DEBUG
  //efr32_rfp_cfg_rac_dbg(ctx);
  //efr32_radio_debug_init(pv);
  efr32_radio_debug_port(pv, 0x0);
#endif
  // Note pvdata and interface into generic context
  ctx->gctx.pvdata = ctx;
  ctx->gctx.drv = &efr32_radio_itfc;
  // Init generic context
  dev_rfpacket_init(&ctx->gctx);
  // Indicate that we finished init
  efr32_radio_printk("init end\n");
  efr32_rfp_req_done_direct(ctx, DEV_RFPACKET_STATUS_MISC);
  return 0;
err_clk:
  for (uint8_t i = 0; i < 8; i++) {
    dev_drv_clock_cleanup(dev, &pv->clk_ep[i]);
  }
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
 #ifdef CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV
  device_iomux_cleanup(dev);
#endif
    mem_free(ctx);
  }
  return err;
}