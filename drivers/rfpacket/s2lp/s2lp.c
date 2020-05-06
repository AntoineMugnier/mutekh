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
*/

#define LOGK_MODULE_ID "s2lp"

#include "s2lp.h"
#include "s2lp_regs.h"
#include "s2lp_config.h"
#include "s2lp_spi.o.h"

static const struct dev_spi_ctrl_config_s spi_cfg = {
  .ck_mode = DEV_SPI_CK_MODE_0,
  .bit_order = DEV_SPI_MSB_FIRST,
  .miso_pol = DEV_SPI_ACTIVE_HIGH,
  .mosi_pol = DEV_SPI_ACTIVE_HIGH,
  .cs_pol   = DEV_SPI_ACTIVE_LOW,
  .bit_rate1k = CONFIG_DRIVER_RFPACKET_S2LP_SPI_BITRATE >> 10,
  .word_width = 8,
};

// Misc functions
static void s2lp_send_config(struct s2lp_ctx_s *pv);
static dev_timer_delay_t s2lp_calc_lbt_rand_time(dev_timer_value_t timebase, dev_timer_value_t curr_time);
static inline void s2lp_fill_status(struct s2lp_ctx_s *pv);
static void s2lp_fill_rx_info(struct s2lp_ctx_s *pv, struct dev_rfpacket_rx_s *rx);
static void s2lp_bytecode_start(struct s2lp_ctx_s *pv, const void *e, uint16_t mask, ...);
static void s2lp_clean(struct device_s *dev);
// Lib device rfpacket interface functions
error_t s2lp_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value);
static error_t s2lp_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
static void s2lp_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void s2lp_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void s2lp_cancel_rxc(struct dev_rfpacket_ctx_s *gpv);
static bool_t s2lp_wakeup(struct dev_rfpacket_ctx_s *gpv);
static bool_t s2lp_sleep(struct dev_rfpacket_ctx_s *gpv);
static void s2lp_idle(struct dev_rfpacket_ctx_s *gpv);

static const struct dev_rfpacket_driver_interface_s s2lp_itfc = {
  s2lp_get_time,
  s2lp_check_config,
  s2lp_rx,
  s2lp_tx,
  s2lp_cancel_rxc,
  s2lp_wakeup,
  s2lp_sleep,
  s2lp_idle,
};

/**************************** TIMER PART ********************************/

static DEV_TIMER_CANCEL(s2lp_timer_cancel) {
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, cancel, rq);
}

static DEV_TIMER_REQUEST(s2lp_timer_request) {
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, request, rq);
}

static DEV_TIMER_GET_VALUE(s2lp_timer_get_value) {
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, get_value, value, rev);
}

static DEV_TIMER_CONFIG(s2lp_timer_config) {
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, config, cfg, res);
}

error_t s2lp_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value) {
  struct s2lp_ctx_s *pv = gpv->pvdata;
  return DEVICE_OP(pv->timer, get_value, value, 0);
}

/**************************** RFPACKET PART ********************************/

static void s2lp_send_config(struct s2lp_ctx_s *pv) {
  logk_trace("Send config to device");

  s2lp_bytecode_start(pv, &s2lp_entry_config,
    S2LP_ENTRY_CONFIG_BCARGS(pv->curr_rf_cfg_data, pv->curr_pk_cfg_data));
}

static dev_timer_delay_t s2lp_calc_lbt_rand_time(dev_timer_value_t timebase, dev_timer_value_t curr_time) {
  uint8_t mult = rand_64_range_r(&curr_time, 0, S2LP_LBT_RAND_TIME_MAX_MULT);
  return timebase * (mult + S2LP_LBT_BASE_TIME_MULT);
}

static inline void s2lp_fill_status(struct s2lp_ctx_s *pv) {
  pv->gctx.status = pv->bc_status;
}

static void s2lp_fill_rx_info(struct s2lp_ctx_s *pv, struct dev_rfpacket_rx_s *rx) {
  if (rx == NULL) {
    return;
  }
  rx->carrier = S2LP_GET_RSSI(pv->carrier) << 3;
  rx->rssi = S2LP_GET_RSSI(pv->avg_rssi) << 3;
  rx->frequency = pv->frequency + pv->afc_offset;
}

static void s2lp_bytecode_start(struct s2lp_ctx_s *pv, const void *e, uint16_t mask, ...) {
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  va_list ap;
  va_start(ap, mask);
  error_t err = dev_spi_bytecode_start_va(&pv->spi, srq, e, mask, ap);
  logk_trace("bcstart %d", err);
  ensure(err == 0);
  va_end(ap);
}

static void s2lp_clean(struct device_s *dev) {
  struct s2lp_ctx_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, pv->src_ep, 1);
  device_stop(&pv->timer->base);
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
  dev_rfpacket_clean(&pv->gctx);
  mem_free(pv);
}





static error_t s2lp_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq) {
  struct s2lp_ctx_s *pv = gpv->pvdata;
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;
  bool_t conf_ok = true;

  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR) {
    // Check that fair tx mode is set when request of type DEV_RFPACKET_RQ_TX_FAIR is used
    if (!s2lp_config_check_fairtx_valid(rfcfg)) {
      return -ENOTSUP;
    }
    // Set rx_tx flags
    pv->flags &= ~S2LP_FLAGS_RX_TX_OK;
    // Test if RX is allowed during TX
    if (dev_rfpacket_can_rxtx(&pv->gctx, rq)) {
      pv->flags |= S2LP_FLAGS_RX_TX_OK;
    }
  }
  // Set configuration flags
  pv->flags &= ~S2LP_FLAGS_RF_CONFIG_OK;
  pv->flags &= ~S2LP_FLAGS_PK_CONFIG_OK;

  if ((pkcfg == pv->pk_cfg) && !pkcfg->cache.dirty) {
    pv->flags |= S2LP_FLAGS_PK_CONFIG_OK;
  } else {
    ((struct dev_rfpacket_pk_cfg_s *)pkcfg)->cache.dirty = 0;
    pv->pk_cfg = pkcfg;
    conf_ok = false;
  }
  if ((rfcfg == pv->rf_cfg) && !rfcfg->cache.dirty) {
    pv->flags |= S2LP_FLAGS_RF_CONFIG_OK;
  } else {
    ((struct dev_rfpacket_rf_cfg_s *)rfcfg)->cache.dirty = 0;
    pv->rf_cfg = rfcfg;
    conf_ok = false;
  }
  // Check if conf changed
  if (conf_ok) {
    return 0;
  }
  // Build conf
  error_t err = s2lp_build_config(pv);

  if (err != 0) {
    return err;
  }
  s2lp_send_config(pv);
  return -EAGAIN;
}

static void s2lp_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry) {
  struct s2lp_ctx_s *pv = gpv->pvdata;
  struct dev_rfpacket_rf_cfg_s *cfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
  uint32_t freq = s2lp_config_get_freq(cfg, rq->channel);

  if (isRetry) {
    s2lp_bytecode_start(pv, &s2lp_entry_rx, S2LP_ENTRY_RX_BCARGS(rq->channel));
    return;
  }
  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX:
      pv->frequency = freq;
      s2lp_bytecode_start(pv, &s2lp_entry_rx, S2LP_ENTRY_RX_BCARGS(rq->channel));
    break;

    case DEV_RFPACKET_RQ_RX_TIMEOUT:
    case DEV_RFPACKET_RQ_RX_CONT:
      if (rq->type == DEV_RFPACKET_RQ_RX_TIMEOUT) {
        pv->flags &= ~S2LP_FLAGS_RXC_INFINITE;
      } else {
        pv->flags |= S2LP_FLAGS_RXC_INFINITE;
      }
      if (pv->frequency != freq) {
          pv->frequency = freq;
      }
      // Reset rssi average value
      pv->avg_rssi = S2LP_SET_RSSI(S2LP_RSSI_AVG_DEF_VAL);
      pv->flags |= S2LP_FLAGS_RX_CONTINOUS;
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_LDC
      s2lp_bytecode_start(pv, &s2lp_entry_rxc_ldc, S2LP_ENTRY_RX_CONT_BCARGS(rq->channel));
#else
      s2lp_bytecode_start(pv, &s2lp_entry_rx_cont, S2LP_ENTRY_RX_CONT_BCARGS(rq->channel));
#endif
    break;

    default:
      UNREACHABLE();
    break;
  }
}

static void s2lp_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry) {
  struct s2lp_ctx_s *pv = gpv->pvdata;
  uint8_t pwr;
  dev_timer_value_t t;

  if (isRetry) {
    switch (rq->type) {
      case DEV_RFPACKET_RQ_TX_FAIR:
        // Take into account previous fifo fill
        if (pv->gctx.size < S2LP_FIFO_SIZE) {
          pv->gctx.size = 0;
        } else {
          pv->gctx.size -= S2LP_FIFO_SIZE - 1;
          pv->gctx.buffer += S2LP_FIFO_SIZE - 1;
        }
        pv->lbt_state = S2LP_LBT_STATE_FREE;
        s2lp_bytecode_start(pv, &s2lp_entry_retry_tx_lbt, S2LP_ENTRY_RETRY_TX_LBT_BCARGS());
      break;

      default:
        UNREACHABLE();
      break;
    }
  } else {
    switch (rq->type) {
      case DEV_RFPACKET_RQ_TX_FAIR:
        // Get time value
        DEVICE_OP(pv->timer, get_value, &t, 0);
        // Init lbt info
        pv->lbt_state = S2LP_LBT_STATE_FREE;
        pv->lbt_rand_time = s2lp_calc_lbt_rand_time(pv->bt, t);
        pv->flags |= S2LP_FLAGS_TX_LBT;
        pwr = s2lp_build_pwr(pv, rq->tx_pwr);
        s2lp_bytecode_start(pv, &s2lp_entry_tx_lbt, S2LP_ENTRY_TX_LBT_BCARGS(pwr, rq->channel));
      break;

      case DEV_RFPACKET_RQ_TX:
        pv->flags &= ~S2LP_FLAGS_TX_LBT;
        pwr = s2lp_build_pwr(pv, rq->tx_pwr);
        s2lp_bytecode_start(pv, &s2lp_entry_tx, S2LP_ENTRY_TX_BCARGS(pwr, rq->channel));
      break;

      default:
        UNREACHABLE();
      break;
    }
  }
}

static void s2lp_cancel_rxc(struct dev_rfpacket_ctx_s *gpv) {
  struct s2lp_ctx_s *pv = gpv->pvdata;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  pv->flags &= ~S2LP_FLAGS_RX_CONTINOUS;
  // Wakeup periodic rssi sampling
  dev_spi_bytecode_wakeup(&pv->spi, srq);
}

// Transceiver is sleeping when this function is called
static bool_t s2lp_wakeup(struct dev_rfpacket_ctx_s *gpv) {
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_SLEEP
  struct s2lp_ctx_s *pv = gpv->pvdata;
  s2lp_bytecode_start(pv, &s2lp_entry_ready, 0, 0);
  return true;
#else
  return false;
#endif
}

static bool_t s2lp_sleep(struct dev_rfpacket_ctx_s *gpv) {
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_SLEEP
  struct s2lp_ctx_s *pv = gpv->pvdata;
  s2lp_bytecode_start(pv, &s2lp_entry_sleep, 0, 0);
  return true;
#else
  return false;
#endif
}

static void s2lp_idle(struct dev_rfpacket_ctx_s *gpv) {
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_SLEEP
  struct s2lp_ctx_s *pv = gpv->pvdata;
  // Delayed clock disable
  device_sleep_schedule(pv->dev);
#endif
}




BC_CCALL_FUNCTION(s2lp_alloc) {
  struct s2lp_ctx_s *pv = (struct s2lp_ctx_s *)bc_get_reg(ctx, S2LP_CTX_PV);
  logk_trace("RX alloc %d", pv->gctx.size);
  uintptr_t p = 0;

  LOCK_SPIN_IRQ(&pv->dev->lock);
  p = dev_rfpacket_alloc(&pv->gctx);
  LOCK_RELEASE_IRQ(&pv->dev->lock);

  return p;
}

static DEV_RFPACKET_REQUEST(s2lp_rfp_request) {
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  va_list vl;
  va_start(vl, accessor);

  while(1) {
    struct dev_rfpacket_rq_s *rq = va_arg(vl, struct dev_rfpacket_rq_s *);

    if (rq == NULL) {
      break;
    }
    rq->error = 0;
    dev_rfpacket_request(&pv->gctx, rq);
  }
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_RFPACKET_CANCEL(s2lp_rfp_cancel) {
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  error_t err = 0;

  logk_trace("cancel");
  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);
  err = dev_rfpacket_cancel(&pv->gctx, rq);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static KROUTINE_EXEC(s2lp_spi_rq_done) {
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->pvdata;
  struct s2lp_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);
  logk_trace("spi req done 0x%x", pv->bc_status);

  if (dev_rfpacket_init_done(&pv->gctx)) {
    assert(!srq->error);
  } else {
    if (srq->error) {
      // Couldn't initialize
      logk_trace("failed initialization");
      s2lp_clean(dev);
      device_async_init_done(dev, -EIO);
      goto end;
    } else {
      logk_trace("init done");
      device_async_init_done(dev, 0);
    }
  }
  if (pv->bc_status == DEV_RFPACKET_STATUS_RX_DONE) {
    s2lp_fill_rx_info(pv, pv->gctx.rxrq);
  }
  s2lp_fill_status(pv);
  dev_rfpacket_req_done(&pv->gctx);
end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(s2lp_irq_source_process) {
  struct device_s *dev = ep->base.dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  lock_spin(&dev->lock);
  // Get timestamp
  DEVICE_OP(pv->timer, get_value, &pv->gctx.timestamp, 0);
  // Wakeup any waiting instruction
  dev_spi_bytecode_wakeup(&pv->spi, srq);
  lock_release(&dev->lock);
}

static DEV_USE(s2lp_use) {
  struct device_s *dev = param;
  struct s2lp_ctx_s *pv = dev->drv_pv;

  return dev_rfpacket_use(param, op, &pv->gctx);
}

static DEV_RFPACKET_STATS(s2lp_rfp_stats) {
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  struct device_s *dev = accessor->dev;
  struct s2lp_ctx_s *pv = dev->drv_pv;
  memcpy(stats, &pv->stats, sizeof(*stats));
  return 0;
#else
  return -ENOTSUP;
#endif
}

static DEV_INIT(s2lp_init);
static DEV_CLEANUP(s2lp_cleanup);

DRIVER_DECLARE(s2lp_drv, 0, "ST Micro s2lp transceiver", s2lp,
               DRIVER_RFPACKET_METHODS(s2lp_rfp),
               DRIVER_TIMER_METHODS(s2lp_timer));

DRIVER_REGISTER(s2lp_drv);

static DEV_INIT(s2lp_init) {
  // Init memory
  struct s2lp_ctx_s *pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv){
    return -ENOMEM;
  }
  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  // Init status
  pv->bc_status = S2LP_BC_STATUS_MISC;

  // Init config structs
  s2lp_init_rf_cfg_array(pv->rf_cfg_array, S2LP_RF_CFG_ARRAY_SIZE);
  s2lp_init_pk_cfg_array(pv->pk_cfg_array, S2LP_PK_CFG_ARRAY_SIZE);

  // Init bytecode
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;
  struct device_gpio_s *gpio;

  if (dev_drv_spi_bytecode_init(dev, srq, &s2lp_bytecode, &spi_cfg, &pv->spi, &gpio, &pv->timer)) {
    goto err_mem;
  }
  // Base 500 us time
  dev_timer_init_sec(pv->timer, &pv->bt, 0, S2LP_BASE_TIME, 1000000);
  // Start timer
  if (device_start(&pv->timer->base)) {
    goto err_srq;
  }
  srq->pvdata = dev;
  pv->dev = dev;

  // Init GPIO stuff
  static const gpio_width_t pin_wmap[] = {1, 1};

  if (device_gpio_setup(gpio, dev, ">sdn:1 <nirq:1", pv->pin_map, NULL)) {
    goto err_timer;
  }
  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

  // Init bytecode callback
  dev_spi_ctrl_rq_init(&srq->base, &s2lp_spi_rq_done);

  // Init irq
  device_irq_source_init(dev, pv->src_ep, 1, &s2lp_irq_source_process);
  if (device_irq_source_link(dev, pv->src_ep, 1, 0x3)) {
    goto err_timer;
  }
  // Note pvdata and interface into generic context
  pv->gctx.pvdata = pv;
  pv->gctx.drv = &s2lp_itfc;
  // Init generic context
  dev_rfpacket_init(&pv->gctx);

  // Start bytecode
  bc_set_reg(&srq->vm, S2LP_CTX_PV, (uintptr_t)pv);
  s2lp_bytecode_start(pv, &s2lp_entry_reset, S2LP_ENTRY_RESET_BCARGS(pv->bt, s2lp_config));
  return -EAGAIN;

 err_timer:
  device_stop(&pv->timer->base);
 err_srq:
  dev_drv_spi_bytecode_cleanup(&pv->spi, srq);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(s2lp_cleanup) {
  struct s2lp_ctx_s *pv = dev->drv_pv;
  error_t err = dev_rfpacket_clean_check(&pv->gctx);

  if (!err) {
    s2lp_clean(dev);
  }
  return err;
}