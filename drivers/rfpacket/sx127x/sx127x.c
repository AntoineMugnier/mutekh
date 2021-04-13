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

#define LOGK_MODULE_ID "sx12"

#include "sx127x_spi.h"
#include "sx127x_spi.o.h"

// Misc functions
static void sx127x_send_config(struct sx127x_private_s *pv);
static inline void sx127x_fill_status(struct sx127x_private_s *pv);
static void sx127x_fill_rx_info(struct sx127x_private_s *pv, struct dev_rfpacket_rx_s *rx);
static void sx127x_bytecode_start(struct sx127x_private_s *pv, const void *e, uint16_t mask, ...);
static void sx127x_clean(struct device_s *dev);
static uint32_t sx127x_set_tx_cmd(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq);
static uint32_t sx127x_set_rx_cmd(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq);
BC_CCALL_FUNCTION(sx127x_alloc);
BC_CCALL_FUNCTION(sx127x_next_hopping_freq);
// Lib device rfpacket interface functions
error_t sx127x_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value);
static error_t sx127x_check_config(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
static void sx127x_rx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void sx127x_tx(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
static void sx127x_cancel_rxc(struct dev_rfpacket_ctx_s *gpv);
static bool_t sx127x_wakeup(struct dev_rfpacket_ctx_s *gpv);
static bool_t sx127x_sleep(struct dev_rfpacket_ctx_s *gpv);
static void sx127x_idle(struct dev_rfpacket_ctx_s *gpv);

static const struct dev_rfpacket_driver_interface_s sx127x_itfc = {
  sx127x_get_time,
  sx127x_check_config,
  sx127x_rx,
  sx127x_tx,
  sx127x_cancel_rxc,
  sx127x_wakeup,
  sx127x_sleep,
  sx127x_idle,
};

/* ******************************** RNG **************************************/

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG) && defined(CONFIG_DRIVER_RFPACKET_SX127X_MOD_LORA)

static DEV_CRYPTO_INFO(sx127x_crypto_info)
{
  memset(info, 0, sizeof(*info));

  info->name       = "Semtech SX127x Transceiver";
  info->modes_mask = 1 << DEV_CRYPTO_MODE_RANDOM;
  info->state_size = CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE;

  return 0;
}

static void sx127x_crypto_rng_end(struct device_s * dev)
{
  struct sx127x_private_s * pv = dev->drv_pv;
  struct dev_crypto_rq_s *  rq = pv->crypto_rq;

  if (rq->op & DEV_CRYPTO_FINALIZE)
    memcpy(rq->out, rq->ctx->state_data,
           __MIN(rq->len, CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE));

  rq->error = 0;

  lock_release(&dev->lock);
  dev_rfpacket_rq_done(rq);
  lock_spin(&dev->lock);

  pv->crypto_rq = NULL;

  /* Process next request, if any */
  sx127x_next_request(dev);
}

static void sx127x_crypto_process_request(struct device_s * dev)
{
  struct sx127x_private_s * pv   = dev->drv_pv;
  struct dev_crypto_rq_s *  rq   = pv->crypto_rq;

  uint_fast8_t i;
  uint8_t *    state = rq->ctx->state_data;

  if (rq->op & DEV_CRYPTO_INIT)
    memset(rq->ctx->state_data, 0, CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE);

  if (rq->op & DEV_CRYPTO_INVERSE)
    {
      for (i = 0; i < __MIN(rq->ad_len, CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE); ++i)
        state[i] |= rq->ad[i];
    }

  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_rng, 0);
}

static DEV_CRYPTO_REQUEST(sx127x_crypto_request)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  rq->error = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->state != SX127X_STATE_IDLE)
    {
      rq->error = -EBUSY;
    }
  else
    {
      pv->crypto_rq = rq;
      sx127x_crypto_process_request(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (rq->error)
    dev_rfpacket_rq_done(rq);
}

#endif // CONFIG_DRIVER_CRYPTO_SX127X_RNG

/**************************** TIMER PART ********************************/

static DEV_TIMER_CANCEL(sx127x_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, cancel, rq);
}

static DEV_TIMER_REQUEST(sx127x_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, request, rq);
}

static DEV_TIMER_GET_VALUE(sx127x_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, get_value, value, rev);
}

static DEV_TIMER_CONFIG(sx127x_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, config, cfg, res);
}

error_t sx127x_get_time(struct dev_rfpacket_ctx_s *gpv, dev_timer_value_t *value)
{
  struct sx127x_private_s *pv = gpv->pvdata;
  return DEVICE_OP(pv->timer, get_value, value, 0);
}

/**************************** RFPACKET PART ********************************/

static void sx127x_send_config(struct sx127x_private_s *pv) {
  logk_trace("Send config to device");

  sx127x_bytecode_start(pv, &sx127x_entry_config,
    SX127X_ENTRY_CONFIG_BCARGS(pv->cfg_regs.cfg));

  // printk("Debug config size; %d\n %P\n", pv->cfg_offset, pv->cfg_regs.cfg, pv->cfg_offset/2);
  // printk("%P\n", pv->cfg_regs.cfg + pv->cfg_offset/2, pv->cfg_offset/2+1);
  // assert(false);
}

static inline void sx127x_fill_status(struct sx127x_private_s *pv) {
  pv->gctx.status = pv->bc_status;
}

static void sx127x_fill_rx_info(struct sx127x_private_s *pv, struct dev_rfpacket_rx_s *rx) {
  if (rx == NULL) {
    return;
  }
  rx->carrier = 0;
  rx->rssi = 0;
  rx->frequency = 0;
}

static void sx127x_bytecode_start(struct sx127x_private_s *pv, const void *e, uint16_t mask, ...) {
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  va_list ap;
  va_start(ap, mask);
  error_t err = dev_spi_bytecode_start_va(&pv->spi, srq, e, mask, ap);
  logk_trace("bcstart %d", err);
  ensure(err == 0);
  va_end(ap);
}

static void sx127x_clean(struct device_s *dev) {
  struct sx127x_private_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, pv->src_ep, 1);
  device_stop(&pv->timer->base);
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
  dev_rfpacket_clean(&pv->gctx);
#if defined(CONFIG_DRIVER_RFPACKET_SX127X_CRYPTO_RNG)
  dev_rq_queue_destroy(&pv->crypt_queue);
#endif
  mem_free(pv);
}

static uint32_t sx127x_set_tx_cmd(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  uint32_t cmd = pv->cfg_regs.sync;

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
    cmd |= CMD_IO_MODE;

  int16_t pwr = (rq->tx_pwr >> 3);
  /* Saturate TX power */
  if (pwr > 15)
    pwr = 15;
  if (pwr < 0)
    pwr = 0;
  /* Pa Select is false: Pout = OutputPower */
  cmd |= (0x70 | pwr) << 8;

  if (rq->channel == pv->cfg_regs.channel)
    return cmd;

  cmd |= CMD_FREQ_CHANGE;
  sx127x_config_freq(pv, rq->channel);

  return cmd;
}

static uint32_t sx127x_set_rx_cmd(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  uint32_t cmd = pv->cfg_regs.sync;

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
    cmd |= CMD_IO_MODE;

  switch (rq->type)
  {
      case DEV_RFPACKET_RQ_RX_CONT:
        if (rq->pk_cfg->format == DEV_RFPACKET_FMT_RAW)
          cmd |= CMD_RXC_RAW;
        if (rq->rx_chan_mask != 1)
        /* Rx scanning on several channels */
          cmd |= CMD_RX_SCANNING;
      case DEV_RFPACKET_RQ_RX:
        if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
          cmd += 1;
        break;

      default:
        UNREACHABLE();
  }
  if (rq->channel == pv->cfg_regs.channel)
    return cmd;

  cmd |= CMD_FREQ_CHANGE;
  sx127x_config_freq(pv, rq->channel);

  return cmd;
}




static error_t sx127x_check_config(struct dev_rfpacket_ctx_s *gpv,
                                   struct dev_rfpacket_rq_s *rq)
{
  struct sx127x_private_s *pv = gpv->pvdata;
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;
  bool_t conf_ok = true;

  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR)
  {
    // Check that fair tx mode is set when request of type DEV_RFPACKET_RQ_TX_FAIR is used
    if (!sx127x_config_check_fairtx_valid(rfcfg))
      return -ENOTSUP;

    // Set rx_tx flags
    pv->flags &= ~SX127X_FLAGS_RX_TX_OK;
    // Test if RX is allowed during TX
    if (dev_rfpacket_can_rxtx(&pv->gctx, rq))
      pv->flags |= SX127X_FLAGS_RX_TX_OK;

  }
  #ifdef CONFIG_DRIVER_RFPACKET_SX127X_MOD_LORA
    sx127x_lora_inverted_iq(pv, rq);
  #endif
  // Set configuration flags
  pv->flags &= ~SX127X_FLAGS_RF_CONFIG_OK;
  pv->flags &= ~SX127X_FLAGS_PK_CONFIG_OK;

  if ((pkcfg == pv->pk_cfg) && !pkcfg->cache.dirty)
    pv->flags |= SX127X_FLAGS_PK_CONFIG_OK;
  else
    conf_ok = false;

  if ((rfcfg == pv->rf_cfg) && !rfcfg->cache.dirty)
    pv->flags |= SX127X_FLAGS_RF_CONFIG_OK;
  else
    conf_ok = false;

  // Check if conf changed
  if (conf_ok)
    return 0;

  // Build conf
  error_t err = sx127x_build_config(pv);

  if (err != 0)
    return err;

  // Send conf
  sx127x_send_config(pv);
  return -EAGAIN;
}

static void sx127x_rx(struct dev_rfpacket_ctx_s *gpv,
                      struct dev_rfpacket_rq_s *rq, bool_t isRetry)
{
  struct sx127x_private_s *pv = gpv->pvdata;
  uint32_t cmd = sx127x_set_rx_cmd(pv, rq);

  if (isRetry)
  {
    sx127x_bytecode_start(pv, &sx127x_entry_rx,
      SX127X_ENTRY_RX_BCARGS(cmd));
    return;
  }

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_RX:
      sx127x_bytecode_start(pv, &sx127x_entry_rx,
        SX127X_ENTRY_RX_BCARGS(cmd));
    break;

    case DEV_RFPACKET_RQ_RX_TIMEOUT:
    case DEV_RFPACKET_RQ_RX_CONT:
      if (rq->type == DEV_RFPACKET_RQ_RX_TIMEOUT)
        pv->flags &= ~SX127X_FLAGS_RXC_INFINITE;
      else
        pv->flags |= SX127X_FLAGS_RXC_INFINITE;

      // Reset rssi average value
      pv->flags |= SX127X_FLAGS_RX_CONTINOUS;
      sx127x_bytecode_start(pv, &sx127x_entry_rx_cont,
        SX127X_ENTRY_RX_CONT_BCARGS(cmd));
    break;

    default:
      UNREACHABLE();
    break;
  }
}

static void sx127x_tx(struct dev_rfpacket_ctx_s *gpv,
                      struct dev_rfpacket_rq_s *rq, bool_t isRetry)
{
  struct sx127x_private_s *pv = gpv->pvdata;
  dev_timer_value_t t;
  uint32_t cmd = sx127x_set_tx_cmd(pv, rq);

  if (isRetry)
  {
    switch (rq->type)
    {
      case DEV_RFPACKET_RQ_TX_FAIR:
        // Take into account previous fifo fill
        if (pv->gctx.size < SX127X_FIFO_SIZE)
          pv->gctx.size = 0;
        else
        {
          pv->gctx.size -= SX127X_FIFO_SIZE - 1;
          pv->gctx.buffer += SX127X_FIFO_SIZE - 1;
        }
        sx127x_bytecode_start(pv, &sx127x_entry_retry_tx_fair,
          SX127X_ENTRY_RETRY_TX_FAIR_BCARGS(cmd));
      break;

      default:
        UNREACHABLE();
      break;
    }
  }
  else
  {
    switch (rq->type)
    {
      case DEV_RFPACKET_RQ_TX_FAIR:
        // Get time value
        DEVICE_OP(pv->timer, get_value, &t, 0);
        // Init lbt info
        pv->flags |= SX127X_FLAGS_TX_LBT;
        sx127x_bytecode_start(pv, &sx127x_entry_tx_fair,
         SX127X_ENTRY_TX_FAIR_BCARGS(cmd));
      break;

      case DEV_RFPACKET_RQ_TX:
        pv->flags &= ~SX127X_FLAGS_TX_LBT;
        sx127x_bytecode_start(pv, &sx127x_entry_tx,
          SX127X_ENTRY_TX_BCARGS(cmd));
      break;

      default:
        UNREACHABLE();
      break;
    }
  }
}

static void sx127x_cancel_rxc(struct dev_rfpacket_ctx_s *gpv)
{
  struct sx127x_private_s *pv = gpv->pvdata;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  pv->flags &= ~SX127X_FLAGS_RX_CONTINOUS;
  // Wakeup periodic rssi sampling
  dev_spi_bytecode_wakeup(&pv->spi, srq);
}

// Transceiver is sleeping when this function is called
static bool_t sx127x_wakeup(struct dev_rfpacket_ctx_s *gpv)
{
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_SLEEP
  struct sx127x_private_s *pv = gpv->pvdata;
  sx127x_bytecode_start(pv, &sx127x_entry_ready, 0, 0);
  return true;
#else
  return false;
#endif
}

static bool_t sx127x_sleep(struct dev_rfpacket_ctx_s *gpv)
{
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_SLEEP
  struct sx127x_private_s *pv = gpv->pvdata;
  sx127x_bytecode_start(pv, &sx127x_entry_sleep, 0, 0);
  return true;
#else
  return false;
#endif
}

static void sx127x_idle(struct dev_rfpacket_ctx_s *gpv)
{
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_SLEEP
  struct sx127x_private_s *pv = gpv->pvdata;
  // Delayed clock disable
  device_sleep_schedule(pv->dev);
#endif
}



BC_CCALL_FUNCTION(sx127x_alloc)
{
  struct sx127x_private_s *pv = (struct sx127x_private_s *)bc_get_reg(ctx, R_CTX_PV);
  logk_trace("RX alloc %d", pv->gctx.size);
  uintptr_t p = 0;

  LOCK_SPIN_IRQ(&pv->dev->lock);
  p = dev_rfpacket_alloc(&pv->gctx);
  LOCK_RELEASE_IRQ(&pv->dev->lock);

  return p;
}

BC_CCALL_FUNCTION(sx127x_next_hopping_freq)
{
  struct sx127x_private_s *pv = (struct sx127x_private_s *)bc_get_reg(ctx, R_CTX_PV);

  LOCK_SPIN_IRQ(&pv->dev->lock);

  assert(pv->gctx.rq);
  uint32_t mask = pv->gctx.rq->rx_chan_mask;

  uint32_t tmp = ~((1 << (pv->cfg_regs.channel + 1)) - 1) & mask;
  uint8_t next = tmp ? bit_ctz(tmp) : bit_ctz(mask);

  sx127x_config_freq(pv, next);

  bc_set_reg((struct bc_context_s *)ctx, 0, next);

  LOCK_RELEASE_IRQ(&pv->dev->lock);

  return 0;
}


static DEV_RFPACKET_REQUEST(sx127x_rfp_request)
{
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;

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

static DEV_RFPACKET_CANCEL(sx127x_rfp_cancel)
{
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  error_t err = 0;

  logk_trace("cancel");
  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);
  err = dev_rfpacket_cancel(&pv->gctx, rq);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static KROUTINE_EXEC(sx127x_spi_rq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->pvdata;
  struct sx127x_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);
  logk_trace("spi req done 0x%x", pv->bc_status);

  if (dev_rfpacket_init_done(&pv->gctx)) {
    assert(!srq->error);
  } else {
    if (srq->error) {
      // Couldn't initialize
      logk_trace("failed initialization");
      sx127x_clean(dev);
      device_async_init_done(dev, -EIO);
      goto end;
    } else {
      logk_trace("init done");
      device_async_init_done(dev, 0);
    }
  }

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
  if ((pv->crypto_rq != NULL) && (pv->bc_status == SX127X_BC_STATUS_RNG))
    {
      sx127x_crypto_rng_end(dev);
      goto end;
    }
#endif

  if (pv->bc_status == DEV_RFPACKET_STATUS_RX_DONE) {
    sx127x_fill_rx_info(pv, pv->gctx.rxrq);
  }
  sx127x_fill_status(pv);
  dev_rfpacket_req_done(&pv->gctx);
end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(sx127x_irq_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  lock_spin(&dev->lock);
  // Get timestamp
  DEVICE_OP(pv->timer, get_value, &pv->gctx.timestamp, 0);
  logk_trace("irq received");
  // Wakeup any waiting instruction
  dev_spi_bytecode_wakeup(&pv->spi, srq);
  lock_release(&dev->lock);
}

static DEV_USE(sx127x_use)
{
  struct device_s *dev = param;
  struct sx127x_private_s *pv = dev->drv_pv;

  return dev_rfpacket_use(param, op, &pv->gctx);
}

static DEV_RFPACKET_STATS(sx127x_rfp_stats)
{
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  memcpy(stats, &pv->stats, sizeof(*stats));
  return 0;
#else
  return -ENOTSUP;
#endif
}

/* ******************************** Device init/cleanup **********************/

static DEV_INIT(sx127x_init);
static DEV_CLEANUP(sx127x_cleanup);

DRIVER_DECLARE(sx127x_drv, 0, "Semtech SX127x Transceiver", sx127x,
               DRIVER_RFPACKET_METHODS(sx127x_rfp)
              ,DRIVER_TIMER_METHODS(sx127x_timer)
#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
              ,DRIVER_CRYPTO_METHODS(sx127x_crypto)
#endif
              );

DRIVER_REGISTER(sx127x_drv);

static DEV_INIT(sx127x_init)
{
  // Init memory
  struct sx127x_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  // Init status
  pv->bc_status = SX127X_BC_STATUS_MISC;

  // Init osc_ppb
  struct dev_freq_s osc;
  if (!device_get_res_freq(dev, &osc, 0))
    pv->osc_ppb = dev_freq_acc_ppb(&osc);
  else
    pv->osc_ppb = 20000;

  // Init bytecode
  struct dev_spi_ctrl_bytecode_rq_s * srq  = &pv->spi_rq;
  struct device_gpio_s *              gpio = NULL;

  static struct dev_spi_ctrl_config_s const spi_ctrl_cfg =
    {
      .bit_rate1k = CONFIG_DRIVER_RFPACKET_SX127X_SPI_BITRATE >> 10,
      .word_width = 8,
      .bit_order  = DEV_SPI_MSB_FIRST,
      .miso_pol   = DEV_SPI_ACTIVE_HIGH,
      .mosi_pol   = DEV_SPI_ACTIVE_HIGH,
      .cs_pol     = DEV_SPI_ACTIVE_LOW,
      .ck_mode    = DEV_SPI_CK_MODE_0,
    };

  if (dev_drv_spi_bytecode_init(dev, srq, &sx127x_bytecode, &spi_ctrl_cfg,
                                &pv->spi, &gpio, &pv->timer))
    goto err_mem;

  // Init timer
  dev_timer_init_sec(pv->timer, &pv->delay_1ms, 0, 1, 1000);
  dev_timer_init_sec(pv->timer, &pv->delay_5ms, 0, 5, 1000);

  // Start timer
  if (device_start(&pv->timer->base))
    goto err_srq;

  srq->base.cs_cfg.polarity = DEV_SPI_ACTIVE_LOW;
  srq->pvdata = dev;
  pv->dev = dev;

  // Init GPIO stuff
  static const gpio_width_t pin_wmap[4] = {1, 1, 1, 1};

  // FIXME lora dio3
  if (device_gpio_setup(gpio, dev, ">rst:1 <dio0:1 <dio3:1 <dio4:1", pv->pin_map, NULL))
    goto err_timer;

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_CRYPTO_RNG)
  dev_rq_queue_init(&pv->crypt_queue);
#endif

  // Init bytecode callback
  dev_spi_ctrl_rq_init(&srq->base, &sx127x_spi_rq_done);

  // Disable bytecode trace
  bc_set_trace(&srq->vm, 0);

  // Init irq
  device_irq_source_init(dev, pv->src_ep, 2, &sx127x_irq_source_process);

  if (device_irq_source_link(dev, pv->src_ep, 2, -1))
    goto err_timer;

  /* Disable DIO4 as irq */
  device_irq_src_disable(&pv->src_ep[1]);

  // Init generic context
  pv->gctx.pvdata = pv;
  pv->gctx.drv = &sx127x_itfc;
  dev_rfpacket_init(&pv->gctx);

  // Start bytecode
  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);
  sx127x_bytecode_start(pv, &sx127x_entry_reset, 0);

  // Init cache
#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
  pv->dirty = SX127X_CFG_CACHE_MSK_ALL;
#endif

  return -EAGAIN;

err_timer:
  device_stop(&pv->timer->base);
err_srq:
  dev_drv_spi_bytecode_cleanup(&pv->spi, srq);
err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(sx127x_cleanup)
{
  struct sx127x_private_s *pv = dev->drv_pv;
  error_t err = dev_rfpacket_clean_check(&pv->gctx);

  if (!err)
    sx127x_clean(dev);

  return err;
}