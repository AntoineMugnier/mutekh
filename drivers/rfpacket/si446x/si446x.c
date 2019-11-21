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

    Copyright (c) 2014 Alexandre Becoulet <alexandre.becoulet@free.fr>
    Copyright (c) 2014 Sebastien Cerdan <sebcerdan@gmail.com>

*/

/* TODO Optimisation driver
  Study unnecessary operations (abor rx, flush fifo, clean irq)
*/


#define LOGK_MODULE_ID "si44"

#include "si446x.h"
#include "si446x_spi.o.h"

/**************************** TIMER PART ********************************/

static DEV_TIMER_CANCEL(si446x_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, cancel, rq);
}

static DEV_TIMER_REQUEST(si446x_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, request, rq);
}

static DEV_TIMER_GET_VALUE(si446x_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, get_value, value, rev);
}

static DEV_TIMER_CONFIG(si446x_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;
  return DEVICE_OP(pv->timer, config, cfg, res);
}


/**************************** RFPACKET PART ********************************/

static dev_timer_delay_t si446x_calc_lbt_rand_rime(dev_timer_value_t timebase, dev_timer_value_t curr_time) {
    uint8_t mult = rand_64_range_r(&curr_time, 0, SI446X_LBT_RAND_TIME_MAX_MULT);
    return timebase * (mult + SI446X_LBT_BASE_TIME_MULT);
}

static void si446x_dump_config(uint8_t *ptr, const uint8_t *c)
{
  uint16_t i = 0;
  size_t size;

  while ((size = c[i]))
    {
      logk_trace("cfg: 11 %02x %02x %02x : %P", c[i + 1], size, c[i + 2],
                 ptr + c[i + 3], size);
      i += 4;
    }
}

static inline void si446x_rfp_set_state(struct si446x_ctx_s *pv, enum si446x_state_s state)
{
  logk_trace("state %d", state);
  pv->state = state;
}

static inline uint8_t si446X_get_pwr_lvl(struct si446x_ctx_s *pv, int16_t pwr)
{
  if (pwr == pv->pwr)
  /* TX power must not be reconfigured */
    return 0;

  pv->flags |= SI446X_FLAGS_TX_POWER;
  pv->pwr = pwr;

  /* TX power in 0.125 dbm with a 40 dbm offset */

  int16_t p = pwr + (SI446X_MIN_TX_POWER << 3);

  if (p < 0)
    p = 0;

  uint16_t max = (SI446X_MIN_TX_POWER +  SI446X_MAX_TX_POWER) << 3;

  if (p > max)
    p = max;

  /* Register value  for a 4dbm step from -40 to 13 dbm */
  static const uint8_t pa_pwr_lvl[14] = {0, 1, 1, 2, 3, 4, 6, 8, 12, 18, 27, 43, 68, 152};
  /*                                            -12 -8  -4   0   4   8   12   dbm */

  uint8_t idx = p >> 5;
  uint16_t xa = idx << 5;
  uint16_t t = ((pa_pwr_lvl[idx + 1] - pa_pwr_lvl[idx]) << 4) >> 5;

  t = pa_pwr_lvl[idx] + (((p - xa) * t) >> 4);

  t = (t > 127) ? 127 : t;

  return (uint8_t)t;
}


static void si446x_bytecode_start(struct si446x_ctx_s *pv, const void *e, uint16_t mask, ...)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  /* Clear status */
  pv->bc_status &= STATUS_IRQ_MSK;
  bc_set_reg(&srq->vm, STATUS, pv->bc_status);

  dev_timer_value_t ts;
  DEVICE_OP(pv->timer, get_value, &ts, 0);
  logk_trace("[%d] bcstart", (uint32_t)ts);
  va_list ap;
  va_start(ap, mask);
  ensure(dev_spi_bytecode_start_va(&pv->spi, srq, e, mask, ap) == 0);
  va_end(ap);
}

/* Send a new packet configuration to device */

static inline error_t si446x_build_pk_config(struct si446x_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{
  struct si446x_pkt_regs_s * pkt = &pv->pk_buff;

  logk_trace("PKT configuration");

  if (rq->pk_cfg->format != DEV_RFPACKET_FMT_SLPC)
    return -ENOTSUP;

  const struct dev_rfpacket_pk_cfg_basic_s *cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

  /** Configure Sync Word */

  uint8_t sw = cfg->sw_len + 1;

  if ((sw & 0x7) || !(sw >> 3))
    return -ENOTSUP;

  sw = sw >> 3;

  pkt->sw.len = sw - 1;
  for (int8_t i=0; i <sw; i++)
    {
      uint8_t x = cfg->sw_value >> ((sw - i - 1) * 8);

      x = ((x & 0x55) << 1) | ((0xAA & x) >> 1);
      x = ((x & 0x33) << 2) | ((0xCC & x) >> 2);
      x = ((x & 0x0F) << 4) | ((0xF0 & x) >> 4);

      pkt->sw.val[i] = x;
    }

  /** Configure Preamble */

  uint32_t msk = (1 << (cfg->pb_pattern_len + 1)) - 1;
  uint32_t preamble = cfg->pb_pattern & msk;

  if (cfg->tx_pb_len > 255 * 8)
    return -ENOTSUP;

  pkt->preamble.tx_length = cfg->tx_pb_len >> 3;
  pkt->preamble.config_std_1 = SI446X_RX_THRESHOLD;
  pkt->preamble.config_nstd = 0;
  pkt->preamble.config_std_2 = 0;

  if (preamble == (0xAAAAAAAA & msk))
    pkt->preamble.config = (1 << 5) | (1 << 4) | (1 << 0);
  else if (preamble == (0x55555555 & msk))
    pkt->preamble.config = (2 << 0) | (1 << 4) | (1 << 1);
  else
    return -ENOTSUP;

  /** Configure CRC */

  /* only all 0 or all 1s are supported for CRC seed */
  switch (cfg->crc_seed)
    {
    case 0xff:
    case 0xffff:
    case 0xffffffff:
      pkt->crc = 0x80;
      break;

    case 0x00:
      pkt->crc = 0;
      break;

    default:
      return -ENOTSUP;
    }

  static const uint32_t crcs[] = {
    0, 0x07, 0x5B93, 0x90D9, 0x8005, 0x1021,
    0x741b8cd7, 0x04c11db7, 0x1edc6f41, 0x3D65
  };

  for (uint_fast8_t i = 0; i < ARRAY_SIZE(crcs); i++)
    if (crcs[i] == cfg->crc)
      {
        pkt->crc |= i;
        goto crc_ok;
      }
  return -ENOTSUP;
 crc_ok:

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_WUT
  /*
    Trxp = Tldc + Twut
    Tldc = Twup + Trx

    Tldc:    Wake-up + Rx time
    Twup:    Wake-up time
    Trx:     Effective RX time
    Twut:    Sleep time
    Trxp:    RX preamble time

     __________                   __
    |          |                 |
    |          |_________________|

    <---><----->
     Twup  Trx
    <----------> <-------------->
       Tldc         Twut
    <--------------------------->
               Trxp
  */

  pkt->clk = 0;

  if (cfg->rx_pb_len > SI446X_RX_THRESHOLD)
    {
      /* Time bit in ns */
      dev_timer_delay_t tb = 1000000000 / rq->rf_cfg->drate;

      uint32_t trx = (SI446X_RX_THRESHOLD + 8 /* Margin */) * tb;
      uint32_t tldc = trx + SI446X_BASE_TIME * 1000;

      /* Get the wake-up period in number of 122 us */
      pkt->wut.ldc = (SI446X_WUT_MIN_TIME - 1 + tldc) / SI446X_WUT_MIN_TIME;

      int32_t twut = (cfg->rx_pb_len - 64 /* Margin */) * tb - pkt->wut.ldc * SI446X_WUT_MIN_TIME;

      logk_trace("twut: %i\n", twut);

      if (twut > 0)
        {
          endian_be16_na_store(pkt->wut.m, twut / SI446X_WUT_MIN_TIME);
          pkt->wut.r = 0x20 | SI446X_WUTR_VALUE;
          pkt->clk = 0x01;

          logk_trace("ldc: %d", tldc);
          logk_trace("sleep_time: %d", twut);
          logk_trace("rx_time: %d", trx);
        }
    }
#endif

  si446x_dump_config((uint8_t*)pkt, si446x_pk_cmd);

  return 0;
}

static bool_t
si446x_modem_configure(struct si446x_ctx_s *pv,
                       struct si446x_rf_regs_s *out,
                       const struct dev_rfpacket_rf_cfg_s *rf_cfg,
                       const struct dev_rfpacket_pk_cfg_s *pk_cfg)
{
  static const uint8_t tab[] = {
    [DEV_RFPACKET_FSK * 2] = MOD_2FSK,
    [DEV_RFPACKET_FSK * 2 + 1] = MOD_4FSK,
    [DEV_RFPACKET_GFSK * 2] = MOD_2GFSK,
    [DEV_RFPACKET_GFSK * 2 + 1] = MOD_4GFSK,
    [DEV_RFPACKET_ASK * 2] = MOD_OOK,
  };

  uint8_t idx = rf_cfg->mod * 2;
  uint32_t fdev = 0;

  switch (rf_cfg->mod)
    {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      {
        const struct dev_rfpacket_rf_cfg_fsk_s * c =
          const_dev_rfpacket_rf_cfg_fsk_s_cast(rf_cfg);

        switch (c->symbols)
          {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK
          case 2:
            break;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK
          case 4:
            break;
#endif
          default:
            return 0;
          }

        fdev = c->deviation;

        idx += (c->symbols == 4);
        break;
      }
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_00K
    case DEV_RFPACKET_ASK:
      {
        const struct dev_rfpacket_rf_cfg_ask_s * c =
          const_dev_rfpacket_rf_cfg_ask_s_cast(rf_cfg);

        if (c->symbols != 2)
          return 0;

        break;
      }
#endif
    default:
      return 0;
    }

  bool_t manchester;

  switch (pk_cfg->encoding)
    {
    case DEV_RFPACKET_CLEAR:
      manchester = 0;
      break;
    case DEV_RFPACKET_MANCHESTER:
      manchester = 1;
      break;
    default:
      return 0;
    }

  uint32_t rx_tx_freq_err = rf_cfg->freq_err   /* remote freq error */
    + (uint64_t)rf_cfg->frequency * pv->osc_ppb / 1000000000; /* local freq error */

  return modem_calc(out, &pv->synth_ratio, tab[idx], rf_cfg->frequency, rf_cfg->drate,
                    fdev, rf_cfg->rx_bw, rf_cfg->chan_spacing, rx_tx_freq_err, manchester);
}

static inline error_t si446x_build_rf_config(struct si446x_ctx_s *pv,
                                             struct si446x_rf_regs_s *out,
                                             struct dev_rfpacket_rq_s *rq)
{
  logk_trace("RF configuration");

  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  if (!si446x_modem_configure(pv, out, rq->rf_cfg, rq->pk_cfg))
    return -ENOTSUP;

  /** Configure RSSI_threshold */

  const struct dev_rfpacket_rf_cfg_fairtx_s *fairtx = NULL;

  switch (cfg->mod)
    {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      {
        const struct dev_rfpacket_rf_cfg_fsk_s * c =
          const_dev_rfpacket_rf_cfg_fsk_s_cast(cfg);
        fairtx = &c->fairtx;
        break;
      }
    case DEV_RFPACKET_ASK:
      {
        const struct dev_rfpacket_rf_cfg_ask_s * c =
          const_dev_rfpacket_rf_cfg_ask_s_cast(cfg);
        fairtx = &c->fairtx;
        break;
      }
    default:
      break;
    }

  int16_t rssi_th = SI446X_MAX_RSSI_VALUE;

  if (fairtx && fairtx->mode == DEV_RFPACKET_LBT)
    rssi_th = fairtx->lbt.rssi;

  if (rssi_th > SI446X_MAX_RSSI_VALUE)
    return -ENOTSUP;

  out->rssi_th = SET_RSSI(rssi_th >> 3);
  pv->lbt_rssi = out->rssi_th;

  if (cfg->jam_rssi > SI446X_MAX_RSSI_VALUE)
    return -ENOTSUP;

  pv->jam_rssi = SET_RSSI(cfg->jam_rssi >> 3);

  si446x_dump_config((uint8_t*)out, si446x_rf_cmd);

  return 0;
}

static void si446x_rf_config_done(struct si446x_ctx_s *pv, const struct dev_rfpacket_rf_cfg_s *cfg)
{
  pv->id = cfg->cache.id;

  struct si446x_cache_entry_s *e = &pv->cache_array[pv->id % SI446X_RF_CONFIG_CACHE_ENTRY];
  struct si446x_rf_regs_s *data = &e->data;

  /* Maximum time to wait before aborting transfer */
  pv->mpst = 400 * e->tb;
  /* 8 * Time bit + SLEEP->RX time*/
  pv->ccad = e->tb + pv->bt;

  logk_trace("ccad : %d", pv->ccad);

#ifdef CONFIG_ARCH_SOCLIB
  pv->mpst *= 10;
  pv->ccad *= 10;
#endif

  si446x_bytecode_start(pv, &si446x_entry_rf_config, SI446X_ENTRY_RF_CONFIG_BCARGS(
                                     (uintptr_t)data, (uintptr_t)si446x_rf_cmd));
}

static void si446x_rfp_end_rq(struct si446x_ctx_s *pv, error_t err);
static void si446x_rfp_end_rxc(struct si446x_ctx_s *pv, error_t err);

static void si446x_rf_config_notsup(struct si446x_ctx_s *pv,
                                    struct dev_rfpacket_rq_s *rq)
{
  switch (rq->type)
    {
    case DEV_RFPACKET_RQ_RX_CONT:
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      return si446x_rfp_end_rxc(pv, -ENOTSUP);
    default:
      return si446x_rfp_end_rq(pv, -ENOTSUP);
    }
}

static KROUTINE_EXEC(si446x_config_deferred)
{
  struct si446x_ctx_s *pv = si446x_ctx_s_from_kr(kr);
  struct dev_rfpacket_rq_s *rq = pv->rq;

  if (rq->type == DEV_RFPACKET_RQ_RX_CONT ||
      rq->type == DEV_RFPACKET_RQ_RX_TIMEOUT)
    assert((pv->state == SI446X_STATE_CONFIG_RXC) ||
           (pv->state == SI446X_STATE_CONFIG_RXC_PENDING_STOP));
  else
    assert(pv->state == SI446X_STATE_CONFIG);

  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;
  struct si446x_cache_entry_s *e = &pv->cache_array[cfg->cache.id % SI446X_RF_CONFIG_CACHE_ENTRY];

  if (si446x_build_rf_config(pv, &e->data, rq) == 0)
    return si446x_rf_config_done(pv, cfg);

  si446x_rf_config_notsup(pv, rq);
}

static inline error_t si446x_check_config(struct si446x_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{
  switch (rq->type)
    {
    case DEV_RFPACKET_RQ_RX_CONT:
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      si446x_rfp_set_state(pv, SI446X_STATE_CONFIG_RXC);
      break;
    default:
      si446x_rfp_set_state(pv, SI446X_STATE_CONFIG);
    }

  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA

  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR)
    {
      switch (rfcfg->mod)
        {
          case DEV_RFPACKET_GFSK:
          case DEV_RFPACKET_FSK:
            {
              const struct dev_rfpacket_rf_cfg_fsk_s * c =
                const_dev_rfpacket_rf_cfg_fsk_s_cast(rfcfg);
              if (c->fairtx.mode == DEV_RFPACKET_NO_FAIRTX)
                return -ENOTSUP;
              break;
            }
          case DEV_RFPACKET_ASK:
            {
              const struct dev_rfpacket_rf_cfg_ask_s * c =
                const_dev_rfpacket_rf_cfg_ask_s_cast(rfcfg);
              if (c->fairtx.mode == DEV_RFPACKET_NO_FAIRTX)
                return -ENOTSUP;
              break;
            }
          default:
            break;
         }

     /* Test if RX is allowed during TX */
      pv->flags &= ~SI446X_FLAGS_RX_ON;
      struct dev_rfpacket_rq_s *rx_cont = dev_rfpacket_rq_head(&pv->rx_cont_queue);
      if (rx_cont && rq->rf_cfg == rx_cont->rf_cfg &&
          rq->pk_cfg == rx_cont->pk_cfg)
        pv->flags |= SI446X_FLAGS_RX_ON;
    }
#endif

  error_t err = 0;

  /* Check packet format configuration */
  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;

  if ((pkcfg != pv->pk_cfg) || pkcfg->cache.dirty)
    {
      pv->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pkcfg;

      if (pkcfg->cache.dirty)
        ((struct dev_rfpacket_pk_cfg_s *)pkcfg)->cache.dirty = 0;

      err = si446x_build_pk_config(pv, rq);

      if (err)
        return err;

      si446x_bytecode_start(pv, &si446x_entry_pkt_config, SI446X_ENTRY_PKT_CONFIG_BCARGS(
        (uintptr_t)&pv->pk_buff, (uintptr_t)si446x_pk_cmd));

      return -EAGAIN;
    }

  /* Check RF configuration */

  struct si446x_cache_entry_s *e = &pv->cache_array[rfcfg->cache.id % SI446X_RF_CONFIG_CACHE_ENTRY];

  if (e->cfg == rfcfg && !rfcfg->cache.dirty)
    {
      if (rfcfg->cache.id == pv->id)
        /** Config is in cache and is applied */
        return 0;

      /** Config is in cache but is not applied */
      si446x_rf_config_done(pv, rfcfg);
      return -EAGAIN;
    }

  /* Update cache entry */

  e->cfg = (struct dev_rfpacket_rf_cfg_s * )rfcfg;
  /* Time byte in us */
  dev_timer_delay_t tb = 8000000/rq->rf_cfg->drate;
  dev_timer_init_sec(pv->timer, &(e->tb), 0, tb, 1000000);

  if (rfcfg->cache.dirty)
    ((struct dev_rfpacket_rf_cfg_s *)rfcfg)->cache.dirty = 0;

  kroutine_init_deferred(&pv->kr, &si446x_config_deferred);
  kroutine_exec(&pv->kr);

  return -EAGAIN;
}

static void si446x_rfp_idle(struct si446x_ctx_s *pv);

static inline void si446x_rfp_process_group(struct si446x_ctx_s *pv, bool_t group)
{
  struct dev_rfpacket_rq_s * rq;

  while (1)
  {
    rq = dev_rfpacket_rq_head(&pv->queue);

    if (!rq || rq->err_group != group)
      break;

    assert(rq->type != DEV_RFPACKET_RQ_RX_CONT &&
           rq->type != DEV_RFPACKET_RQ_RX_TIMEOUT);

    rq->error = -ECANCELED;
    rq->base.drvdata = NULL;

    dev_rfpacket_rq_pop(&pv->queue);
    dev_rfpacket_rq_done(rq);
  }
}

static void si446x_rfp_end_rxc(struct si446x_ctx_s *pv, error_t err)
{
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);

  if (rq)
    rq->error = err;

  switch (pv->state)
    {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case SI446X_STATE_TX_LBT_STOPPING_RXC:
#endif
    case SI446X_STATE_RXC:
    case SI446X_STATE_CONFIG_RXC:
    case SI446X_STATE_CONFIG_RXC_PENDING_STOP:
    case SI446X_STATE_STOPPING_RXC:
    case SI446X_STATE_PAUSE_RXC:
      assert(rq);
      rq->base.drvdata = NULL;
      dev_rfpacket_rq_remove(&pv->rx_cont_queue, rq);
      dev_rfpacket_rq_done(rq);
      return si446x_rfp_idle(pv);
    default:
      UNREACHABLE();
    }
}

static void si446x_rfp_end_rq(struct si446x_ctx_s *pv, error_t err)
{
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->queue);

  assert(rq && rq->type != DEV_RFPACKET_RQ_RX_CONT &&
         rq->type != DEV_RFPACKET_RQ_RX_TIMEOUT);

  rq->error = err;
  rq->base.drvdata = NULL;

  dev_rfpacket_rq_pop(&pv->queue);
  dev_rfpacket_rq_done(rq);

  if (rq->error)
    si446x_rfp_process_group(pv, rq->err_group);

  switch (pv->state)
  {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case SI446X_STATE_TX_LBT_STOPPING_RXC:
      return si446x_rfp_end_rxc(pv, 0);
#endif
    default:
      return si446x_rfp_idle(pv);
  }
}

static inline void si446x_start_rx(struct si446x_ctx_s *pv, struct dev_rfpacket_rq_s *rq)
{
  assert(pv->state == SI446X_STATE_READY);
  assert(rq);

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  pv->rq = rq;

  struct dev_rfpacket_rf_cfg_s * cfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
  uint32_t freq = cfg->frequency + rq->channel * cfg->chan_spacing;

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_RX:
      logk_trace("R");

      pv->deadline = rq->deadline ? rq->deadline : t;
      pv->timeout = pv->deadline + rq->lifetime;
      pv->frequency = freq;

      si446x_rfp_set_state(pv, SI446X_STATE_RX);

      si446x_bytecode_start(pv, &si446x_entry_rx,
        SI446X_ENTRY_RX_BCARGS(rq->channel));
      break;

    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      /* FIXME check if RQ timeout has already elapsed,
         FIXME do this before computing configuration */
      logk_trace("RT");
      pv->flags &= ~SI446X_FLAGS_RXC_INFINITE;
      pv->rxc_timeout = rq->deadline;
      goto rxc;

    case DEV_RFPACKET_RQ_RX_CONT:
      logk_trace("RC");
      pv->flags |= SI446X_FLAGS_RXC_INFINITE;

    rxc:
      if (pv->frequency != freq)
        {
          pv->rssi = SET_RSSI(SI446X_RSSI_AVERAGE_DEFAULT) << 8;
          pv->frequency = freq;
        }

      pv->flags |= SI446X_FLAGS_RX_CONTINOUS;

      si446x_rfp_set_state(pv, SI446X_STATE_RXC);

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_WUT
      if (pv->pk_buff.clk)
	{
          si446x_bytecode_start(pv, &si446x_entry_rx_wake_up,
                                SI446X_ENTRY_RX_WAKE_UP_BCARGS(rq->channel));
	}
      else
#endif
        {
          si446x_bytecode_start(pv, &si446x_entry_rx_cont,
                                SI446X_ENTRY_RX_CONT_BCARGS(rq->channel));
        }
      break;

    default:
      UNREACHABLE();
  }
}

static inline void si446x_retry_rx(struct si446x_ctx_s *pv)
{
  assert(pv->state == SI446X_STATE_RX);

  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->queue);

  assert(rq && rq->type == DEV_RFPACKET_RQ_RX);
  pv->rq = rq;

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  if (t >= pv->timeout)
    return si446x_rfp_end_rq(pv, 0);

  si446x_bytecode_start(pv, &si446x_entry_rx,
    SI446X_ENTRY_RX_BCARGS(rq->channel));
}

static inline void si446x_start_tx(struct si446x_ctx_s *pv)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  assert(rq && (rq->tx_size < SI446X_MAX_PACKET_SIZE));

  pv->rq = rq;
  pv->size = rq->tx_size;
  pv->buffer = (uint8_t *)rq->tx_buf;

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  assert(pv->state == SI446X_STATE_READY);

  pv->deadline = rq->deadline ? rq->deadline : t;
  uint8_t pwr;

  switch (rq->type)
  {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case DEV_RFPACKET_RQ_TX_FAIR:

      pv->timeout = pv->deadline + rq->lifetime;
      logk_trace("TF");

      if (t >= pv->timeout)
      /* Timeout date is already reached */
        return si446x_rfp_end_rq(pv, -ETIMEDOUT);

      pwr = si446X_get_pwr_lvl(pv, rq->tx_pwr);

      // Init lbt state
      pv->lbt_state = SI446X_LBT_STATE_FREE;
      // Init lbt random waiting value
      pv->lbt_rand_time = si446x_calc_lbt_rand_rime(pv->bt, t);

      si446x_rfp_set_state(pv, SI446X_STATE_TX_LBT);

      si446x_bytecode_start(pv, &si446x_entry_tx_cca,
              SI446X_ENTRY_TX_CCA_BCARGS(pwr, rq->channel));
      break;
#endif
    case DEV_RFPACKET_RQ_TX:

      logk_trace("T");

      pwr = si446X_get_pwr_lvl(pv, rq->tx_pwr);

      si446x_rfp_set_state(pv, SI446X_STATE_TX);

      si446x_bytecode_start(pv, &si446x_entry_tx,
              SI446X_ENTRY_TX_BCARGS(pwr, rq->channel));
      break;

    default:
      UNREACHABLE();
  }
}

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA

/* TX with LBT has been interrupted by a RX packet and TX fifo must be refilled */

static inline void si446x_retry_tx(struct si446x_ctx_s *pv, bool_t refill)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  assert(rq && rq->type == DEV_RFPACKET_RQ_TX_FAIR);

  pv->rq = rq;
  pv->size = rq->tx_size;
  pv->buffer = (uint8_t *)rq->tx_buf;

  switch (pv->state)
  {
    case SI446X_STATE_TX_LBT:
    case SI446X_STATE_TX_LBT_STOPPING_RXC:
    /* TX has been interrupted by a RX packet */
      assert(rq->type == DEV_RFPACKET_RQ_TX_FAIR);

      if (refill)
        {
          dev_timer_value_t t;
          DEVICE_OP(pv->timer, get_value, &t, 0);
          /* TX has been interrupted by an error */
          if (t >= pv->timeout)
          /* Timeout date is already reached */
            return si446x_rfp_end_rq(pv, -ETIMEDOUT);

          // Init lbt state
          pv->lbt_state = SI446X_LBT_STATE_FREE;

          si446x_bytecode_start(pv, &si446x_entry_tx_cca,
            SI446X_ENTRY_TX_CCA_BCARGS(0, rq->channel));
        }
      else
        {
          if (pv->size < SI446X_FIFO_SIZE)
            pv->size = 0;
          else
            {
              pv->size -= SI446X_FIFO_SIZE - 1;
              pv->buffer += SI446X_FIFO_SIZE - 1;
            }

          // Init lbt state
          pv->lbt_state = SI446X_LBT_STATE_FREE;

          si446x_bytecode_start(pv, &si446x_entry_retry_tx_cca,
                  SI446X_ENTRY_RETRY_TX_CCA_BCARGS(rq->channel));
        }
      break;

    default:
      UNREACHABLE();
  }
}

#endif

/* Transceiver is idle */

static void si446x_rfp_idle(struct si446x_ctx_s *pv)
{
  si446x_rfp_set_state(pv, SI446X_STATE_READY);

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

  if (!rq)
    rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);

  if (!rq)
    {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
     /* delayed clock disable */
      device_sleep_schedule(pv->dev);
#endif
      return;
    }

  pv->rq = rq;

  logk_trace("idle %d", rq->type);

  /* Check transceiver configuration */
  switch (si446x_check_config(pv, rq))
    {
    case -EAGAIN:
      /** Configuration is being applied */
      return;
    case -ENOTSUP:
      /** Unsupported configuration */
      return si446x_rf_config_notsup(pv, rq);
    default:
      /* Configuration is already applied */
      si446x_rfp_set_state(pv, SI446X_STATE_READY);
      break;
    }

  switch (rq->type)
  {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case DEV_RFPACKET_RQ_TX_FAIR:
#endif
    case DEV_RFPACKET_RQ_TX:
      pv->txcca_timestamp = 0;
      return si446x_start_tx(pv);
    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      return si446x_start_rx(pv, rq);
    default:
      return si446x_rfp_end_rq(pv, -ENOTSUP);
  }
}

static void si446x_cancel_rxc(struct si446x_ctx_s *pv)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  pv->flags &= ~SI446X_FLAGS_RX_CONTINOUS;

  /* Wakeup periodic rssi sampling */
  dev_spi_bytecode_wakeup(&pv->spi, srq);
}

static DEV_RFPACKET_REQUEST(si446x_rfp_request)
{
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  va_list vl;
  va_start(vl, accessor);

  while(1)
  {
    struct dev_rfpacket_rq_s *rq = va_arg(vl, struct dev_rfpacket_rq_s *);

    if (rq == NULL)
      break;

    dev_timer_value_t ts;
    DEVICE_OP(pv->timer, get_value, &ts, 0);
    logk_trace("[%d] req %d %d %d", (uint32_t)ts, rq->type, rq->tx_size, pv->state);

    rq->error = 0;

    switch (rq->type)
      {
      case DEV_RFPACKET_RQ_RX_TIMEOUT:
        if (rq->lifetime != 0)
          {
            dev_timer_value_t t;
            DEVICE_OP(pv->timer, get_value, &t, 0);
            rq->deadline = t + rq->lifetime;
          }
      case DEV_RFPACKET_RQ_RX_CONT:
        rq->base.drvdata = pv;
        if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
          rq->deadline = -1;
        switch (pv->state)
        {
          case SI446X_STATE_READY:
            assert(dev_rq_queue_isempty(&pv->rx_cont_queue));
            dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
            si446x_rfp_idle(pv);
            break;
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
          case SI446X_STATE_SLEEP:
            assert(dev_rq_queue_isempty(&pv->rx_cont_queue));
            si446x_bytecode_start(pv, &si446x_entry_ready, 0, 0);
            si446x_rfp_set_state(pv, SI446X_STATE_AWAKING);
          case SI446X_STATE_ENTER_SLEEP:
#endif
          case SI446X_STATE_AWAKING:
          case SI446X_STATE_CONFIG:
          case SI446X_STATE_RX:
          case SI446X_STATE_TX:
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
          case SI446X_STATE_TX_LBT:
#endif
          case SI446X_STATE_STOPPING_RXC:
          case SI446X_STATE_PAUSE_RXC:
          case SI446X_STATE_CONFIG_RXC_PENDING_STOP:
          case SI446X_STATE_CONFIG_RXC:
          case SI446X_STATE_TX_LBT_STOPPING_RXC:
            dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
            break;

          case SI446X_STATE_RXC:
            dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
            /* FIXME share the radio when using the same configuration */
            if (rq == dev_rfpacket_rq_head(&pv->rx_cont_queue))
              {
                si446x_rfp_set_state(pv, SI446X_STATE_PAUSE_RXC);
                si446x_cancel_rxc(pv);
              }
            break;

          case SI446X_STATE_INITIALISING:
            UNREACHABLE();
        }
        break;

      case DEV_RFPACKET_RQ_RX:
      case DEV_RFPACKET_RQ_TX_FAIR:
      case DEV_RFPACKET_RQ_TX: {
        bool_t empty = dev_rq_queue_isempty(&pv->queue);
        dev_rfpacket_rq_pushback(&pv->queue, rq);
        rq->base.drvdata = pv;

        if (!empty)
          break;

        switch (pv->state)
            {
              case SI446X_STATE_RXC:
                assert(!dev_rq_queue_isempty(&pv->rx_cont_queue));
                assert(rq->deadline == 0);
                si446x_rfp_set_state(pv, SI446X_STATE_PAUSE_RXC);
                si446x_cancel_rxc(pv);
                break;
              case SI446X_STATE_READY:
                si446x_rfp_idle(pv);
                break;
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
              case SI446X_STATE_SLEEP:
                si446x_bytecode_start(pv, &si446x_entry_ready, 0, 0);
                si446x_rfp_set_state(pv, SI446X_STATE_AWAKING);
              case SI446X_STATE_ENTER_SLEEP:
#endif
              default:
                break;

              case SI446X_STATE_INITIALISING:
                UNREACHABLE();
            }
      }
      }
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_RFPACKET_CANCEL(si446x_rfp_cancel)
{
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;

  logk_trace("cancel %d", pv->state);

  error_t err = -EBUSY;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  struct dev_rfpacket_rq_s *hrq = dev_rfpacket_rq_head(&pv->queue);
  struct dev_rfpacket_rq_s *hrqcont = dev_rfpacket_rq_head(&pv->rx_cont_queue);

  if (rq == hrqcont)
    {
      switch (pv->state)
      {
        case SI446X_STATE_CONFIG_RXC:
          si446x_rfp_set_state(pv, SI446X_STATE_CONFIG_RXC_PENDING_STOP);
          break;
        case SI446X_STATE_RXC:
          si446x_cancel_rxc(pv);
        case SI446X_STATE_PAUSE_RXC:
          si446x_rfp_set_state(pv, SI446X_STATE_STOPPING_RXC);
        case SI446X_STATE_STOPPING_RXC:
          break;
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
        case SI446X_STATE_TX_LBT:
          si446x_rfp_set_state(pv, SI446X_STATE_TX_LBT_STOPPING_RXC);
          break;
        case SI446X_STATE_CONFIG_RXC_PENDING_STOP:
        case SI446X_STATE_TX_LBT_STOPPING_RXC:
          break;
#endif
        default:
          err = 0;
          rq->base.drvdata = NULL;
          dev_rfpacket_rq_remove(&pv->rx_cont_queue, rq);
          break;
      }
    }
  else if ((rq->base.drvdata == pv) && (rq != hrq))
  /* Request is in queue and is not being processed */
    {
      err = 0;
      rq->base.drvdata = NULL;

      switch (rq->type)
        {
        case DEV_RFPACKET_RQ_TX:
        case DEV_RFPACKET_RQ_TX_FAIR:
        case DEV_RFPACKET_RQ_RX:
          dev_rfpacket_rq_remove(&pv->queue, rq);
          break;

        case DEV_RFPACKET_RQ_RX_CONT:
        case DEV_RFPACKET_RQ_RX_TIMEOUT:
          dev_rfpacket_rq_remove(&pv->rx_cont_queue, rq);
          break;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static void si446x_clean(struct device_s *dev)
{
  struct si446x_ctx_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, pv->src_ep, SI446X_IRQ_SRC_COUNT);

  device_stop(&pv->timer->base);

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);

  mem_free(pv);
}

BC_CCALL_FUNCTION(si446x_enable_cts_irq) {
  struct si446x_ctx_s *pv = (struct si446x_ctx_s *)bc_get_reg(ctx, R_CTX_PV);
  struct dev_irq_src_s *p_irq_src = &pv->src_ep[SI446X_IRQ_SRC_CTS];
  // TODO Prevent race condition by locking irq till the driver is in yieldc ?
  // Enable cts irq
  device_irq_src_enable(p_irq_src);
  return 0;
}

BC_CCALL_FUNCTION(si446x_disable_cts_irq) {
  struct si446x_ctx_s *pv = (struct si446x_ctx_s *)bc_get_reg(ctx, R_CTX_PV);
  struct dev_irq_src_s *p_irq_src = &pv->src_ep[SI446X_IRQ_SRC_CTS];
  // Disable cts irq
  device_irq_src_disable(p_irq_src);
  return 0;
}

BC_CCALL_FUNCTION(si446x_alloc)
{
  struct si446x_ctx_s *pv = (struct si446x_ctx_s *)bc_get_reg(ctx, R_CTX_PV);
  struct dev_rfpacket_rq_s *rq;
  uintptr_t p = 0;

  logk_trace("RX alloc %d", pv->size);

  LOCK_SPIN_IRQ(&pv->dev->lock);

  switch (pv->state)
  {
    case SI446X_STATE_RX:
      rq = dev_rfpacket_rq_head(&pv->queue);
      break;
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case SI446X_STATE_TX_LBT:
    case SI446X_STATE_TX_LBT_STOPPING_RXC:
#endif
    case SI446X_STATE_PAUSE_RXC:
    case SI446X_STATE_RXC:
      rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);
      break;
    case SI446X_STATE_STOPPING_RXC:
      /* RX continous is being cancelled */
      rq = NULL;
      break;
    default:
      UNREACHABLE();
  }

  if (rq == NULL)
    goto error;

  struct dev_rfpacket_rx_s *rx = rq->rx_alloc(rq, pv->size);

  if (rx == NULL)
    goto error;

  rx->channel = rq->channel;
  rx->timestamp =
    rq->anchor == DEV_RFPACKET_TIMESTAMP_START
    ? pv->cache_array[pv->id].tb : 0;

  pv->rxrq = rx;
  pv->buffer = (uint8_t*)rx->buf;

  assert(rx->size == pv->size);

  p = (uintptr_t)pv->buffer;

  logk_trace("%d", pv->size);

error:

  LOCK_RELEASE_IRQ(&pv->dev->lock);

  return p;
}

static inline void si446x_rfp_end_rxrq(struct si446x_ctx_s *pv)
{
  struct dev_rfpacket_rx_s *rx = pv->rxrq;

  if (rx == NULL)
    return;

  error_t err = 0;

  if (bit_get(pv->bc_status, STATUS_OTHER_ERR))
    err = -EIO;
  else if (bit_get(pv->bc_status, STATUS_CRC_ERROR))
    err = -EBADDATA;

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_STATISTICS
  pv->stats.rx_count++;
  if (err)
    pv->stats.rx_err_count++;
#endif

  rx->error = err;

  if (!err)
    {
      rx->carrier = GET_RSSI(pv->carrier) << 3;
      rx->rssi = GET_RSSI(pv->rssi >> 8) << 3;
      rx->frequency = pv->frequency + (((int64_t)pv->synth_ratio * pv->afc_offset) >> 23);
      rx->timestamp = pv->timestamp - ((dev_timer_delay_t)/* tb */rx->timestamp * rx->size);
    }
  else
    {
      rx->size = 0;
    }

  kroutine_exec(&rx->kr);
  pv->rxrq = NULL;
}

/* Transceiver is idle when this function is called */
static inline void si446x_rfp_error(struct si446x_ctx_s *pv)
{
  logk_trace("-EIO error %d", pv->state);
  /* Terminate allocated rx request */
  si446x_rfp_end_rxrq(pv);

  switch (pv->state)
  {
    case SI446X_STATE_TX:
      return si446x_rfp_end_rq(pv, -EIO);
    case SI446X_STATE_RX:
      return si446x_retry_rx(pv);
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case SI446X_STATE_TX_LBT:
    case SI446X_STATE_TX_LBT_STOPPING_RXC:
      si446x_retry_tx(pv, 1);
      break;
#endif
    case SI446X_STATE_STOPPING_RXC:
    case SI446X_STATE_PAUSE_RXC:
      return si446x_rfp_end_rxc(pv, 0);
    case SI446X_STATE_RXC:
      return si446x_rfp_idle(pv);
    default:
      UNREACHABLE();
  }
}

static inline void si446x_rfp_end_txrq(struct si446x_ctx_s *pv)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_STATISTICS
  pv->stats.tx_count++;
#endif

  rq->tx_timestamp = pv->timestamp;
  if (rq->anchor == DEV_RFPACKET_TIMESTAMP_START)
    rq->tx_timestamp -= rq->tx_size * pv->cache_array[pv->id].tb;

  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR) {
    rq->tx_lbt_td = pv->timestamp - (rq->tx_size * pv->cache_array[pv->id].tb) - pv->txcca_timestamp;
  }
}

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
/* Transceiver is sleeping when this function is called */
static inline void si446x_check_wakeup(struct si446x_ctx_s *pv)
{
  bool_t empty = dev_rq_queue_isempty(&pv->queue)
    && dev_rq_queue_isempty(&pv->rx_cont_queue);

  if (!empty)
    {
      si446x_bytecode_start(pv, &si446x_entry_ready, 0, 0);
      si446x_rfp_set_state(pv, SI446X_STATE_AWAKING);
    }
  else
    {
      logk_trace("sleeping");
    }
}
#endif

static KROUTINE_EXEC(si446x_spi_rq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->pvdata;
  struct si446x_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  pv->bc_status = bc_get_reg(&srq->vm, STATUS);

  dev_timer_value_t ts;
  DEVICE_OP(pv->timer, get_value, &ts, 0);
  logk_trace("[%d] bdone %d 0x%x", (uint32_t)ts, pv->state, pv->bc_status);

  if (pv->state != SI446X_STATE_INITIALISING)
    assert(!srq->error);

  if (pv->icount != (pv->bc_status & STATUS_IRQ_MSK))
    {
      /* FIXME bytecode should be written so that is does not leave
         when an irq is expected */
      si446x_bytecode_start(pv, &si446x_entry_irq, 0, 0);
      goto end;
    }

  if (pv->bc_status & bit(STATUS_OTHER_ERR))
    {
      si446x_rfp_error(pv);
      goto end;
    }

  switch (pv->state)
  {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
    case SI446X_STATE_ENTER_SLEEP:
      si446x_rfp_set_state(pv, SI446X_STATE_SLEEP);
      si446x_check_wakeup(pv);
      break;
    case SI446X_STATE_AWAKING:
      logk_trace("awaken");
      si446x_rfp_idle(pv);
      break;
#endif
    case SI446X_STATE_INITIALISING:
      if (srq->error)
        {
          si446x_clean(dev);
          device_async_init_done(dev, -EIO);
          goto end;
        }
      device_async_init_done(dev, 0);
    case SI446X_STATE_CONFIG_RXC:
    case SI446X_STATE_CONFIG:
      si446x_rfp_idle(pv);
      break;
    case SI446X_STATE_RX:
      if (pv->bc_status & bit(STATUS_RX_TIMEOUT))
        {
          si446x_rfp_end_rxrq(pv);
          si446x_rfp_end_rq(pv, 0);
          break;
        }
      if (pv->bc_status & STATUS_RX_END_MSK)
        {
          si446x_rfp_end_rxrq(pv);
          si446x_retry_rx(pv);
          break;
        }
      UNREACHABLE();

    case SI446X_STATE_PAUSE_RXC:
    case SI446X_STATE_RXC:
      if (pv->bc_status & bit(STATUS_JAMMING))
        {
          logk_trace("Jamming %d %d",
                     GET_RSSI((int16_t)(pv->rssi >> 8)),
                     GET_RSSI((int16_t)(pv->jam_rssi >> 8)));
          assert(pv->rxrq == NULL);
          pv->rssi = SET_RSSI(SI446X_RSSI_AVERAGE_DEFAULT) << 8;
          si446x_rfp_end_rxc(pv, -EAGAIN);
          break;
        }
      if (pv->bc_status & bit(STATUS_RX_TIMEOUT))
        {
          si446x_rfp_end_rxrq(pv);
          si446x_rfp_end_rxc(pv, 0);
          break;
        }
      if (pv->bc_status & STATUS_RX_END_MSK)
        {
          si446x_rfp_end_rxrq(pv);
          si446x_rfp_idle(pv);
          break;
        }
      si446x_rfp_idle(pv);
      break;

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CCA
    case SI446X_STATE_TX_LBT:
    case SI446X_STATE_TX_LBT_STOPPING_RXC:
      if (pv->bc_status & STATUS_RX_END_MSK)
        {
          si446x_rfp_end_rxrq(pv);
          si446x_retry_tx(pv, 0);
          break;
        }
      if (pv->bc_status & bit(STATUS_TX_TIMEOUT))
        {
# ifdef CONFIG_DRIVER_RFPACKET_SI446X_STATISTICS
          pv->stats.tx_err_count++;
# endif
          si446x_rfp_end_rq(pv, -ETIMEDOUT);
          break;
        }
#endif
    case SI446X_STATE_TX:
      if (pv->bc_status & STATUS_TX_END_MSK)
        {
          /* Packet has been transmitted */
          si446x_rfp_end_txrq(pv);
          si446x_rfp_end_rq(pv, 0);
          break;
        }
      UNREACHABLE();

    case SI446X_STATE_STOPPING_RXC:
      if (pv->bc_status & STATUS_RX_END_MSK)
        si446x_rfp_end_rxrq(pv);
    case SI446X_STATE_CONFIG_RXC_PENDING_STOP:
      si446x_rfp_end_rxc(pv, 0);
      break;

    case SI446X_STATE_SLEEP:
    default:
      UNREACHABLE();
  }

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(si446x_irq_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct si446x_ctx_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  lock_spin(&dev->lock);

  // Check irq endpoint
  if (ep == &pv->src_ep[SI446X_IRQ_SRC_NIRQ]) {
    // Increment irq count
    pv->icount++;
    // Get timer value
    DEVICE_OP(pv->timer, get_value, &pv->timestamp, 0);
  }
  // Wakeup any waiting instruction
  dev_spi_bytecode_wakeup(&pv->spi, srq);

  lock_release(&dev->lock);
}

static DEV_USE(si446x_use)
{
  switch (op)
    {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
    case DEV_USE_SLEEP: {
      struct device_s *dev = param;
      struct si446x_ctx_s *pv = dev->drv_pv;

      switch (pv->state)
      {
        case SI446X_STATE_READY:
          logk_trace("sleep");
          si446x_rfp_set_state(pv, SI446X_STATE_ENTER_SLEEP);
          si446x_bytecode_start(pv, &si446x_entry_sleep, 0, 0);
          break;

        default:
          break;
      }
    }
#endif
    default:
      return dev_use_generic(param, op);
    }
}

static DEV_RFPACKET_STATS(si446x_rfp_stats)
{
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_STATISTICS
  struct device_s *dev = accessor->dev;
  struct si446x_ctx_s *pv = dev->drv_pv;
  memcpy(stats, &pv->stats, sizeof(*stats));
  return 0;
#else
  return -ENOTSUP;
#endif
}

static DEV_INIT(si446x_init);
static DEV_CLEANUP(si446x_cleanup);

DRIVER_DECLARE(si446x_drv, 0, "SiLabs si446x transceiver", si446x,
               DRIVER_RFPACKET_METHODS(si446x_rfp),
               DRIVER_TIMER_METHODS(si446x_timer));

DRIVER_REGISTER(si446x_drv);

static DEV_INIT(si446x_init)
{
  struct si446x_ctx_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;
  struct device_gpio_s *gpio;

  static const struct dev_spi_ctrl_config_s spi_cfg = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .cs_pol   = DEV_SPI_ACTIVE_LOW,
    .bit_rate1k = CONFIG_DRIVER_RFPACKET_SI446X_SPI_BITRATE >> 10,
    .word_width = 8,
  };

  if (dev_drv_spi_bytecode_init(dev, srq, &si446x_bytecode, &spi_cfg, &pv->spi, &gpio, &pv->timer))
    goto err_mem;

  /* Base 500 us time */
  dev_timer_init_sec(pv->timer, &pv->bt, 0, SI446X_BASE_TIME, 1000000);

  /* Start timer */
  if (device_start(&pv->timer->base))
    goto err_srq;

  srq->pvdata = dev;
  pv->dev = dev;

  /* init GPIO stuff */

  static const gpio_width_t pin_wmap[4] = {1, 1, 1, 1};

  if (device_gpio_setup(gpio, dev, ">sdn:1 <nirq:1 <cts:1"
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_TCXO
                        " >xoen?:1"
#endif
                        , pv->pin_map, NULL))
    goto err_timer;

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

  dev_rq_queue_init(&pv->queue);
  dev_rq_queue_init(&pv->rx_cont_queue);

  dev_spi_ctrl_rq_init(&srq->base, &si446x_spi_rq_done);

  /* Init irq*/
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_CTS_IRQ
  device_irq_source_init(dev, pv->src_ep, SI446X_IRQ_SRC_COUNT, &si446x_irq_source_process);

  if (device_irq_source_link(dev, pv->src_ep, SI446X_IRQ_SRC_COUNT, 0x3)) {
    goto err_timer;
  }
  device_irq_src_disable(&pv->src_ep[SI446X_IRQ_SRC_CTS]);
#else
  device_irq_source_init(dev, pv->src_ep, 1, &si446x_irq_source_process);

  if (device_irq_source_link(dev, pv->src_ep, 1, 0x3)) {
    goto err_timer;
  }
#endif

  pv->pwr = 0xFFFF;

  struct dev_freq_s osc;
  if (!device_get_res_freq(dev, &osc, 0))
    pv->osc_ppb = dev_freq_acc_ppb(&osc);
  else
    pv->osc_ppb = 20000;

  if (device_get_param_blob(dev, "rftune", 0, (const void**)&pv->rftune))
    pv->rftune = NULL;

  /* Start initialisation */
  si446x_rfp_set_state(pv, SI446X_STATE_INITIALISING);

  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  si446x_bytecode_start(pv, &si446x_entry_reset,
                        SI446X_ENTRY_RESET_BCARGS(pv->bt, si446x_config));

  return -EAGAIN;

 err_timer:
  device_stop(&pv->timer->base);
 err_srq:
  dev_drv_spi_bytecode_cleanup(&pv->spi, srq);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(si446x_cleanup)
{
  struct si446x_ctx_s *pv = dev->drv_pv;

  switch (pv->state)
    {
    case SI446X_STATE_READY:
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_SLEEP
    case SI446X_STATE_SLEEP:
#endif
      assert(dev_rq_queue_isempty(&pv->queue));
      break;
    default:
      return -EBUSY;
    }

  si446x_clean(dev);
  return 0;
}

const uint8_t si446x_config[] = {
#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV == 0x22
  0x08, 0x04, 0x21, 0x71, 0x4b, 0x00, 0x00, 0xba, 0x9e,
  0x08, 0x05, 0x48, 0x23, 0x2e, 0x2b, 0x90, 0xb1, 0x4e,
  0x08, 0xea, 0x3f, 0xb9, 0xe8, 0x8b, 0xa9, 0xca, 0xd6,
  0x08, 0x05, 0xd2, 0xe5, 0xbe, 0xd1, 0x27, 0x55, 0x82,
  0x08, 0xe5, 0x56, 0x2a, 0x3b, 0x76, 0x76, 0x96, 0x48,
  0x08, 0x05, 0x8e, 0x26, 0xd8, 0x5d, 0x01, 0xa7, 0x88,
  0x08, 0xe2, 0x89, 0xcc, 0x63, 0x79, 0x95, 0x00, 0x4b,
  0x08, 0x05, 0xe0, 0x75, 0xcd, 0xa4, 0xb9, 0x46, 0xbc,
  0x08, 0xea, 0xd3, 0x37, 0xd2, 0x9a, 0x89, 0x82, 0xea,
  0x08, 0x05, 0x0c, 0xae, 0x4c, 0xf5, 0xf6, 0x3c, 0xb3,
  0x08, 0xe9, 0xa7, 0x70, 0xdf, 0xf1, 0x14, 0x4f, 0x04,
  0x08, 0x05, 0xfe, 0x5b, 0xdf, 0x47, 0x0a, 0x7c, 0x5b,
  0x08, 0xe2, 0xfb, 0x3e, 0x21, 0xa2, 0x1b, 0xaa, 0x93,
  0x08, 0x05, 0xbf, 0xfd, 0xab, 0x69, 0x6c, 0xa8, 0x5a,
  0x08, 0xe2, 0x66, 0xb7, 0x2e, 0x2c, 0x45, 0x2d, 0xfb,
  0x08, 0x05, 0x0d, 0x55, 0xbd, 0xc2, 0x37, 0x00, 0x72,
  0x08, 0xe2, 0xff, 0x57, 0x4d, 0x7c, 0x6c, 0x00, 0x2c,
  0x08, 0x05, 0x9e, 0xf2, 0x46, 0xfd, 0xd3, 0x16, 0x1b,
  0x08, 0xea, 0x16, 0x7f, 0x67, 0x4d, 0xe5, 0xe2, 0xc8,
  0x08, 0x05, 0x37, 0x33, 0x1c, 0xfa, 0xbb, 0xee, 0xef,
  0x08, 0xea, 0x00, 0x5f, 0xbe, 0xa4, 0xfc, 0xbf, 0xc1,
  0x08, 0x05, 0x95, 0x12, 0x2f, 0x0a, 0xcf, 0x55, 0x8c,
  0x08, 0xe7, 0x70, 0xc2, 0xd4, 0xf0, 0x81, 0x95, 0xc8,
  0x08, 0xe7, 0x72, 0x00, 0xf9, 0x8d, 0x15, 0x01, 0xa3,
  0x08, 0xe7, 0x18, 0xe5, 0x6c, 0x51, 0x1f, 0x86, 0x9f,
  0x08, 0xe7, 0xdd, 0x37, 0x59, 0x4b, 0xad, 0xb0, 0x9c,
  0x08, 0xe7, 0xc8, 0xe8, 0x84, 0xcd, 0x55, 0x41, 0x83,
  0x08, 0xef, 0x4f, 0x8e, 0x38, 0xcb, 0x37, 0x02, 0x87,
  0x08, 0xe7, 0xf5, 0x00, 0x88, 0x4c, 0x09, 0x65, 0xce,
  0x08, 0xef, 0xdd, 0xbc, 0x65, 0x62, 0xac, 0x75, 0x62,
  0x08, 0xe7, 0xc0, 0xf1, 0x5d, 0x98, 0xb0, 0xdd, 0x43,
  0x08, 0xe7, 0x19, 0xb4, 0xf8, 0x9f, 0x6d, 0x8c, 0xcb,
  0x08, 0xe1, 0xde, 0x63, 0xc2, 0x32, 0xc6, 0xe4, 0x2f,
  0x08, 0x05, 0xf4, 0x33, 0xb7, 0x2e, 0x72, 0x9a, 0xf9,
  0x08, 0xe7, 0x65, 0xd9, 0x38, 0xb8, 0xfe, 0x31, 0x16,
  0x08, 0xe7, 0xf3, 0x06, 0x2d, 0xf5, 0xfe, 0x0c, 0x38,
  0x08, 0xe7, 0x70, 0x4f, 0xe7, 0x49, 0xb4, 0x58, 0x39,
  0x08, 0xef, 0xf1, 0x46, 0xa9, 0x23, 0x38, 0x64, 0xc0,
  0x08, 0xe7, 0x09, 0x4e, 0x04, 0xd3, 0x46, 0x15, 0x02,
  0x08, 0xef, 0x8d, 0xc7, 0x20, 0xc3, 0x90, 0x87, 0x4d,
  0x08, 0xef, 0x00, 0xab, 0x7f, 0x27, 0x02, 0xc6, 0xa0,
  0x08, 0xe7, 0x23, 0xa6, 0xa6, 0xa4, 0x27, 0x11, 0x7d,
  0x08, 0xef, 0xb3, 0xf1, 0x9e, 0x6a, 0xb3, 0x19, 0xaf,
  0x08, 0xe7, 0xab, 0xf5, 0x15, 0x78, 0x5e, 0x48, 0xf8,
  0x08, 0xef, 0x5b, 0xb1, 0x2e, 0xaf, 0x2a, 0xff, 0x16,
  0x08, 0xe7, 0x30, 0x62, 0x5c, 0x82, 0x7a, 0x3f, 0x83,
  0x08, 0xef, 0x91, 0xa7, 0xd3, 0x1b, 0x64, 0x85, 0xbe,
  0x08, 0xe7, 0x4d, 0x81, 0x94, 0xe4, 0xaa, 0xe8, 0xdb,
  0x08, 0xef, 0xa0, 0xcc, 0x4a, 0x23, 0xa5, 0x7e, 0x36,
  0x08, 0xef, 0x0c, 0x72, 0x4c, 0xfb, 0x26, 0x5a, 0xec,
  0x08, 0xef, 0x0e, 0x42, 0xfa, 0xaf, 0x49, 0xa0, 0xa8,
  0x08, 0xe7, 0x6d, 0x12, 0xdf, 0x2b, 0x0c, 0x61, 0x58,
  0x08, 0xea, 0xb6, 0x9b, 0xde, 0x81, 0xb9, 0xff, 0xff,
  0x08, 0x05, 0x04, 0xeb, 0xd8, 0x12, 0xd6, 0x8d, 0xe0,
  0x08, 0xec, 0x29, 0x66, 0x4b, 0xde, 0xb7, 0xde, 0x36,
  0x08, 0x05, 0x0d, 0x28, 0xb9, 0x0a, 0x89, 0x31, 0x1a,
#elif CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV < 0x22
  0x08, 0x04, 0x21, 0x2b, 0x90, 0x00, 0x00, 0xb7, 0x8b,
  0x08, 0x05, 0x16, 0xa6, 0xa6, 0x0f, 0xc5, 0xf5, 0xf3,
  0x08, 0xe6, 0xaa, 0x23, 0xef, 0xb5, 0x26, 0x3d, 0xf9,
  0x08, 0x05, 0x4a, 0x8a, 0x06, 0xbd, 0xbd, 0x36, 0x94,
  0x08, 0xe2, 0x1b, 0xcb, 0xdf, 0xf7, 0xaf, 0x47, 0x38,
  0x08, 0x05, 0xb2, 0xf0, 0x2c, 0x9a, 0x47, 0x27, 0x46,
  0x08, 0xe2, 0x5d, 0x66, 0x3a, 0x7c, 0xa6, 0x35, 0x4d,
  0x08, 0x05, 0x20, 0xd5, 0x48, 0x59, 0x06, 0x60, 0xa2,
  0x08, 0xea, 0x75, 0xd1, 0xdf, 0x7f, 0xbd, 0xf7, 0xcf,
  0x08, 0x05, 0xa5, 0x5e, 0xbc, 0x7c, 0x3c, 0x84, 0x4b,
  0x08, 0xea, 0xdd, 0xbf, 0x32, 0x52, 0xbf, 0x1b, 0x68,
  0x08, 0x05, 0x72, 0xb4, 0x53, 0x79, 0x7d, 0xb7, 0x21,
  0x08, 0xef, 0xdc, 0x15, 0xb1, 0xb3, 0xf9, 0xd0, 0x01,
  0x08, 0xe7, 0x06, 0xee, 0xd0, 0xc0, 0x11, 0x9f, 0xfa,
  0x08, 0xef, 0x52, 0xf3, 0x04, 0x6d, 0xcb, 0x50, 0x6a,
  0x08, 0xe9, 0xc8, 0x1d, 0x90, 0x2f, 0x53, 0x60, 0x8d,
  0x08, 0x05, 0x9e, 0x37, 0xf2, 0xc0, 0xd5, 0xb8, 0x6b,
  0x08, 0xe2, 0x96, 0x42, 0xa6, 0xf3, 0xdf, 0xda, 0xb5,
  0x08, 0x05, 0x32, 0xc4, 0x11, 0x8a, 0xb8, 0xd0, 0xeb,
  0x08, 0xe7, 0x5d, 0xe3, 0x51, 0x61, 0x4a, 0xf6, 0x10,
  0x08, 0xe4, 0x94, 0x49, 0x4c, 0x30, 0x39, 0x17, 0x58,
  0x08, 0x05, 0xd2, 0x8b, 0xe8, 0xdf, 0x50, 0x71, 0x71,
  0x08, 0xe9, 0xa8, 0x66, 0x8d, 0xcd, 0xfc, 0x1c, 0x8a,
  0x08, 0x05, 0xdf, 0xb0, 0x7d, 0x06, 0x66, 0x96, 0x98,
  0x08, 0xe2, 0xbc, 0xd7, 0x00, 0x7e, 0xe5, 0xfd, 0xdc,
  0x08, 0x05, 0xdb, 0xdc, 0xda, 0x21, 0x5f, 0xcd, 0x36,
  0x08, 0xe4, 0x04, 0xde, 0x64, 0xda, 0xdf, 0xe0, 0xf4,
  0x08, 0x05, 0xfd, 0x1d, 0x90, 0xe8, 0x17, 0x5a, 0x46,
  0x08, 0xe7, 0xf2, 0x2f, 0x73, 0x6c, 0x94, 0x7a, 0x57,
  0x08, 0xe7, 0x64, 0x97, 0x56, 0xdf, 0xbe, 0x28, 0x1d,
  0x08, 0x05, 0xbe, 0x88, 0x63, 0x91, 0xf3, 0x61, 0x13,
#endif

  /* Power UP */
  0x07, 0x02, 0x81,
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_TCXO
  0x01,
#else
  0x00,
#endif
  GET_SI446X_XO_BYTE(3), GET_SI446X_XO_BYTE(2), GET_SI446X_XO_BYTE(1), GET_SI446X_XO_BYTE(0),

  /* global config */
  0x08, 0x11, 0x00, 0x04, 0x00, SI446X_X0_TUNE_VALUE, 0x00, 0x18, 0x60,
#if CONFIG_DRIVER_RFPACKET_SI446X_MAX_PKT_SIZE <= SI446X_FIFO_SIZE
  /* interrupt PACKET_SENT|PACKET_RX|CRC_ERROR|UDF_OVF */
  0x08, 0x11, 0x01, 0x04, 0x00, 0x05, 0x38, 0x0, 0x20,
#else
  /* interrupt PACKET_SENT|PACKET_RX|CRC_ERROR|TX_THRD|RX_THRD|UDF_OVF */
  0x08, 0x11, 0x01, 0x04, 0x00, 0x05, 0x3b, 0x0, 0x20,
#endif
  /* packet config */
  0x0B, 0x11, 0x12, 0x07, 0x06, 0x82, 0x0, 0x22, 0x01, 0x00, SI446X_FIFO_THRESHOLD, SI446X_FIFO_THRESHOLD,
  /* Packet handler configuration */
  0x05, 0x11, 0x12, 0x01, 0x10, 0xA2,
  0x0B, 0x11, 0x12, 0x07, 0x22, 0x01, 0x00, 0x82, 0x00, 0xFF, 0x00, 0x0a,
  0x00
};