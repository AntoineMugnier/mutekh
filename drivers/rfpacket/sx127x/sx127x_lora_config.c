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

#define LOGK_MODULE_ID "s127"

#include "sx127x_lora_spi.h"

// Const values


// Private functions prototypes
static void sx127x_config_calc_time_consts(struct sx127x_ctx_s *pv, uint32_t drate);
static error_t sx127x_build_rf_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static error_t sx127x_build_pk_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq);

struct sx127x_config_bw_s {
  uint32_t BITFIELD(bw,24);
  uint8_t  bits;
};

static const struct sx127x_config_bw_s sx127x_lora_config_bw[] = {
  {   7800, 0x0 },
  {  10400, 0x1 },
  {  15600, 0x2 },
  {  20800, 0x3 },
  {  31250, 0x4 },
  {  41700, 0x5 },
  {  62500, 0x6 },
  { 125000, 0x7 },
  { 250000, 0x8 },
  { 500000, 0x9 },
};

static void sx127x_lora_config_pk(struct sx127x_ctx_s *pv, const struct dev_rfpacket_pk_cfg_lora_s *cfg) {
  uint8_t *modemcfg1 = &pv->config_regs.modemcfg[0];
  uint8_t *modemcfg2 = &pv->config_regs.modemcfg[1];

  /* Start updating the configuration. */
  pv->config_regs.sw = pk_cfg->sw_value;
  pv->config_regs.pl = pk_cfg->pb_len;

  *modemcfg1 &= 0xf0;
  *modemcfg2 &= 0xfb;

  if (!cfg->header)
    *modemcfg1 |= 0x1;

  *modemcfg1 |= (cfg->crate & 0x7) << 1;

  if (cfg->crc)
    *modemcfg2 |= 0x4;
}

static void sx127x_lora_config_rf(struct sx127x_ctx_s *pv, const struct dev_rfpacket_rf_cfg_lora_s *cfg) {
  uint8_t bw = 0;
  uint_fast8_t i;

  uint8_t *modemcfg1 = &pv->config_regs.modemcfg[0];
  uint8_t *modemcfg2 = &pv->config_regs.modemcfg[1];
  uint8_t *modemcfg3 = &pv->config_regs.modemcfg[2];

  switch (rq->type)
    {
    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
      pv->config_regs.iq = cfg->iq_inverted ? 0x67 : 0x27;
      break;

    case DEV_RFPACKET_RQ_TX:
      pv->config_regs.iq = cfg->iq_inverted ? 0x26 : 0x27;
      break;

    default:
      UNREACHABLE(); 
    }

  for (i = 0; i < ARRAY_SIZE(sx127x_lora_config_bw); ++i) {
      if (sx127x_lora_config_bw[i].bw == cfg->common.rx_bw)
        bw = sx127x_lora_config_bw[i].bits;
  }

  *modemcfg1 &= 0x0f;
  *modemcfg2 &= 0x07; /* force TX continuous to zero */
  *modemcfg3  = 0;

  *modemcfg1 |= (bw & 0xf) << 4;
  *modemcfg2 |= (cfg->spreading & 0xf) << 4;

  /* If the symbol duration is more than 16ms, than low data rate optimization
   * must be enabled. */
  uint8_t const ts = (uint16_t)(1 << cfg->spreading) * 1000 / cfg->common.rx_bw;

  if (ts >= 16)
    *modemcfg3 |= 0x8;

  // CONFIG FREQ
  int32_t  const freq_offset = channel * cfg->common.chan_spacing;
  uint64_t const freq = (int64_t)cfg->common.frequency + freq_offset;

  /* Truncate to lowest 32 bits */
  int16_t channel = rq->channel;
  uint8_t data[4] = &pv->config_freq[0];
  uint32_t frf = (freq << 19) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;
  endian_be32_na_store(data, frf << 8);
}












// Private functions
static void sx127x_config_calc_time_consts(struct sx127x_ctx_s *pv, uint32_t drate) {
  assert(drate != 0);
  // Calc time byte in us
  dev_timer_delay_t tb = 8000000 / drate;
  dev_timer_init_sec(pv->timer, &(pv->gctx.time_byte), 0, tb, 1000000);
  // Calc other time constants
  pv->mpst = 2 * (SX127X_FIFO_SIZE - SX127X_FIFO_THRESHOLD) * pv->gctx.time_byte + pv->bt;
  pv->ccad = 2 * 8 * pv->gctx.time_byte + pv->bt;
}

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG

static error_t sx127x_build_dynamic_rf_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  // Retrieve data struct
  const struct dev_rfpacket_rf_cfg_fairtx_s *fairtx = NULL;
  const struct dev_rfpacket_rf_cfg_std_s *common = NULL;
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = NULL;
  const struct dev_rfpacket_rf_cfg_ask_s *cask = NULL;
  const struct dev_rfpacket_rf_cfg_lora_s *clora = NULL;

  switch (cfg->mod) {
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

    case DEV_RFPACKET_LORA:
      clora = const_dev_rfpacket_rf_cfg_lora_s_cast(cfg);
      common = &clora->common;
    break;

    default:
      return -ENOTSUP;
    break;
  }
  // Calculate time values
  sx127x_config_calc_time_consts(pv, common->drate);
  pv->curr_drate = common->drate;

  //printk("Rf config array: %P\n", pv->rf_cfg_array, SX127X_RF_CFG_ARRAY_SIZE);

  // Set current rf_cfg
  pv->curr_rf_cfg_data = pv->rf_cfg_array;
  pv->curr_rf_cfg_size = SX127X_RF_CFG_DATA_SIZE;
  return 0;
}
#endif



#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
static error_t sx127x_build_dynamic_pk_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_s *cfg = rq->pk_cfg;

  // Retrieve data struct
  const struct dev_rfpacket_pk_cfg_basic_s *cbasic = NULL;

  switch (cfg->format) {
    case DEV_RFPACKET_FMT_SLPC:
      cbasic = const_dev_rfpacket_pk_cfg_basic_s_cast(cfg);
    break;

    default:
      return -ENOTSUP;
    break;
  }

  // Set current pk_config
  pv->curr_pk_cfg_data = pv->pk_cfg_array;
  pv->curr_pk_cfg_size = SX127X_PK_CFG_DATA_SIZE;
  return 0;
}
#endif

static error_t sx127x_build_static_rf_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_static_s *cstatic = const_dev_rfpacket_rf_cfg_static_s_cast(rq->rf_cfg);
  struct sx127x_rf_cfg_s *cfg = NULL;

  // Retrieve config
  error_t err = device_get_param_blob(pv->dev, cstatic->cfg_name, 0, (const void **)&cfg);
  if (err != 0) {
    logk_trace("Couldn't retrieve rf param blob.");
    return err;
  }
  assert(cfg);
  // Calc time constants
  sx127x_config_calc_time_consts(pv, cfg->drate);
  // Note info
  //printk("RF CONFIG: %d, %d, %d, %d, %P\n", cfg->drate, cfg->jam_rssi, cfg->lbt_rssi, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->jam_rssi = cfg->jam_rssi;
  pv->lbt_rssi = cfg->lbt_rssi;
  pv->curr_drate = cfg->drate;
  pv->curr_rf_cfg_size = cfg->config_size;
  pv->curr_rf_cfg_data = cfg->config_data;
  return 0;
}

static error_t sx127x_build_extern_rf_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_extern_s *cextern = const_dev_rfpacket_rf_cfg_extern_s_cast(rq->rf_cfg);

  // Retrieve config
  struct sx127x_rf_cfg_s *cfg = cextern->p_cfg;
  assert(cfg);
  // Calc time constants
  sx127x_config_calc_time_consts(pv, cfg->drate);
  // Note info
  //printk("RF CONFIG: %d, %d, %d, %d, %P\n", cfg->drate, cfg->jam_rssi, cfg->lbt_rssi, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->jam_rssi = cfg->jam_rssi;
  pv->lbt_rssi = cfg->lbt_rssi;
  pv->curr_drate = cfg->drate;
  pv->curr_rf_cfg_size = cfg->config_size;
  pv->curr_rf_cfg_data = cfg->config_data;
  return 0;
}

static error_t sx127x_build_static_pk_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_static_s *cstatic = const_dev_rfpacket_pk_cfg_static_s_cast(rq->pk_cfg);
  struct sx127x_pk_cfg_s *cfg = NULL;

  // Retrieve config
  error_t err = device_get_param_blob(pv->dev, cstatic->cfg_name, 0, (const void **)&cfg);
  if (err != 0) {
    logk_trace("Couldn't retrieve pk param blob.");
    return err;
  }
  assert(cfg);
  // Note info
  //printk("PK CONFIG: 0x%02x, 0x%02x, %d, %P\n", cfg->prot1, cfg->prot2, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->curr_prot1 = cfg->prot1;
  pv->curr_prot2 = cfg->prot2;
  pv->curr_pk_cfg_size = cfg->config_size;
  pv->curr_pk_cfg_data = cfg->config_data;
  return 0;
}

static error_t sx127x_build_extern_pk_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_extern_s *cextern = const_dev_rfpacket_pk_cfg_extern_s_cast(rq->pk_cfg);

  // Retrieve config
  struct sx127x_pk_cfg_s *cfg = cextern->p_cfg;
  assert(cfg);
  // Note info
  //printk("PK CONFIG: 0x%02x, 0x%02x, %d, %P\n", cfg->prot1, cfg->prot2, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->curr_prot1 = cfg->prot1;
  pv->curr_prot2 = cfg->prot2;
  pv->curr_pk_cfg_size = cfg->config_size;
  pv->curr_pk_cfg_data = cfg->config_data;
  return 0;
}

static error_t sx127x_build_rf_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  logk_trace("RF configuration");

  switch (cfg->mod) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
    case DEV_RFPACKET_ASK:
    case DEV_RFPACKET_LORA:
      return sx127x_build_dynamic_rf_config(pv, rq);
#endif

    case DEV_RFPACKET_MOD_STATIC:
      return sx127x_build_static_rf_config(pv, rq);

    case DEV_RFPACKET_MOD_EXTERN:
      return sx127x_build_extern_rf_config(pv, rq);

    default:
      return -ENOTSUP;
  }
}

static error_t sx127x_build_pk_config(struct sx127x_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_s *cfg = rq->pk_cfg;

  logk_trace("PKT configuration");

  switch (cfg->format) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
    case DEV_RFPACKET_FMT_SLPC:
    case DEV_RFPACKET_FMT_LORA:
      return sx127x_build_dynamic_pk_config(pv, rq);
#endif

    case DEV_RFPACKET_FMT_STATIC:
      return sx127x_build_static_pk_config(pv, rq);

    case DEV_RFPACKET_FMT_EXTERN:
      return sx127x_build_extern_pk_config(pv, rq);

    default:
      return -ENOTSUP;
  }
}


// Public functions

error_t sx127x_build_config(struct sx127x_ctx_s *pv) {
  error_t err = 0;
  struct dev_rfpacket_rq_s *rq = pv->gctx.rq;

  if ((pv->flags & sx127x_FLAGS_RF_CONFIG_OK) == 0) {
    err = sx127x_build_rf_config(pv, rq);
    if (err != 0) {
      return err;
    }
    // Update rf cfg values
    ((struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg)->cache.dirty = 0;
    pv->rf_cfg = rq->rf_cfg;
  }
  if ((pv->flags & sx127x_FLAGS_PK_CONFIG_OK) == 0) {
    err = sx127x_build_pk_config(pv, rq);
    if (err != 0) {
      return err;
    }
    // Update pk cfg values
    ((struct dev_rfpacket_pk_cfg_s *)rq->pk_cfg)->cache.dirty = 0;
    pv->pk_cfg = rq->pk_cfg;
  }
  return 0;
}

uint8_t sx127x_build_pwr(struct sx127x_ctx_s *pv, int16_t pwr) {
  uint8_t reg_pwr = 0;

  // Check if power valid
  if (!sx127x_VALID_TX_POWER(pwr)) {
    logk_trace("Unsupported power level requested");
    return 0;
  }
  // Check if power need to be configured
  pv->flags &= ~sx127x_FLAGS_TX_POWER_OK;

  if (pv->pwr == pwr) {
    pv->flags |= sx127x_FLAGS_TX_POWER_OK;
    return 0;
  }
  // Set new power value
  pv->pwr = pwr;
  // Calc power reg value
  if (pwr >= (sx127x_MAX_PA_VALUE * 8)) {
    reg_pwr = 1;
  } else {
    // Reg = 29 - 2 * dbm value
    reg_pwr = (uint8_t)((int32_t)29 - (pwr / 4)); // 0.5 dbm res, power value is truncated
  }
  return reg_pwr;
}

bool sx127x_config_check_fairtx_valid(const struct dev_rfpacket_rf_cfg_s *rfcfg) {
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk;
  const struct dev_rfpacket_rf_cfg_ask_s *cask;

  switch (rfcfg->mod) {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      cfsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rfcfg);
      if (cfsk->fairtx.mode == DEV_RFPACKET_NO_FAIRTX) {
        return false;
      }
    break;

    case DEV_RFPACKET_ASK:
      cask = const_dev_rfpacket_rf_cfg_ask_s_cast(rfcfg);
      if (cask->fairtx.mode == DEV_RFPACKET_NO_FAIRTX) {
        return false;
      }
    break;

    default:
      break;
  }
  return true;
}

uint32_t sx127x_config_get_freq(const struct dev_rfpacket_rf_cfg_s *cfg, uint8_t chan) {
  const struct dev_rfpacket_rf_cfg_std_s *common = NULL;
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = NULL;
  const struct dev_rfpacket_rf_cfg_ask_s *cask = NULL;

  switch (cfg->mod) {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      cfsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(cfg);
      common = &cfsk->common;
    break;

    case DEV_RFPACKET_ASK:
      cask = const_dev_rfpacket_rf_cfg_ask_s_cast(cfg);
      common = &cask->common;
    break;

    default:
    break;
  }
  if (common != NULL) {
    return common->frequency + chan * common->chan_spacing;
  }
  return 0;
}