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
#include "power_curves.h"

// Baudrate calc values
#define EFR32_BR_MAX_DEN 0x8000 // Found empirically through simplicity studio
#define EFR32_BR_MAX_NUM 0xff

// Power calculation values
#define EFR32_POW_STRIPE_MAX_VAL  32
#define EFR32_POW_MIN_DBM        -176 // 0.125 dbm unit (-22dbm)
#define EFR32_POW_MAX_DBM         160 // 0.125 dbm unit (20dbm)
#define EFR32_POW_MIN_RAW         1
#define EFR32_POW_MAX_RAW         248

static void efr32_send_radio_config(uint32_t cfg_size, uint32_t *p_cfg);
static dev_timer_delay_t efr32_radio_calc_time(struct radio_efr32_rfp_ctx_s *ctx, uint32_t drate);
static error_t efr32_calc_power(dev_rfpacket_pwr_t pwr_dbm, uint32_t freq, uint32_t *p_sgpac_val, uint32_t *p_pac_val);
static error_t efr32_build_static_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static error_t efr32_build_extern_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static error_t efr32_build_static_pk_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
static error_t efr32_build_extern_pk_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
static uint32_t efr32_build_baudrate(uint32_t br);
static error_t efr32_build_gfsk_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
#endif

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
static error_t efr32_build_slpc_pkt_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
#endif

// *** PRIVATE FUNCTIONS ***

static void efr32_send_radio_config(uint32_t cfg_size, uint32_t *p_cfg) {
  // Parse config table (addr, data) pairs
  for (uint32_t i = 0; i < cfg_size - 1; i += 2) {
    // Retrieve data
    uint32_t addr = p_cfg[i];
    uint32_t data = p_cfg[i+1];
    // Check addr and write data
    assert(addr);
    cpu_mem_write_32(addr, data);
  }
}

static dev_timer_delay_t efr32_radio_calc_time(struct radio_efr32_rfp_ctx_s *ctx, uint32_t drate) {
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  uint64_t result, num, denom;

  assert(drate);
  num = 8000000 / drate * pv->freq.num;
  denom = 1000000 * pv->freq.denom;
  result = num / denom;
  dev_timer_delay_t tb = (dev_timer_delay_t)result;
  return tb;
}

static error_t efr32_calc_power(dev_rfpacket_pwr_t pwr_dbm, uint32_t freq, uint32_t *p_sgpac_val, uint32_t *p_pac_val) {
  int64_t pa_curve_value;
  uint32_t pwr_raw = 0;
  const struct _efr32_pa_data_s *efr32_pa_data = NULL;
  const struct _efr32_pa_curve_s *efr32_pa_curve = NULL;
  // Limit intput power value
  if (pwr_dbm < EFR32_POW_MIN_DBM) {
    pwr_dbm = EFR32_POW_MIN_DBM;
    logk_trace("Power requested too low, setting value to %d", pwr_dbm);
  } else if (pwr_dbm > EFR32_POW_MAX_DBM) {
    pwr_dbm = EFR32_POW_MAX_DBM;
    logk_trace("Power requested too high, setting value to %d", pwr_dbm);
  }
  // Get pa curves
  if (freq > 1000000000) {
    efr32_pa_data = &efr32_radio_2g4_pa_data;
  } else {
    efr32_pa_data = &efr32_radio_sg_pa_data;
  }
  assert(efr32_pa_data);
  // Get pa curve index
  for (int16_t idx = efr32_pa_data->data_size - 1; idx > 0; idx--) {
    efr32_pa_curve = &efr32_pa_data->data[idx];

    if (pwr_dbm < efr32_pa_curve->max_pwr_dbm) {
      break;
    }
  }
  assert(efr32_pa_curve);
  // Convert pwr_dbm from 1/8th to 1/10th dbm
  pwr_dbm = pwr_dbm * 10 / 8;
  // Calc pa curve value
  pa_curve_value = efr32_pa_curve->slope * pwr_dbm + efr32_pa_curve->offset;
  // Calc raw and value rounding
  pwr_raw = (pa_curve_value + 500) / 1000;
  // Limit value
  if (pwr_raw < EFR32_POW_MIN_RAW) {
    pwr_raw = EFR32_POW_MIN_RAW;
  } else if (pwr_raw > EFR32_POW_MAX_RAW) {
    pwr_raw = EFR32_POW_MAX_RAW;
  }
  // Calc reg value based on raw
  uint8_t stripe = pwr_raw % EFR32_POW_STRIPE_MAX_VAL;
  uint8_t slice_cascode = (bit(pwr_raw / EFR32_POW_STRIPE_MAX_VAL + 1) - 1);
  bool en_SGVBATDET = (pwr_raw >= 120) ? true : false; // TODO check is useful ?
  uint32_t power_reg = (slice_cascode << EFR32_RAC_SGPACTRL0_CASCODE_IDX) | (slice_cascode << EFR32_RAC_SGPACTRL0_SLICE_IDX) |
    (stripe << EFR32_RAC_SGPACTRL0_STRIPE_IDX) | EFR32_RAC_SGPACTRL0_DACGLITCHCTRL;
  *p_pac_val = power_reg;
  *p_sgpac_val = power_reg | (en_SGVBATDET << EFR32_RAC_SGPACTRL0_SGVBATDET_IDX);
  return 0;
}

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
static uint32_t efr32_build_baudrate(uint32_t br) {
  uint32_t num = 0, den = 0;
  // Formula for Tx baudrate: Br = (ModemFreq * txbrnum)/(8 * txbrden)
  // Calc (reverse) ratio with rounding
  uint64_t ratio =  (CONFIG_DRIVER_EFR32_RADIO_HFXO_CLK + 4 * br) / (8 * br);
  // Set denominator
  den = ratio * EFR32_BR_MAX_NUM;
  // Check if denominator is within dynamic to set numerator
  if (den > EFR32_BR_MAX_DEN) {
      num = EFR32_BR_MAX_DEN / ratio;
      den = ratio * num;
  } else {
      num = EFR32_BR_MAX_NUM;
  }
  //printk("Radio baudrate: %d 0x%x\n", br, (EFR32_MODEM_TXBR_TXBRNUM(rnum) | EFR32_MODEM_TXBR_TXBRDEN(rden)));
  return (EFR32_MODEM_TXBR_TXBRNUM(num) | EFR32_MODEM_TXBR_TXBRDEN(den));
  // TODO rx baudrate (not simple !)
}

static error_t efr32_build_gfsk_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rq->rf_cfg);
  const struct dev_rfpacket_rf_cfg_std_s *common = &cfsk->common;

  // Init config
  efr32_rfp_fsk_init(ctx);
  // Build config
  uint32_t div = cpu_mem_read_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR);
  div = EFR32_SYNTH_DIVCTRL_LODIVFREQCTRL_GET(div);
  // Configure frequency
  uint64_t f = ((uint64_t)(common->frequency) * div) << 19;
  f /= CONFIG_DRIVER_EFR32_RADIO_HFXO_CLK;
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_FREQ_ADDR, (uint32_t)f);
  // Configure channel spacing
  uint64_t chsp = ((uint64_t)(common->chan_spacing) * div) << 19;
  chsp /= CONFIG_DRIVER_EFR32_RADIO_HFXO_CLK;
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHSP_ADDR, (uint32_t)chsp);
  // Configure baudrate
  uint32_t br = efr32_build_baudrate(common->drate);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TXBR_ADDR, br);
  // Calc time byte
  ctx->gctx.time_byte = efr32_radio_calc_time(ctx, common->drate);
  // Note config values
  ctx->curr_freq = common->frequency;
  ctx->curr_drate = common->drate;
  return 0;
}
#endif

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
static error_t efr32_build_slpc_pkt_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_basic_s *cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

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
  // Configure variable length mode
  uint32_t x = EFR32_FRC_CTRL_RXFCDMODE(FCDMODE2) |
               EFR32_FRC_CTRL_TXFCDMODE(FCDMODE2) |
               EFR32_FRC_CTRL_BITORDER(MSB) |
               EFR32_FRC_CTRL_BITSPERWORD(7);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);

  x = EFR32_FRC_DFLCTRL_DFLMODE(SINGLEBYTE) |
      EFR32_FRC_DFLCTRL_MINLENGTH(1) |
      EFR32_FRC_DFLCTRL_DFLBITS(8);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, x);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MAXLENGTH_ADDR, EFR32_RADIO_RFP_BUFFER_SIZE - 1);

  // Configure Sync Word
  uint8_t sw = cfg->sw_len + 1;
  if ((sw >> 5) || (sw % 4)) {
    return -ENOTSUP;
  }
  x = EFR32_MODEM_CTRL1_SYNCBITS(cfg->sw_len);
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
  x = EFR32_MODEM_PRE_BASEBITS(cfg->pb_pattern_len);
  uint32_t msk = (1 << (cfg->pb_pattern_len + 1)) - 1;
  uint32_t preamble = cfg->pb_pattern & msk;

  if (preamble == (0xAAAAAAAA & msk)) {
    EFR32_MODEM_PRE_BASE_SET(x, 10); // TYPE 1010
  } else if (preamble == (0x55555555 & msk)) {
    EFR32_MODEM_PRE_BASE_SET(x, 01); // TYPE 0101
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
  // Note current config value
  ctx->curr_rx_pb_len = cfg->rx_pb_len;
  return 0;
}
#endif

static error_t efr32_build_static_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_static_s *cstatic = const_dev_rfpacket_rf_cfg_static_s_cast(rq->rf_cfg);
  struct radio_efr32_rf_cfg_s *cfg = NULL;

  // Retrieve config
  error_t err = device_get_param_blob(ctx->pv.dev, cstatic->cfg_name, 0, (const void **)&cfg);
  if (err != 0) {
    logk_trace("Couldn't retrieve rf param blob.");
    return err;
  }
  assert(cfg);
  // Raise sigfox flag if needed
#ifdef CONFIG_DRIVER_EFR32_SIGFOX
  if (strcmp(cstatic->cfg_name, "rf_sigfox") == 0) {
    ctx->isSigfox = true;
  } else {
    ctx->isSigfox = false;
  }
#endif
  // Send config
  efr32_send_radio_config(cfg->config_size, cfg->config_data);
  // Note info
  //printk("RF CONFIG: %d, %d, %P\n", cfg->drate, cfg->config_size, cfg->config_data, cfg->config_size);
  ctx->curr_freq = cfg->frequency;
  ctx->curr_drate = cfg->drate;
  // Calc time constants
  ctx->gctx.time_byte = efr32_radio_calc_time(ctx, cfg->drate);
  return 0;
}

static error_t efr32_build_extern_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_extern_s *cextern = const_dev_rfpacket_rf_cfg_extern_s_cast(rq->rf_cfg);

  // Retrieve config
  struct radio_efr32_rf_cfg_s *cfg = cextern->p_cfg;
  assert(cfg);
  // Send config
  efr32_send_radio_config(cfg->config_size, cfg->config_data);
  // Note info
  //printk("RF CONFIG: %d, %d, %P\n", cfg->drate, cfg->config_size, cfg->config_data, cfg->config_size);
  ctx->curr_freq = cfg->frequency;
  ctx->curr_drate = cfg->drate;
  // Calc time constants
  ctx->gctx.time_byte = efr32_radio_calc_time(ctx, cfg->drate);
  return 0;
}

static error_t efr32_build_static_pk_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_static_s *cstatic = const_dev_rfpacket_pk_cfg_static_s_cast(rq->pk_cfg);
  struct radio_efr32_pk_cfg_s *cfg = NULL;

  // Retrieve config
  error_t err = device_get_param_blob(ctx->pv.dev, cstatic->cfg_name, 0, (const void **)&cfg);
  if (err != 0) {
    logk_trace("Couldn't retrieve rf param blob.");
    return err;
  }
  assert(cfg);
  // Send config
  efr32_send_radio_config(cfg->config_size, cfg->config_data);
  // Note info
  ctx->curr_rx_pb_len = cfg->rx_preamb_len;
  //printk("PK CONFIG: %d, %P\n", cfg->config_size, cfg->config_data, cfg->config_size);
  return 0;
}

static error_t efr32_build_extern_pk_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_extern_s *cextern = const_dev_rfpacket_pk_cfg_extern_s_cast(rq->pk_cfg);

  // Retrieve config
  struct radio_efr32_pk_cfg_s *cfg = cextern->p_cfg;
  assert(cfg);
  // Send config
  efr32_send_radio_config(cfg->config_size, cfg->config_data);
  // Note info
  ctx->curr_rx_pb_len = cfg->rx_preamb_len;
  //printk("PK CONFIG: %d, %P\n", cfg->config_size, cfg->config_data, cfg->config_size);
  return 0;
}



// *** PUBLIC FUNCTIONS ***

int32_t efr32_calc_synth_ratio(uint32_t freq) {
// Returned synth ratio as 1/128th
  if (freq > 1000000000) {
    return 9344;
  } else if (freq > 779000000) {
    return 3072;
  } else if (freq > 584000000) {
    return 2342;
  } else if (freq > 358000000) {
    return 1562;
  } else if (freq > 191000000) {
    return 934;
  } else {
    return 589;
  }
}

error_t efr32_set_tx_power(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  uint32_t sg_pac_reg, pac_reg;

  error_t err = efr32_calc_power(rq->tx_pwr, ctx->curr_freq, &sg_pac_reg, &pac_reg);

  if (err != 0) {
    return err;
  }
  // Write power value
  //cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PACTRL0_ADDR, pac_reg); // Bootloader crc bug
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPACTRL0_ADDR, sg_pac_reg);
  return 0;
}

void efr32_rfp_set_cca_threshold(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
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

error_t efr32_build_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  switch (rq->rf_cfg->mod) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
    case DEV_RFPACKET_GFSK:
      return efr32_build_gfsk_rf_config(ctx, rq);
#endif

    case DEV_RFPACKET_MOD_STATIC:
      return efr32_build_static_rf_config(ctx, rq);

    case DEV_RFPACKET_MOD_EXTERN:
      return efr32_build_extern_rf_config(ctx, rq);

    default:
      return -ENOTSUP;
  }
}

error_t efr32_build_pkt_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq) {
  switch (rq->pk_cfg->format) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
    case DEV_RFPACKET_FMT_SLPC:
      return efr32_build_slpc_pkt_config(ctx, rq);
#endif

    case DEV_RFPACKET_FMT_STATIC:
      return efr32_build_static_pk_config(ctx, rq);

    case DEV_RFPACKET_FMT_EXTERN:
      return efr32_build_extern_pk_config(ctx, rq);

    default:
      return -ENOTSUP;
  }
}

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
error_t efr32_rfp_fsk_init(struct radio_efr32_rfp_ctx_s *ctx) {
  uint32_t base[2] = {EFR32_FRC_ADDR, EFR32_RADIO_SEQ_RAM_ADDR};
  uint16_t i = 0;

  const uint32_t generated[] = {
    0x00013008UL, 0x0100AC13UL,
    0x00023030UL, 0x00104000UL,
    /*    3034 */ 0x00000003UL,
    0x00013040UL, 0x00000000UL,
    0x000140A0UL, 0x0F0027AAUL,
    0x000140B8UL, 0x0023C000UL,
    0x000140F4UL, 0x00001020UL,
    0x00024134UL, 0x00000880UL,
    /*    4138 */ 0x000087F6UL,
    0x00024140UL, 0x008800E0UL,
    /*    4144 */ 0x4D52E6C1UL,
    0x00044160UL, 0x00000000UL,
    /*    4164 */ 0x00000000UL,
    /*    4168 */ 0x00000006UL,
    /*    416C */ 0x00000006UL,
    0x00086014UL, 0x00000010UL,
    /*    6018 */ 0x04000000UL,
    /*    601C */ 0x0002C00FUL,
    /*    6020 */ 0x00005000UL,
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV
    /*    6024 */ (EFR32_MODEM_CTRL3_ANTDIVMODE(ANTSELRSSI) |
#else
    /*    6024 */ (EFR32_MODEM_CTRL3_ANTDIVMODE(ANTENNA0) |
#endif
                   EFR32_MODEM_CTRL3_TSAMPMODE(ON) |
                   EFR32_MODEM_CTRL3_TSAMPDEL(3) |
                   EFR32_MODEM_CTRL3_TSAMPLIM(8)),
    /*    6028 */ 0x03000000UL,
    /*    602C */ 0x00000000UL,
    /*    6030 */ 0x00000000UL,
    0x00066050UL, 0x00FF7C83UL, // Baudrate TX
    /*    6054 */ 0x00000F73UL, // Baudrate RX
    /*    6058 */ 0x00000160UL,
    /*    605C */ 0x00140011UL,
    /*    6060 */ 0x000075E3UL,
    /*    6064 */ 0x00000000UL,
    0x000C6078UL, 0x11A0071BUL,
    /*    607C */ 0x00000000UL,
    /*    6080 */ 0x003B0373UL,
    //0x002d03b9UL, // FDEV tx rx
    /*    6084 */ 0x00000000UL,
    /*    6088 */ 0x00000000UL,
    /*    608C */ 0x22140A04UL,
    /*    6090 */ 0x4F4A4132UL,
    /*    6094 */ 0x00000000UL,
    /*    6098 */ 0x00000000UL,
    /*    609C */ 0x00000000UL,
    /*    60A0 */ 0x00000000UL,
    /*    60A4 */ 0x00000000UL,
    0x000760E4UL, 0x04000080UL,
    /*    60E8 */ 0x00000000UL,
    /*    60EC */ 0x07830464UL,
    /*    60F0 */ 0x3AC81388UL,
    /*    60F4 */ 0x000A209CUL,
    /*    60F8 */ 0x00206100UL,
    /*    60FC */ 0x123556B7UL,
    0x00036104UL, 0x00108000UL,
    /*    6108 */ 0x29043020UL,
    /*    610C */ 0x0040BB88UL,
    0x00016120UL, 0x00000000UL,
    0x00086130UL, 0x00FA53E8UL,
    /*    6134 */ 0x00000000UL,
    /*    6138 */ 0x00000000UL,
    /*    613C */ 0x00000000UL,
    /*    6140 */ 0x00000000UL,
    /*    6144 */ 0x00000000UL,
    /*    6148 */ 0x00000000UL,
    /*    614C */ 0x00000001UL,
    0x00077014UL, 0x000270FEUL,
    /*    7018 */ 0x00000300UL,
    /*    701C */ 0x834A0060UL,
    /*    7020 */ 0x00000000UL,
    /*    7024 */ 0x00000082UL,
    /*    7028 */ 0x00000000UL,
    /*    702C */ 0x000000D5UL,
    0x00027048UL, 0x0000383EUL,
    /*    704C */ 0x000025BCUL,
    0x00037070UL, 0x00120105UL,
    /*    7074 */ 0x00083019UL,
    /*    7078 */ 0x006D8480UL,
    0xFFFFFFFFUL,
  };
  while(1) {
    uint32_t v0 = generated[i];

    if (v0 == 0xFFFFFFFF) {
      break;
    }
    uint32_t offset = (v0 >> 24) & 0xFF;
    uint32_t count = (v0 >> 16) & 0xFF;
    uint32_t idx = 0;
    assert(offset< 2);

    while(count--) {
      uint32_t addr = (v0 & 0xFFFF) + idx;
      addr |= base[offset];
      uint32_t v1 = generated[i + 1];
      cpu_mem_write_32(addr, v1);
      //efr32_radio_printk("0x%x 0x%x\n", addr, v1);
      i += 1;
      idx += 4;
    }
    i += 1;
  }
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CTRL_ADDR, 0x100ac3f);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CALCTRL_ADDR, 0x42801);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCDACCTRL_ADDR, 0x23);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, 0x18);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOTUNING_ADDR, 0x7f);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOGAIN_ADDR, 0x23);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_AUXVCDACCTRL_ADDR, 0x7);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CAPCALCYCLECNT_ADDR, 0x3b);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CTRL_ADDR, 0x380);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSCTRL_ADDR, 0xc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSIDLESEQ_ADDR, 0xbc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_HFXORETIMECTRL_ADDR, 0x540);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PRESC_ADDR, 0x7);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHENCTRL_ADDR, 0x87f2);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHREGCTRL_ADDR, 0x3636d80);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL_ADDR, 0x1147b);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL_ADDR, 0x2e);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCAL_ADDR, 0x24);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_AUXCTRL_ADDR, 0x1688030);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFENCTRL_ADDR, 0x1001000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGRFENCTRL0_ADDR, 0xb0000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGLNAMIXCTRL_ADDR, 0x186db00);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PACTRL0_ADDR, 0x5b01c1c0);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPACTRL0_ADDR, 0x5b01c1c0);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPAPKDCTRL_ADDR, 0x108d000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL0_ADDR, 0x7000445);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL1_ADDR, 0x84523);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCTRL_ADDR, 0x34);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCAL_ADDR, 0x30191a);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACAL_ADDR, 0x46404301);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RCTUNE_ADDR, 0x1f001f);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_APC_ADDR, 0xff000000L);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_FREQOFFEST_ADDR, 0xed9d0000L);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPCTRL_ADDR, 0x600);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPLEV_ADDR, 0x960000);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCCOMP_ADDR, 0x33);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_BREST_ADDR, 0x2c7);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_POE_ADDR, 0x1ff);

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_SLICECODE_ADDR, 0xca86543);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE1_ADDR, 0x6543210);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE2_ADDR, 0x18b52507);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE3_ADDR, 0x25183dcd);

  // Reduce HFXO boot-up time
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFXOTIMEOUTCTRL_ADDR, 0x2a047);

  return 0;
}
#endif