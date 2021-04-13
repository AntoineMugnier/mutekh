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

// Private types
struct sx127x_config_bw_s
{
  uint32_t BITFIELD(bw, 24);
  uint8_t  bits;
};

// Private variables
static const struct sx127x_config_bw_s sx127x_config_bw_tbl[] =
{
 /* RX bandwidth given in datasheet is for single side band. */
    { 2 * 2600  , 0x17 },
    { 2 * 3100  , 0x0F },
    { 2 * 3900  , 0x07 },
    { 2 * 5200  , 0x16 },
    { 2 * 6300  , 0x0E },
    { 2 * 7800  , 0x06 },
    { 2 * 10400 , 0x15 },
    { 2 * 12500 , 0x0D },
    { 2 * 15600 , 0x05 },
    { 2 * 20800 , 0x14 },
    { 2 * 25000 , 0x0C },
    { 2 * 31300 , 0x04 },
    { 2 * 41700 , 0x13 },
    { 2 * 50000 , 0x0B },
    { 2 * 62500 , 0x03 },
    { 2 * 83333 , 0x12 },
    { 2 * 100000, 0x0A },
    { 2 * 125000, 0x02 },
    { 2 * 166700, 0x11 },
    { 2 * 200000, 0x09 },
    { 2 * 250000, 0x01 },
};

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_MOD_LORA

static const struct sx127x_config_bw_s sx127x_lora_config_bw[] =
{
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


// Private functions

static error_t sx127x_lora_config_pk(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_pk_cfg_lora_s *cfg =
    const_dev_rfpacket_pk_cfg_lora_s_cast(rq->pk_cfg);

  uint8_t *modemcfg1 = &pv->lora_cfg.modemcfg[0];
  uint8_t *modemcfg2 = &pv->lora_cfg.modemcfg[1];

  /* Start updating the configuration. */
  pv->lora_cfg.sw = cfg->sw_value;
  pv->lora_cfg.pl = cfg->pb_len;

  *modemcfg1 &= 0xf0;
  *modemcfg2 &= 0xfb;

  if (!cfg->header)
    *modemcfg1 |= 0x1;

  *modemcfg1 |= (cfg->crate & 0x7) << 1;

  if (cfg->crc)
    *modemcfg2 |= 0x4;

  return 0;
}

static error_t sx127x_lora_config_rf(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_lora_s *cfg =
    const_dev_rfpacket_rf_cfg_lora_s_cast(rq->rf_cfg);

  uint8_t bw = 0;
  uint_fast8_t i;

  uint8_t *modemcfg1 = &pv->lora_cfg.modemcfg[0];
  uint8_t *modemcfg2 = &pv->lora_cfg.modemcfg[1];
  uint8_t *modemcfg3 = &pv->lora_cfg.modemcfg[2];

  for (i = 0; i < ARRAY_SIZE(sx127x_lora_config_bw); ++i)
  {
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
  int32_t  const freq_offset = rq->channel * cfg->common.chan_spacing;
  uint64_t const freq = (int64_t)cfg->common.frequency + freq_offset;

  /* Truncate to lowest 32 bits */
  uint32_t frf = (freq << 19) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;
  endian_be32_na_store(pv->lora_freq, frf << 8);

  return 0;
}

#endif


static uint8_t* sx127x_config_get_cfg_buff(struct sx127x_private_s *pv)
{
  return &pv->cfg_regs.cfg[pv->cfg_offset];
}

static void sx127x_config_calc_time_consts(struct sx127x_private_s *pv, uint32_t drate)
{
  assert(drate != 0);
  // Calc time byte in us
  dev_timer_delay_t tb = 8000000 / drate;
  dev_timer_init_sec(pv->timer, &(pv->gctx.time_byte), 0, tb, 1000000);

  /* Compute time to send a bit in us */
  dev_timer_delay_t t = 1000000 / drate;
  dev_timer_init_sec(pv->timer, &pv->timebit, 0, t, 1000000);
}

static inline void sx127x_config_freq_update(struct sx127x_private_s *pv, uint32_t channel)
{
  uint32_t const freq = (pv->cfg_regs.coeff.freq + channel * pv->cfg_regs.coeff.chan) >> 15;
  uint8_t *f = pv->cfg_regs.freq;

  f[0] = freq >> 16;
  f[1] = freq >> 8;
  f[2] = freq;

  pv->cfg_regs.channel = channel;
}

static inline void sx127x_config_coeff_freq_update(struct sx127x_private_s *pv, const struct dev_rfpacket_rf_cfg_std_s *rf_cfg, uint32_t channel)
{
  pv->cfg_regs.coeff.freq = ((uint64_t)rf_cfg->frequency << (19 + 15)) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;
  pv->cfg_regs.coeff.chan = ((uint64_t)rf_cfg->chan_spacing << (19 + 15)) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;

  sx127x_config_freq_update(pv, channel);
}

static uint8_t sx127x_get_bw(uint32_t bw)
{
  for (uint16_t i = 0; i < ARRAY_SIZE(sx127x_config_bw_tbl); i++)
    {
      if (bw <= sx127x_config_bw_tbl[i].bw)
        return sx127x_config_bw_tbl[i].bits;
    }

  return 0;
}

static error_t sx1276_build_bw_config(struct sx127x_private_s *pv,
                                      const struct dev_rfpacket_rf_cfg_std_s * __restrict__ rf_cfg,
                                      uint32_t default_)
{
  uint8_t *rf = sx127x_config_get_cfg_buff(pv);

  /* Bandwidth */
  *rf++ = 2;
  *rf++ = SX1276_REG_RXBW | 0x80;

  uint32_t bw = rf_cfg->rx_bw;

  if (bw == 0)
    bw = default_;

  uint8_t bwcode = sx127x_get_bw(bw);
  logk_trace("bandwidth: %u : 0x%02x", bw, bwcode);

  if (!bwcode)
    return -EINVAL;

  *rf++ = bwcode;

  uint32_t rx_tx_freq_err = rf_cfg->freq_err   /* remote freq error */
    + (uint64_t)rf_cfg->frequency * pv->osc_ppb / 1000000000; /* local freq error */

  /* AFC Bandwidth */
  bw += rx_tx_freq_err * 2;
  bwcode = sx127x_get_bw(bw);
  logk_trace("AFC bandwidth: %u : 0x%02x", bw, bwcode);

  *rf++ = bwcode ? bwcode : /* 500k */ 0x01;

  pv->cfg_regs.rxcfg |= SX1276_RXCONFIG_AFCAUTO_ON;

  *rf++ = 1;
  *rf++ = SX1276_REG_AFCFEI;
  *rf++ = SX1276_AFCFEI_AFCAUTOCLEAR_ON;

  // Add offset to buffer
  pv->cfg_offset += 7;

  return 0;
}



// Rf config

static error_t sx127x_build_fsk_config(struct sx127x_private_s *pv,
                                       struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_fsk_s * fsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rq->rf_cfg);

  if (sx1276_build_bw_config(pv, &fsk->common, /* carson rule */
                             fsk->common.drate + 2 * fsk->deviation))
    return -EINVAL;

  uint8_t *rf = sx127x_config_get_cfg_buff(pv);

  if (fsk->fairtx.mode == DEV_RFPACKET_LBT)
  /* Rssi threshold configuration */
    {
      if (fsk->fairtx.lbt.rssi > 0)
        return -EINVAL;

      *rf++ = 1;
      *rf++ = SX1276_REG_RSSITHRESH | 0x80;
      *rf++ = (abs(fsk->fairtx.lbt.rssi >> 3)) << 1;
      /* Rssi threshold in 0,25dBm steps */
      pv->cfg_regs.rssi_th = fsk->fairtx.lbt.rssi >> 1;
      // Add offset to buffer
      pv->cfg_offset += 3;
    }

  *rf++ = 2;
  *rf++ = SX1276_REG_FDEVMSB | 0x80;

  /* Deviation */
  uint16_t dev = ((uint64_t)fsk->deviation << 19)/CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;

  if (dev >> 13)
    return -EINVAL;

  endian_be16_na_store(rf, dev);

  rf += 2;

  /* PA config and PA Ramp */
  *rf++ = 2;
  *rf++ = SX1276_REG_PACONFIG | 0x80;
  *rf++ = 0x7f;

  /* Set Gaussian filter to 1 when GFSK */
  if (rq->rf_cfg->mod == DEV_RFPACKET_GFSK)
    *rf++ = 0x0A | SX1276_PARAMP_MODULATIONSHAPING_01;
  else
    *rf++ = 0x0A | SX1276_PARAMP_MODULATIONSHAPING_00;

  pv->cfg_regs.mode = SX1276_OPMODE_MODULATIONTYPE_FSK |
                 SX1276_OPMODE_LONGRANGEMODE_OFF;

  // Add offset to buffer
  pv->cfg_offset += 8;

  return 0;
}

static error_t sx127x_build_ask_config(struct sx127x_private_s *pv,
                                       struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_ask_s * ask = const_dev_rfpacket_rf_cfg_ask_s_cast(rq->rf_cfg);

  if (ask->symbols > 2)
    return -EINVAL;

  if (sx1276_build_bw_config(pv, &ask->common, 2 * ask->common.drate))
    return -EINVAL;

  uint8_t *rf = sx127x_config_get_cfg_buff(pv);

  if (ask->fairtx.mode == DEV_RFPACKET_LBT)
  /* Rssi threshold configuration */
    {
      if (ask->fairtx.lbt.rssi > 0)
        return -EINVAL;

      *rf++ = 1;
      *rf++ = SX1276_REG_RSSITHRESH | 0x80;
      *rf++ = (abs(ask->fairtx.lbt.rssi >> 3)) << 1;
      /* Rssi threshold in 0,25dBm steps */
      pv->cfg_regs.rssi_th = ask->fairtx.lbt.rssi >> 1;

      // Add offset to buffer
      pv->cfg_offset += 3;
    }

  *rf++ = 2;
  *rf++ = SX1276_REG_OOKPEAK | 0x80;
  *rf++ = SX1276_OOKPEAK_OOKTHRESHTYPE_PEAK | SX1276_OOKPEAK_BITSYNC_ON;
  /* Sensitivity of the OOK receiver */
  *rf++ = 0x80;

  /* PA config and PA Ramp */
  *rf++ = 2;
  *rf++ = SX1276_REG_PACONFIG | 0x80;
  *rf++ = 0x7f;
  *rf++ = 0x00;

  pv->cfg_regs.mode = SX1276_OPMODE_MODULATIONTYPE_OOK |
                 SX1276_OPMODE_LONGRANGEMODE_OFF;

  // Add offset to buffer
  pv->cfg_offset += 8;

  return 0;
}

static error_t sx127x_build_dynamic_rf_config(struct sx127x_private_s *pv,
                                            struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

  // Retrieve data struct
  const struct dev_rfpacket_rf_cfg_std_s *common = NULL;
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = NULL;
  const struct dev_rfpacket_rf_cfg_ask_s *cask = NULL;

  switch (rfcfg->mod) {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      sx127x_build_fsk_config(pv, rq);
      cfsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rfcfg);
      common = &cfsk->common;
    break;

    case DEV_RFPACKET_ASK:
      sx127x_build_ask_config(pv, rq);
      cask = const_dev_rfpacket_rf_cfg_ask_s_cast(rfcfg);
      common = &cask->common;
    break;

    default:
      return -ENOTSUP;
    break;
  }

  /* Calc time values*/
  sx127x_config_calc_time_consts(pv, common->drate);

  /* Frequency */
  sx127x_config_coeff_freq_update(pv, common, rq->channel);

  /* Datarate */
  uint8_t *rf = sx127x_config_get_cfg_buff(pv);
  *rf++ = 2;
  *rf++ = SX1276_REG_BITRATEMSB | 0x80;

  uint16_t dr = (CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO/common->drate);
  endian_be16_na_store(rf, dr);

  // Add offset to buffer
  pv->cfg_offset += 4;

  return 0;
}

static error_t sx127x_build_rf_config(struct sx127x_private_s *pv,
                                      struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  logk_trace("RF configuration");

  switch (cfg->mod) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
    case DEV_RFPACKET_ASK:
      return sx127x_build_dynamic_rf_config(pv, rq);

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_MOD_LORA
    case DEV_RFPACKET_LORA:
      return sx127x_lora_config_rf(pv, rq);
#endif

#endif

    default:
      return -ENOTSUP;
  }
}




// Packet config

static error_t sx127x_build_reg_pkt_config(struct sx127x_private_s * __restrict__ pv,
                                       struct dev_rfpacket_rq_s * __restrict__  rq)
{
  uint8_t *pk = sx127x_config_get_cfg_buff(pv);
  const struct dev_rfpacket_pk_cfg_basic_s *pk_cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

  /* Fifo threshold */

  *pk++ = 1;
  *pk++ = SX1276_REG_FIFOTHRESH | 0x80;
  *pk++ = 0x80 | 0x3F;

  pv->cfg_regs.rxcfg |= SX1276_RXCONFIG_RXTRIGER_PREAMBLEDETECT;

  /* Preamble and Sync Word config */

  if (pk_cfg->pb_pattern_len > 1)
    return -ENOTSUP;

  uint8_t sw = pk_cfg->sw_len + 1;

  /* Sync word size must be either 8 or 16 bits */
  if ((sw & 0x7) || (sw >> 5))
    return -ENOTSUP;

  sw = sw >> 3;

  uint16_t pb_len_byte = pk_cfg->tx_pb_len / 8;

  *pk++ = 3 + sw;
  *pk++ = SX1276_REG_PREAMBLEMSB | 0x80;
  *pk++ = pb_len_byte >> 8;
  *pk++ = pb_len_byte;

  uint8_t sc = (pk_cfg->pb_pattern & 2) ? SX1276_SYNCCONFIG_PREAMBLEPOLARITY_AA : SX1276_SYNCCONFIG_PREAMBLEPOLARITY_55;

  /* Store in private data for IO homecontrol mode */
  pv->cfg_regs.sync = sc | SX1276_SYNCCONFIG_SYNC_ON | (sw - 1);

  *pk++ = pv->cfg_regs.sync;

  for (int8_t i=0; i < sw; i++)
    {
      uint8_t x = pk_cfg->sw_value >> ((sw - i - 1) * 8);

      if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
        {
          x = ((x & 0x55) << 1) | ((0xAA & x) >> 1);
          x = ((x & 0x33) << 2) | ((0xCC & x) >> 2);
          x = ((x & 0x0F) << 4) | ((0xF0 & x) >> 4);
        }

      *pk++ = x;
      // Add offset to buffer
      pv->cfg_offset++;
    }

  uint8_t rx_pb_len = pk_cfg->rx_pb_len >> 3;

  /* Saturate RX preamble threshold */
  if (rx_pb_len == 0)
    rx_pb_len = 1;
  else if (rx_pb_len > 3)
    rx_pb_len = 3;

  /*  Sync word + preambule + crc + fifo filling time + 2 bytes margin */
  pv->tbrs = pv->timebit * ((68 + sw) * 8 + IO_MAX_PREAMBLE_SIZE);

  /* Time to achieve preambule detection */
  dev_timer_delay_t t = SX127X_TS_RE_US + SX127X_TS_FS_US + 100;
  dev_timer_init_sec(pv->timer, &pv->tpbrx, 0, t, 1000000);
  pv->tpbrx += pv->timebit * rx_pb_len * 8;

  *pk++ = 1;
  *pk++ = SX1276_REG_PREAMBLEDETECT | 0x80;
  *pk++ = SX1276_PREAMBLEDETECT_DETECTOR_ON | ((rx_pb_len - 1) << 5) | 0xA;

  /* Packet config mode */

  *pk++ = 3;
  *pk++ = SX1276_REG_PACKETCONFIG1 | 0x80;

  if (pk_cfg->crc_seed != 0xffff &&
      !(pk_cfg->crc_seed == 0 && !pk_cfg->crc))
    return -ENOTSUP;

  uint8_t cfg = 0;

  switch (pk_cfg->crc)
    {
    case 0:
      /* NO CRC */;
      break;
    case 0x8005:
      /* CRC16-IBM */;
      cfg = SX1276_PACKETCONFIG1_CRC_ON | SX1276_PACKETCONFIG1_CRCWHITENINGTYPE_IBM;
      break;
    case 0x1021:
      /* CRC16-CCIT */;
      cfg = SX1276_PACKETCONFIG1_CRC_ON | SX1276_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT;
      break;
    default:
      return -ENOTSUP;
    }

  cfg |= SX1276_PACKETCONFIG1_PACKETFORMAT_VARIABLE;

  switch (pk_cfg->encoding)
    {
    case DEV_RFPACKET_CLEAR:
      break;
    case DEV_RFPACKET_MANCHESTER:
      cfg |= SX1276_PACKETCONFIG1_DCFREE_MANCHESTER;
      break;
    default:
      return -ENOTSUP;
    }

  *pk++ = cfg;

  cfg = SX1276_PACKETCONFIG2_DATAMODE_PACKET | SX127X_FIFO_SIZE >> 8;

  /* Enable uart mode */

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
    cfg |= SX1276_PACKETCONFIG2_IOHOME_ON;

  *pk++ = cfg;
  *pk++ = 0xFF;

  /* DIO Mapping */
  *pk++ = 2;
  *pk++ = SX1276_REG_DIOMAPPING1 | 0x80;
  *pk++ = SX1276_DIOMAPPING1_DIO2_11;    /* Sync Address */
  *pk++ = SX1276_DIOMAPPING2_DIO4_11 |   /* Preamble detect */
          SX1276_DIOMAPPING2_DIO5_10 |   /* Data */
          SX1276_DIOMAPPING2_MAP_PREAMBLEDETECT;

  // Add offset to buffer
  pv->cfg_offset += 20;

  return 0;
}

static error_t sx127x_build_dynamic_pk_config(struct sx127x_private_s *pv,
                                            struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;

  switch (pkcfg->format) {
    case DEV_RFPACKET_FMT_IO:
    case DEV_RFPACKET_FMT_SLPC:
      sx127x_build_reg_pkt_config(pv, rq);
    break;

    default:
      return -ENOTSUP;
    break;
  }
  uint8_t *pk = sx127x_config_get_cfg_buff(pv);

  /* LNA gain and RXConfig */
  *pk++ = 2;
  *pk++ = SX1276_REG_LNA | 0x80;
  *pk++ = SX1276_LNA_BOOST_ON | SX1276_LNA_GAIN_G1;
  *pk++ = pv->cfg_regs.rxcfg;

  // Add offset to buffer
  pv->cfg_offset += 4;

  return 0;
}

static error_t sx127x_build_pk_config(struct sx127x_private_s *pv,
                                      struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_pk_cfg_s *cfg = rq->pk_cfg;

  logk_trace("PKT configuration");

  switch (cfg->format) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
    case DEV_RFPACKET_FMT_SLPC:
      return sx127x_build_dynamic_pk_config(pv, rq);

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_MOD_LORA
    case DEV_RFPACKET_FMT_LORA:
      return sx127x_lora_config_pk(pv, rq);
#endif

#endif

    default:
      return -ENOTSUP;
  }
}




// Public functions

error_t sx127x_build_config(struct sx127x_private_s * pv)
{
  error_t err = 0;
  struct dev_rfpacket_rq_s *rq = pv->gctx.rq;

  pv->cfg_offset = 0;
  pv->cfg_regs.rxcfg = SX1276_RXCONFIG_AGCAUTO_ON;

  if ((pv->flags & SX127X_FLAGS_RF_CONFIG_OK) == 0)
  {
    err = sx127x_build_rf_config(pv, rq);
    if (err != 0)
      return err;

    // Update rf cfg values
    ((struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg)->cache.dirty = 0;
    pv->rf_cfg = rq->rf_cfg;
  }
  if ((pv->flags & SX127X_FLAGS_PK_CONFIG_OK) == 0)
  {
    err = sx127x_build_pk_config(pv, rq);
    if (err != 0)
      return err;

    // Update pk cfg values
    ((struct dev_rfpacket_pk_cfg_s *)rq->pk_cfg)->cache.dirty = 0;
    pv->pk_cfg = rq->pk_cfg;
  }
  return 0;
}

bool sx127x_config_check_fairtx_valid(const struct dev_rfpacket_rf_cfg_s *rfcfg)
{
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

void sx127x_config_freq(struct sx127x_private_s *pv, uint32_t channel)
{
  sx127x_config_freq_update(pv, channel);
}

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_MOD_LORA
void sx127x_lora_inverted_iq(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

  if (rfcfg->mod != DEV_RFPACKET_LORA)
    return;

  const struct dev_rfpacket_rf_cfg_lora_s *cfg =
    const_dev_rfpacket_rf_cfg_lora_s_cast(rq->rf_cfg);

    switch (rq->type)
    {
    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
      pv->lora_cfg.iq = cfg->iq_inverted ? 0x67 : 0x27;
      break;

    case DEV_RFPACKET_RQ_TX:
      pv->lora_cfg.iq = cfg->iq_inverted ? 0x26 : 0x27;
      break;

    default:
      UNREACHABLE();
    }
}
#endif