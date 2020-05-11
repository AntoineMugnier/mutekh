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

// Const values
#if CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO > S2LP_DIV_X0_THRESH
  #define S2LP_RCO_CONF1_VAL 0x45
#else
  #define S2LP_RCO_CONF1_VAL (0x45 | S2LP_XO1_PD_CLKDIV_REGMASK) // 0x55
#endif

#ifndef CONFIG_DRIVER_RFPACKET_S2LP_LDC
  #define S2LP_RCO_CONF0_VAL 0x30
#else
  #define S2LP_RCO_CONF0_VAL (0x30 | S2LP_XO0_RCO_CALIBRATION_REGMASK) // 0x31
#endif

#define S2LP_IF_VAL 300000

#define S2LP_IF_OFFSET_ANA_VAL (uint8_t)((((uint64_t)S2LP_IF_VAL) << 13) * 3 / CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO - 100)

#if CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO > S2LP_DIV_X0_THRESH
  #define S2LP_IF_OFFSET_DIG_VAL (uint8_t)((((uint64_t)S2LP_IF_VAL) << 13) * 3 / (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO / 2) - 100)
#else
  #define S2LP_IF_OFFSET_DIG_VAL (uint8_t)((((uint64_t)S2LP_IF_VAL) << 13) * 3 / CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO - 100)
#endif

#if (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO == 25000000) || (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO == 50000000)
  #define S2LP_FREQ_RCO 33300
#elif (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO == 24000000) || (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO == 48000000)
  #define S2LP_FREQ_RCO 32000
#else
  #define S2LP_FREQ_RCO 34700
#endif

#define S2LP_IRQ_REG_3_VALUE 0x0
#define S2LP_IRQ_REG_2_VALUE 0x0
#define S2LP_IRQ_REG_1_VALUE (S2LP_IRQ_TX_FIFO_ALMOST_EMPTY | S2LP_IRQ_RX_FIFO_ALMOST_FULL)
#define S2LP_IRQ_REG_0_VALUE (S2LP_IRQ_RX_DATA_READY | S2LP_IRQ_TX_DATA_SENT | S2LP_IRQ_MAX_RE_TX_REACH | \
                              S2LP_IRQ_CRC_ERROR | S2LP_IRQ_TX_FIFO_ERROR | S2LP_IRQ_RX_FIFO_ERROR)

// Config macros and boundary values
#define S2LP_MAX_RSSI_VALUE 20 // arbitrary, in dbm

#define S2LP_MIN_PA_VALUE -31 // dbm
#define S2LP_MAX_PA_VALUE  14 // dbm

#define S2LP_VALID_TX_POWER(Power)      ((Power) >= (S2LP_MIN_PA_VALUE) && (Power) <= (S2LP_MAX_PA_VALUE))

#define S2LP_VCO_CENTER_FREQ 3600000000 // VCO center frequency in Hz
#define S2LP_DIV_X0_THRESH   30000000   // Digital domain logic threshold for XTAL in Hz
#define SLP_REF_DIV_VAL      1

#define S2LP_HIGH_BAND_FACTOR        4 // Band select factor for high band. Factor B in the equation 2
#define S2LP_MIDDLE_BAND_FACTOR      8 // Band select factor for middle band. Factor B in the equation 2

#define S2LP_HIGH_BAND_LOWER_LIMIT   825900000   // Lower limit of the high band: 860 MHz (S2-LPQTR)
#define S2LP_HIGH_BAND_UPPER_LIMIT   1056000000  // Upper limit of the high band: 940 MHz (S2-LPCBQTR)
#define S2LP_MIDDLE_BAND_LOWER_LIMIT 412900000   // Lower limit of the middle band: 430 MHz (S2-LPQTR)
#define S2LP_MIDDLE_BAND_UPPER_LIMIT 527100000   // Upper limit of the middle band: 470 MHz (S2-LPCBQTR)

#define S2LP_FREQ_BAND_HIGH(Freq) ((Freq) >= S2LP_HIGH_BAND_LOWER_LIMIT && \
                                           (Freq) <= S2LP_HIGH_BAND_UPPER_LIMIT)

#define S2LP_FREQ_BAND_MIDDLE(Freq) ((Freq) >= S2LP_MIDDLE_BAND_LOWER_LIMIT && \
                                             (Freq) <= S2LP_MIDDLE_BAND_UPPER_LIMIT)

#define S2LP_VALID_FREQ(Freq) (S2LP_FREQ_BAND_HIGH(Freq) || S2LP_FREQ_BAND_MIDDLE(Freq))

#define S2LP_CH_BW_LOWER_LIMIT(F_clk) (((uint64_t)1100 * F_clk / 1000000) / 26)   // Minimum value of the channel filter bandwidth
#define S2LP_CH_BW_UPPER_LIMIT(F_clk) (((uint64_t)800100 * F_clk / 1000000) / 26) // Maximum value of the channel filter bandwidth

#define S2LP_VALID_CH_BW(Bw, F_Xo)   ((Bw) >= S2LP_CH_BW_LOWER_LIMIT(F_Xo) && (Bw) <= S2LP_CH_BW_UPPER_LIMIT(F_Xo))

#define S2LP_MINIMUM_DATARATE 100    // Minimum datarate supported by S2LP 100 bps
#define S2LP_MAXIMUM_DATARATE 250000 // Maximum datarate supported by S2LP 250 ksps

#define S2LP_F_DEV_LOWER_LIMIT(F_Xo)   (F_Xo >> 22)
#define S2LP_F_DEV_UPPER_LIMIT(F_Xo)   (((uint64_t)787109 * F_Xo / 1000000) / 26)

#define S2LP_VALID_FREQ_DEV(F_dev, F_Xo) (F_dev >= S2LP_F_DEV_LOWER_LIMIT(F_Xo) && \
                                                F_dev <= S2LP_F_DEV_UPPER_LIMIT(F_Xo))
/*
* It represents the available channel bandwidth times 10 for 26 Mhz xtal.
* The channel bandwidth for others xtal frequencies can be computed since this table
* multiplying the current table by a factor xtal_frequency/26e6.
*/
static const uint16_t s2lp_chan_bw_table[90] = {
  8001, 7951, 7684, 7368, 7051, 6709, 6423, 5867, 5414, \
    4509, 4259, 4032, 3808, 3621, 3417, 3254, 2945, 2703, \
      2247, 2124, 2015, 1900, 1807, 1706, 1624, 1471, 1350, \
        1123, 1062, 1005,  950,  903,  853,  812,  735,  675, \
          561,  530,  502,  474,  451,  426,  406,  367,  337, \
            280,  265,  251,  237,  226,  213,  203,  184,  169, \
              140,  133,  126,  119,  113,  106,  101,   92,   84, \
                70,   66,   63,   59,   56,   53,   51,   46,   42, \
                  35,   33,   31,   30,   28,   27,   25,   23,   21, \
                    18,   17,   16,   15,   14,   13,   13,   12,   11
};

// Private functions prototypes
static void s2lp_config_calc_time_consts(struct s2lp_ctx_s *pv, uint32_t drate);
static error_t s2lp_build_rf_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static error_t s2lp_build_pk_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq);


// Private functions
static void s2lp_config_calc_time_consts(struct s2lp_ctx_s *pv, uint32_t drate) {
    // Calc time byte in us
  dev_timer_delay_t tb = 8000000 / drate;
  dev_timer_init_sec(pv->timer, &(pv->gctx.time_byte), 0, tb, 1000000);
  // Calc other time constants
  pv->mpst = 2 * (S2LP_FIFO_SIZE - S2LP_FIFO_THRESHOLD) * pv->gctx.time_byte + pv->bt;
  pv->ccad = 2 * 8 * pv->gctx.time_byte + pv->bt;
}

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG

static uint32_t s2lp_calc_datarate(uint32_t freq_xo, uint16_t cM, uint8_t cE) {
  uint64_t dr;

  if(cE == 0) {
    dr = ((uint64_t)freq_xo * cM);
    return (uint32_t)(dr >> 32);
  }
  dr = ((uint64_t)freq_xo) * ((uint64_t)cM + 65536);
  return (uint32_t)(dr >> (33 - cE));
}

static void s2lp_find_datarate_params(uint32_t freq_xo, uint32_t datarate, uint16_t* pcM, uint8_t* pcE) {
  uint32_t datarateTmp;
  uint8_t uDrE;
  uint64_t tgt1, tgt2, tgt;

  if (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO > S2LP_DIV_X0_THRESH) {
    freq_xo /= 2;
  }
  // Search exponent value
  for (uDrE = 0; uDrE < 12; uDrE++) {
    datarateTmp = s2lp_calc_datarate(freq_xo, 0xFFFF, uDrE);
    if (datarate <= datarateTmp) {
      break;
    }
  }
  (*pcE) = (uint8_t)uDrE;
  if (uDrE == 0) {
    tgt = ((uint64_t)datarate) << 32;
    (*pcM) = (uint16_t)(tgt / freq_xo);
    tgt1 = (uint64_t)freq_xo * (*pcM);
    tgt2 = (uint64_t)freq_xo * ((*pcM) + 1);
  } else {
    tgt = ((uint64_t)datarate) << (33 - uDrE);
    (*pcM) = (uint16_t)((tgt / freq_xo) - 65536);
    tgt1 = (uint64_t)freq_xo * ((*pcM) + 65536);
    tgt2 = (uint64_t)freq_xo * ((*pcM) + 1 + 65536);
  }
  (*pcM) = ((tgt2 - tgt) < (tgt - tgt1)) ? ((*pcM) + 1) : (*pcM);
}

static uint32_t s2lp_calc_freqdev(uint32_t freq_xo, uint8_t cM, uint8_t cE) {
  if (cE == 0) {
    return (uint32_t)(((uint64_t)freq_xo * cM) >> 22);
  }
  return (uint32_t)(((uint64_t)freq_xo * (256 + cM)) >> (23 - cE));
}

static void s2lp_find_freqdev_params(uint32_t freq_xo, uint32_t freqdev, uint8_t* pcM, uint8_t* pcE) {
  uint8_t uFDevE;
  uint32_t freqdevTmp;
  uint64_t tgt1, tgt2, tgt;

  // Search the exponent of the frequency deviation value
  for (uFDevE = 0; uFDevE < 12; uFDevE++) {
    freqdevTmp = s2lp_calc_freqdev(freq_xo, 255, uFDevE);
    if (freqdev < freqdevTmp) {
      break;
    }
  }
  (*pcE) = (uint8_t)uFDevE;

  if (uFDevE == 0) {
    tgt = ((uint64_t)freqdev) << 22;
    (*pcM) = (uint32_t)(tgt / freq_xo);
    tgt1 = (uint64_t)freq_xo * (*pcM);
    tgt2 = (uint64_t)freq_xo * ((*pcM) + 1);
  } else {
    tgt = ((uint64_t)freqdev) << (23 - uFDevE);
    (*pcM) = (uint32_t)(tgt / freq_xo) - 256;
    tgt1 = (uint64_t)freq_xo * ((*pcM) + 256);
    tgt2 = (uint64_t)freq_xo * ((*pcM) + 1 + 256);
  }
  (*pcM) = ((tgt2 - tgt) < (tgt - tgt1)) ? ((*pcM) + 1) : (*pcM);
}

static uint8_t s2lp_calc_chan_spacing(uint32_t freq_xo, uint32_t channel_space) {
  return (uint32_t)(((uint64_t)channel_space) << 15) / freq_xo;
}

static void s2lp_find_channel_params(uint32_t freq_xo, uint32_t chan_bw, uint8_t* pcM, uint8_t* pcE) {
  int8_t i, i_tmp;
  int32_t chfltCalculation[3];

  if (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO > S2LP_DIV_X0_THRESH) {
    freq_xo /= 2;
  }
  // Search the channel filter bandwidth table index
  for (i = 0; i < 90; i++) {
    if (chan_bw >= (uint32_t)(((uint64_t)s2lp_chan_bw_table[i] * freq_xo) / 260000)) {
      break;
    }
  }
  if (i != 0) {
    // Finds the index value with best approximation in i-1, i and i+1 elements
    i_tmp = i;

    for (uint8_t j = 0; j < 3; j++) {
      if (((i_tmp + j - 1) >= 0) && ((i_tmp + j - 1) <= 89)) {
        chfltCalculation[j] =
          (int32_t)chan_bw - (int32_t)(((uint64_t)s2lp_chan_bw_table[i_tmp + j - 1] * freq_xo) / 260000);
      } else {
        chfltCalculation[j] = 0x7FFFFFFF;
      }
    }
    uint32_t chfltDelta = 0xFFFFFFFF;

    for (uint8_t j = 0; j < 3; j++) {
      if (__ABS(chfltCalculation[j]) < chfltDelta) {
        chfltDelta = __ABS(chfltCalculation[j]);
        i = i_tmp + j - 1;
      }
    }
  }
  (*pcE) = (uint8_t)(i / 9);
  (*pcM) = (uint8_t)(i % 9);
}

static uint32_t s2lp_calc_synth(uint32_t freq_xo, uint32_t frequency, uint8_t refdiv) {
  uint8_t band;

  if (S2LP_FREQ_BAND_HIGH(frequency)) {
    band = S2LP_HIGH_BAND_FACTOR;
  } else {
    band = S2LP_MIDDLE_BAND_FACTOR;
  }
  uint64_t tgt1,tgt2,tgt;
  uint32_t synth;

  tgt = (((uint64_t)frequency) << 19) * (band * refdiv);
  synth = (uint32_t)(tgt / freq_xo);
  tgt1 = (uint64_t)freq_xo * (synth);
  tgt2 = (uint64_t)freq_xo * (synth + 1);
  synth = ((tgt2 - tgt) < (tgt - tgt1)) ? (synth + 1) : (synth);
  return synth;
}

static void s2lp_find_charge_pump_params(uint32_t freq_xo, uint32_t freq, uint8_t refdiv, uint8_t* cp_isel, uint8_t* pfd_split) {
  uint32_t vcofreq, lFRef;
  uint8_t band;

  if (S2LP_FREQ_BAND_HIGH(freq)) {
    band = S2LP_HIGH_BAND_FACTOR;
  } else {
    band = S2LP_MIDDLE_BAND_FACTOR;
  }
  // Calculate VCO frequency
  vcofreq = freq * band;
  // Calculate reference frequency clock
  lFRef = freq_xo / refdiv;
  // Set correct charge pump word
  if (vcofreq >= S2LP_VCO_CENTER_FREQ) {
    if (lFRef > S2LP_DIV_X0_THRESH) {
      *cp_isel = 0x02;
      *pfd_split = 0;
    } else {
      *cp_isel = 0x01;
      *pfd_split = 1;
    }
  } else {
    if (lFRef > S2LP_DIV_X0_THRESH) {
      *cp_isel = 0x03;
      *pfd_split = 0;
    } else {
      *cp_isel = 0x02;
      *pfd_split = 1;
    }
  }
}

static error_t s2lp_build_dynamic_rf_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  // Retrieve data struct
  const struct dev_rfpacket_rf_cfg_fairtx_s *fairtx = NULL;
  const struct dev_rfpacket_rf_cfg_std_s *common = NULL;
  const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = NULL;
  const struct dev_rfpacket_rf_cfg_ask_s *cask = NULL;

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

    default:
      return -ENOTSUP;
    break;
  }
  // Calculate time values
  s2lp_config_calc_time_consts(pv, common->drate);

  // Configure xo dig div
  uint32_t freq_xo = CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO;
  uint8_t *pXoConf1 = &pv->rf_cfg_array[32];

  if (freq_xo > S2LP_DIV_X0_THRESH) {
    *pXoConf1 |= S2LP_XO1_PD_CLKDIV_REGMASK;
  }

  // Configure RSSI_threshold
  int16_t rssi_th = 0;
  int16_t jam_rssi = common->jam_rssi >> 3;
  uint8_t *pRssiThr = &pv->rf_cfg_array[3];

  if (fairtx && (fairtx->mode == DEV_RFPACKET_LBT)) {
    rssi_th = fairtx->lbt.rssi >> 3;
  } else {
    rssi_th = jam_rssi;
  }
  if ((rssi_th > S2LP_MAX_RSSI_VALUE) || (jam_rssi > S2LP_MAX_RSSI_VALUE)) {
    logk_trace("RSSI value too big");
    return -ENOTSUP;
  }
  pv->lbt_rssi = S2LP_SET_RSSI(rssi_th);
  pv->jam_rssi = S2LP_SET_RSSI(jam_rssi);
  *pRssiThr = S2LP_SET_RSSI(rssi_th);

  // Configure data rate
  if ((common->drate < S2LP_MINIMUM_DATARATE) || (common->drate > S2LP_MAXIMUM_DATARATE)) {
    logk_trace("Datarate is out of bounds");
    return -ENOTSUP;
  }
  uint8_t *pDatarateMantissaMsb = &pv->rf_cfg_array[9];
  uint8_t *pDatarateMantissaLsb = &pv->rf_cfg_array[10];
  uint8_t *pDatarateExponent = &pv->rf_cfg_array[11];
  uint16_t dr_mantissa;
  uint8_t dr_exponent;

  s2lp_find_datarate_params(freq_xo, common->drate, &dr_mantissa, &dr_exponent);
  *pDatarateMantissaMsb = (uint8_t)(dr_mantissa >> 8);
  *pDatarateMantissaLsb = (uint8_t)dr_mantissa;
  *pDatarateExponent &= ~S2LP_MOD2_DATARATE_E_REGMASK;
  *pDatarateExponent |= dr_exponent;

  // Configure modulation
  uint8_t *pModType = &pv->rf_cfg_array[11];

  switch (cfg->mod) {
    case DEV_RFPACKET_GFSK:
      switch (cfsk->symbols) {
        case 2:
          *pModType &= ~S2LP_MOD2_MOD_TYPE_REGMASK;
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_GFSK_BT_FILTER_1
          *pModType |= S2LP_MOD_2GFSK_BT1;
#else
          *pModType |= S2LP_MOD_2GFSK_BT05;
#endif
        break;

        case 4:
          *pModType &= ~S2LP_MOD2_MOD_TYPE_REGMASK;
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_GFSK_BT_FILTER_1
          *pModType |= S2LP_MOD_4GFSK_BT1;
#else
          *pModType |= S2LP_MOD_4GFSK_BT05;
#endif
        break;

        default:
          logk_trace("Unsupported symbol number");
          return -ENOTSUP;
        break;
      }
    break;

    case DEV_RFPACKET_FSK:
      switch (cfsk->symbols) {
        case 2:
          *pModType &= ~S2LP_MOD2_MOD_TYPE_REGMASK;
          *pModType |= S2LP_MOD_2FSK;
        break;

        case 4:
          *pModType &= ~S2LP_MOD2_MOD_TYPE_REGMASK;
          *pModType |= S2LP_MOD_4FSK;
        break;

        default:
          logk_trace("Unsupported symbol number");
          return -ENOTSUP;
        break;
      }
    break;

    case DEV_RFPACKET_ASK:
      switch (cask->symbols) {
        case 2:
          *pModType &= ~S2LP_MOD2_MOD_TYPE_REGMASK;
          *pModType |= S2LP_MOD_ASK_OOK;
        break;

        case 4:
        case 8:
        default:
          logk_trace("Unsupported symbol number");
          return -ENOTSUP;
        break;
      }
    break;

    default:
      logk_trace("Unsupported modulation");
      return -ENOTSUP;
    break;
  }

  // Configure frequency deviation
  if (cfsk != NULL) {
    uint8_t *pFdevExponent = &pv->rf_cfg_array[12];
    uint8_t *pFdevMantissa = &pv->rf_cfg_array[13];
    uint8_t fdev_mantissa, fdev_exponent;
    if (!S2LP_VALID_FREQ_DEV(cfsk->deviation, CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO)) {
      logk_trace("Freq deviation out of bounds");
      return -ENOTSUP;
    }
    s2lp_find_freqdev_params(freq_xo, cfsk->deviation, &fdev_mantissa, &fdev_exponent);
    *pFdevExponent &= ~S2LP_MOD1_FDEV_E_REGMASK;
    *pFdevExponent |= fdev_exponent;
    *pFdevMantissa = fdev_mantissa;
  }
  // Configure channel spacing
  uint8_t *pChanFilter = &pv->rf_cfg_array[14];
  uint8_t *pChanSpacing = &pv->rf_cfg_array[7];
  uint8_t chan_mantissa, chan_exponent;

  if (!S2LP_VALID_CH_BW(common->chan_spacing, freq_xo)) {
    logk_trace("Chan spacing out of bounds");
    return -ENOTSUP;
  }
  s2lp_find_channel_params(freq_xo, common->chan_spacing, &chan_mantissa, &chan_exponent);
  *pChanFilter = (chan_mantissa << 4) | chan_exponent;
  *pChanSpacing = s2lp_calc_chan_spacing(freq_xo, common->chan_spacing);

  // Configure power
  uint8_t *pPapow0 = &pv->rf_cfg_array[18];
  uint8_t *pPacfg1 = &pv->rf_cfg_array[19];
  uint8_t *pPacfg0 = &pv->rf_cfg_array[20];

  switch (cfg->mod) {
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
      *pPapow0 &= ~S2LP_DIG_SMOOTH_EN_REGMASK;
      *pPacfg1 &= ~S2LP_FIR_EN_REGMASK;
    default:
    break;

    case DEV_RFPACKET_ASK:
      *pPapow0 |= S2LP_DIG_SMOOTH_EN_REGMASK;
      *pPacfg1 |= S2LP_FIR_EN_REGMASK;
    break;
  }
  *pPapow0 &= ~S2LP_PA_MAXDBM_REGMASK;

  // Configure power filter
  *pPacfg0 &= ~S2LP_PA_FC_REGMASK;

  if (common->drate < 16000) {
    *pPacfg0 |= 0x00;
  } else if (common->drate < 32000) {
    *pPacfg0 |= 0x01;
  } else if (common->drate < 62500) {
    *pPacfg0 |= 0x02;
  } else {
    *pPacfg0 |= 0x03;
  }

  // Configure frequency
  if (!S2LP_VALID_FREQ(common->frequency)) {
    logk_trace("Frequency out of bounds");
    return -ENOTSUP;
  }
  uint32_t synth = s2lp_calc_synth(freq_xo, common->frequency, SLP_REF_DIV_VAL);
  uint8_t cp_isel, pfd_split, bs;
  uint8_t *pSynt3 = &pv->rf_cfg_array[25];
  uint8_t *pSynt2 = &pv->rf_cfg_array[26];
  uint8_t *pSynt1 = &pv->rf_cfg_array[27];
  uint8_t *pSynt0 = &pv->rf_cfg_array[28];
  uint8_t *pSyntCfg2 = &pv->rf_cfg_array[21];
  s2lp_find_charge_pump_params(freq_xo, common->frequency, SLP_REF_DIV_VAL, &cp_isel, &pfd_split);

  if (S2LP_FREQ_BAND_HIGH(common->frequency)) {
    bs = 0;
  } else {
    bs = 1;
  }

  *pSynt3 = (((uint8_t)(synth >> 24)) & S2LP_SYNT_27_24_REGMASK) | (cp_isel << 5) | (bs << 4);
  *pSynt2 = (uint8_t)(synth >> 16);
  *pSynt1 = (uint8_t)(synth >> 8);
  *pSynt0 = (uint8_t)synth;
  *pSyntCfg2 &= ~S2LP_PLL_PFD_SPLIT_EN_REGMASK;
  *pSyntCfg2 |= (pfd_split << 2);

  //printk("Rf config array: %P\n", pv->rf_cfg_array, S2LP_RF_CFG_ARRAY_SIZE);

  // Set current rf_cfg
  pv->curr_rf_cfg_data = pv->rf_cfg_array;
  pv->curr_rf_cfg_size = S2LP_RF_CFG_DATA_SIZE;
  return 0;
}
#endif



#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG

#ifdef CONFIG_DRIVER_RFPACKET_S2LP_LDC

static void s2lp_find_wut_params(uint32_t timeval, uint8_t* pCount , uint8_t* pPresc, uint8_t* pMult) {
  uint32_t n;
  uint8_t i;

  // Calc multiplier value
  for(i = 0; i < 4; i++) {
    if(timeval < (((uint32_t)((uint64_t)1000000 * 65536 / S2LP_FREQ_RCO)) << i)) {
      break;
    }
  }
  (*pMult) = i;
  /* N cycles in the time base of the timer:
     - clock of the timer is RCO frequency
     - divide times 1000000 more because we have an input in us
  */
  n = (uint32_t)((uint64_t)timeval * (S2LP_FREQ_RCO >> i) / 1000000);

  // Check if possible to reach that target with prescaler and counter of S2LP
  if(n / 0xFF > 0xFD) {
    // If not return the maximum possible value
    (*pCount) = 0xFF;
    (*pPresc) = 0xFF;
    return;
  }
  // Prescaler is really 2 as min value
  (*pPresc) = (n / 0xFF) + 2;
  (*pCount) = n / (*pPresc);
  // Decrement prescaler and counter according to the logic of this timer in S2LP
  (*pPresc)--;
  if((*pCount) > 1) {
    (*pCount)--;
  } else {
    (*pCount) = 1;
  }
}

static void s2lp_find_rxt_params(uint32_t freq_xo, uint32_t timeval , uint8_t* pCount , uint8_t* pPresc) {
  uint32_t n;
  uint64_t tgt,tgt1,tgt2;

  if (CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO > S2LP_DIV_X0_THRESH) {
    freq_xo /= 2;
  }

  /* N cycles in the time base of the timer:
     - clock of the timer is freq_xo/1210
     - divide times 1000000 more because we have an input in us
  */
  tgt = (uint64_t)timeval * freq_xo;
  n = (uint32_t)(tgt / 1210000000);
  tgt1 = (uint64_t)1210000000 * n;
  tgt2 = (uint64_t)1210000000 * (n + 1);

  n = ((tgt2 - tgt) < (tgt - tgt1)) ? (n + 1) : (n);

  // Check if possible to reach that target with prescaler and counter of S2LP
  if(n / 0xFF > 0xFD) {
    // If not return the maximum possible value
    (*pCount) = 0xFF;
    (*pPresc) = 0xFF;
    return;
  }
  // Prescaler is really 2 as min value
  (*pPresc) = (n / 0xFF) + 2;
  (*pCount) = n / (*pPresc);
  // Decrement prescaler and counter according to the logic of this timer in S2LP
  (*pPresc)--;
  if((*pCount) == 0) {
    (*pCount) = 1;
  }
}
#endif

static error_t s2lp_build_dynamic_pk_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
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

  // Configure encoding
  uint8_t *pPcktCtrl2 = &pv->pk_cfg_array[7];

  switch (cbasic->encoding) {
    default:
    case DEV_RFPACKET_CLEAR:
      *pPcktCtrl2 &= ~S2LP_PCKTCTRL2_MANCHESTER_EN_REGMASK;
    break;

    case DEV_RFPACKET_MANCHESTER:
      *pPcktCtrl2 |= S2LP_PCKTCTRL2_MANCHESTER_EN_REGMASK;
    break;
  }
  // Activate packet variable length
  *pPcktCtrl2 |= S2LP_PCKTCTRL2_FIX_VAR_LEN_REGMASK;

  // Configure Sync Word
  uint8_t *pSync_len = &pv->pk_cfg_array[3];
  uint8_t sync_len = cbasic->sw_len + 1;
  uint8_t sync_byte_nb = (sync_len % 8 == 0) ? sync_len / 8 : sync_len / 8 + 1;

  if (sync_byte_nb > 4) {
    logk_trace("Sync word too big");
    return -ENOTSUP;
  }
  // Set sync length
  *pSync_len &= ~S2LP_PCKTCTRL6_SYNC_LEN_REGMASK;
  *pSync_len |= S2LP_PCKTCTRL6_SYNC_LEN_OFFSET(sync_len);
  // Set sync value
  for (uint8_t idx = 0; idx < sync_byte_nb; idx++) {
    // Get sync byte address (sync0 first)
    uint8_t *pSync = &pv->pk_cfg_array[15 - idx];
    // Set sync byte (msb first)
    *pSync = (uint8_t)(cbasic->sw_value >> 8 * (sync_byte_nb - (idx + 1)));
  }

  // Configure Preamble pattern
  uint8_t *pPbMode = &pv->pk_cfg_array[6];

  if (cbasic->pb_pattern_len > 0x3) {
    logk_trace("Preamble pattern too big");
    return -ENOTSUP;
  }
  switch (cbasic->pb_pattern) {
    case 0x1:
    case 0x5:
    case 0x7:
      *pPbMode &= ~S2LP_PCKTCTRL3_PREAMBLE_SEL_REGMASK;
      *pPbMode |= S2LP_PKT_PREAMBLE_MODE_1;
    break;

    case 0x2:
    case 0xa:
      *pPbMode &= ~S2LP_PCKTCTRL3_PREAMBLE_SEL_REGMASK;
      *pPbMode |= S2LP_PKT_PREAMBLE_MODE_2;
    break;

    case 0x3:
    case 0x8:
      *pPbMode &= ~S2LP_PCKTCTRL3_PREAMBLE_SEL_REGMASK;
      *pPbMode |= S2LP_PKT_PREAMBLE_MODE_4;
    break;

    case 0xc:
    case 0xd:
      *pPbMode &= ~S2LP_PCKTCTRL3_PREAMBLE_SEL_REGMASK;
      *pPbMode |= S2LP_PKT_PREAMBLE_MODE_3;
    break;

    default:
      logk_trace("Unsupported preamble pattern");
      return -ENOTSUP;
    break;
  }

  // Configure preamble length
  uint8_t *pPbLen_msb = &pv->pk_cfg_array[3];
  uint8_t *pPbLen_lsb = &pv->pk_cfg_array[4];

  if ((cbasic->tx_pb_len > 2046) || (cbasic->tx_pb_len % 2 == 1)) {
    logk_trace("Preamble too big or odd");
    return -ENOTSUP;
  }
  // Store preamble length as pair number
  uint16_t pb_pair_nb = cbasic->tx_pb_len / 2;
  // Clear msb
  *pPbLen_msb &= ~S2LP_PCKTCTRL6_PREAMBLE_LEN_REGMASK;
  // Note msb only if needed
  if (pb_pair_nb > 255) {
    *pPbLen_msb |= pb_pair_nb >> 8;
  }
  *pPbLen_lsb = (uint8_t)pb_pair_nb;

  // Configure CRC
  uint8_t *pCrcMode = &pv->pk_cfg_array[8];

  switch (cbasic->crc_seed) {
    case 0xff:
    case 0xffff:
    case 0xffffffff:
    break;

    default:
      logk_trace("Bad crc seed, only 1's supported");
      return -ENOTSUP;
    break;
  }
  switch (cbasic->crc) {
    case 0:
      *pCrcMode &= ~S2LP_PCKTCTRL1_CRC_MODE_REGMASK;
    break;

    case 0x07:
      *pCrcMode &= ~S2LP_PCKTCTRL1_CRC_MODE_REGMASK;
      *pCrcMode |= S2LP_PKT_CRC_MODE_8BITS;
    break;

    case 0x8005:
      *pCrcMode &= ~S2LP_PCKTCTRL1_CRC_MODE_REGMASK;
      *pCrcMode |= S2LP_PKT_CRC_MODE_16BITS_1;
    break;

    case 0x1021:
      *pCrcMode &= ~S2LP_PCKTCTRL1_CRC_MODE_REGMASK;
      *pCrcMode |= S2LP_PKT_CRC_MODE_16BITS_2;
    break;

    case 0x864CFB:
      *pCrcMode &= ~S2LP_PCKTCTRL1_CRC_MODE_REGMASK;
      *pCrcMode |= S2LP_PKT_CRC_MODE_24BITS;
    break;

    case 0x04C011BB7:
      *pCrcMode &= ~S2LP_PCKTCTRL1_CRC_MODE_REGMASK;
      *pCrcMode |= S2LP_PKT_CRC_MODE_32BITS;
    break;

    default:
      logk_trace("Unsupported CRC");
      return -ENOTSUP;
    break;
  }
  // Set csma persistant mode
  uint8_t *pProt2 = &pv->pk_cfg_array[19];
  uint8_t *pProt1 = &pv->pk_cfg_array[20];
  *pProt1 |= S2LP_PROTOCOL1_CSMA_PERS_ON_REGMASK;

#ifdef CONFIG_DRIVER_RFPACKET_S2LP_LDC
  // Set LDC timer parameters
  uint8_t mult;
  uint8_t *pRxtCount = &pv->pk_cfg_array[25];
  uint8_t *pRxtPresc = &pv->pk_cfg_array[26];
  uint8_t *pWutPresc = &pv->pk_cfg_array[27];
  uint8_t *pWutCount = &pv->pk_cfg_array[28];

  // TODO SET LDC CALC FUNCTIONS
  //  Twut < (tx_pb_len - 8) * Tbit - 0.1ms
  // dev_timer_delay_t time_byte = 8000000 / rq->rf_cfg->drate;
  // uint32_t max_twut = (cbasic->tx_pb_len / 8 - 1) * time_byte;
  // max_twut = (max_twut > 100) ? (max_twut - 100) : (max_twut);
  // uint32_t max_trxt = (cbasic->tx_pb_len + cbasic->sw_len + 1) / 8 * time_byte;

  // printk("Maw Twut = %d, Max Trxt = %d\n", max_twut, max_trxt);

  s2lp_find_wut_params(S2LP_LDC_WUT_US, pWutCount, pWutPresc, &mult);

  *pProt2 |= (mult & S2LP_PROTOCOL2_LDC_TIMER_MULT_REGMASK);

  s2lp_find_rxt_params(CONFIG_DRIVER_RFPACKET_S2LP_FREQ_XO, S2LP_LDC_RXT_US, pRxtCount, pRxtPresc);
#endif

  //printk("Pk config array: %P\n", pv->pk_cfg_array, S2LP_PK_CFG_ARRAY_SIZE);
  //printk("protocol regs: %d, %d\n", *pProt1, *pProt2);

  // Set current protocol values
  pv->curr_prot1 = *pProt1;
  pv->curr_prot2 = *pProt2;
  // Set current pk_config
  pv->curr_pk_cfg_data = pv->pk_cfg_array;
  pv->curr_pk_cfg_size = S2LP_PK_CFG_DATA_SIZE;
  return 0;
}
#endif

static error_t s2lp_build_static_rf_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_static_s *cstatic = const_dev_rfpacket_rf_cfg_static_s_cast(rq->rf_cfg);
  struct s2lp_rf_cfg_s *cfg = NULL;

  // Retrieve config
  error_t err = device_get_param_blob(pv->dev, cstatic->cfg_name, 0, (const void **)&cfg);
  if (err != 0) {
    return err;
  }
  assert(cfg);
  // Calc time constants
  s2lp_config_calc_time_consts(pv, cfg->drate);
  // Note info
  //printk("RF CONFIG: %d, %d, %d, %d, %P\n", cfg->drate, cfg->jam_rssi, cfg->lbt_rssi, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->jam_rssi = cfg->jam_rssi;
  pv->lbt_rssi = cfg->lbt_rssi;
  pv->curr_rf_cfg_size = cfg->config_size;
  pv->curr_rf_cfg_data = cfg->config_data;
  return 0;
}

static error_t s2lp_build_extern_rf_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_extern_s *cextern = const_dev_rfpacket_rf_cfg_extern_s_cast(rq->rf_cfg);

  // Retrieve config
  struct s2lp_rf_cfg_s *cfg = cextern->p_cfg;
  assert(cfg);
  // Calc time constants
  s2lp_config_calc_time_consts(pv, cfg->drate);
  // Note info
  //printk("RF CONFIG: %d, %d, %d, %d, %P\n", cfg->drate, cfg->jam_rssi, cfg->lbt_rssi, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->jam_rssi = cfg->jam_rssi;
  pv->lbt_rssi = cfg->lbt_rssi;
  pv->curr_rf_cfg_size = cfg->config_size;
  pv->curr_rf_cfg_data = cfg->config_data;
  return 0;
}

static error_t s2lp_build_static_pk_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_static_s *cstatic = const_dev_rfpacket_pk_cfg_static_s_cast(rq->pk_cfg);
  struct s2lp_pk_cfg_s *cfg = NULL;

  // Retrieve config
  error_t err = device_get_param_blob(pv->dev, cstatic->cfg_name, 0, (const void **)&cfg);
  if (err != 0) {
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

static error_t s2lp_build_extern_pk_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_extern_s *cextern = const_dev_rfpacket_pk_cfg_extern_s_cast(rq->pk_cfg);

  // Retrieve config
  struct s2lp_pk_cfg_s *cfg = cextern->p_cfg;
  assert(cfg);
  // Note info
  //printk("PK CONFIG: 0x%02x, 0x%02x, %d, %P\n", cfg->prot1, cfg->prot2, cfg->config_size, cfg->config_data, cfg->config_size);
  pv->curr_prot1 = cfg->prot1;
  pv->curr_prot2 = cfg->prot2;
  pv->curr_pk_cfg_size = cfg->config_size;
  pv->curr_pk_cfg_data = cfg->config_data;
  return 0;
}

static error_t s2lp_build_rf_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_rf_cfg_s *cfg = rq->rf_cfg;

  logk_trace("RF configuration");

  switch (cfg->mod) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
    case DEV_RFPACKET_GFSK:
    case DEV_RFPACKET_FSK:
    case DEV_RFPACKET_ASK:
      return s2lp_build_dynamic_rf_config(pv, rq);
#endif

    case DEV_RFPACKET_MOD_STATIC:
      return s2lp_build_static_rf_config(pv, rq);

    case DEV_RFPACKET_MOD_EXTERN:
      return s2lp_build_extern_rf_config(pv, rq);

    default:
      return -ENOTSUP;    
  }
}

static error_t s2lp_build_pk_config(struct s2lp_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  const struct dev_rfpacket_pk_cfg_s *cfg = rq->pk_cfg;

  logk_trace("PKT configuration");

  switch (cfg->format) {
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
    case DEV_RFPACKET_FMT_SLPC:
      return s2lp_build_dynamic_pk_config(pv, rq);
#endif

    case DEV_RFPACKET_FMT_STATIC:
      return s2lp_build_static_pk_config(pv, rq);

    case DEV_RFPACKET_FMT_EXTERN:
      return s2lp_build_extern_pk_config(pv, rq);

    default:
      return -ENOTSUP;
  }
}


// Public functions

error_t s2lp_build_config(struct s2lp_ctx_s *pv) {
  error_t err = 0;
  struct dev_rfpacket_rq_s *rq = pv->gctx.rq;

  if ((pv->flags & S2LP_FLAGS_RF_CONFIG_OK) == 0) {
    err = s2lp_build_rf_config(pv, rq);
    if (err != 0) {
      // Clear rf cfg values
      pv->rf_cfg = NULL;
      pv->curr_rf_cfg_data = NULL;
      pv->curr_rf_cfg_size = 0;
      return err;
    }
  }
  if ((pv->flags & S2LP_FLAGS_PK_CONFIG_OK) == 0) {
    err = s2lp_build_pk_config(pv, rq);
    if (err != 0) {
      // Clear pk  cfg values
      pv->pk_cfg = NULL;
      pv->curr_pk_cfg_data = NULL;
      pv->curr_pk_cfg_size = 0;
      return err;
    }
  }
  return 0;
}

uint8_t s2lp_build_pwr(struct s2lp_ctx_s *pv, int16_t pwr) {
  int16_t pwr_dbm = pwr >> 3;
  uint8_t reg_pwr = 0;

  // Check if power valid
  if (!S2LP_VALID_TX_POWER(pwr_dbm)) {
    logk_trace("Unsupported power level requested");
    return 0;
  }
  // Check if power need to be configured
  pv->flags &= ~S2LP_FLAGS_TX_POWER_OK;

  if (pv->pwr == pwr) {
    pv->flags |= S2LP_FLAGS_TX_POWER_OK;
    return 0;
  }
  // Set new power value
  pv->pwr = pwr;
  // Calc power reg value
  if (pwr_dbm >= 14) {
    reg_pwr = 1;
  } else {
    reg_pwr = (uint8_t)((int32_t)29 - 2 * pwr_dbm);
  }
  return reg_pwr;
}

bool s2lp_config_check_fairtx_valid(const struct dev_rfpacket_rf_cfg_s *rfcfg) {
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

uint32_t s2lp_config_get_freq(const struct dev_rfpacket_rf_cfg_s *cfg, uint8_t chan) {
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

void s2lp_init_rf_cfg_array(uint8_t *pArray, uint16_t array_size) {
  // Check size
  assert(array_size == S2LP_RF_CFG_ARRAY_SIZE);
  // Reset memory
  memset(pArray, 0, array_size);
  // Header Rssi th
  pArray[0] = 0x03;
  pArray[1] = S2LP_WRITE_REG_BYTE;
  pArray[2] = S2LP_RSSI_TH_ADDR;
  // Header modulation / channel filter
  pArray[4] = 0x0a;
  pArray[5] = S2LP_WRITE_REG_BYTE;
  pArray[6] = S2LP_CH_SPACE_ADDR;
  // Header power
  pArray[15] = 0x06;
  pArray[16] = S2LP_WRITE_REG_BYTE;
  pArray[17] = S2LP_PA_POWER0_ADDR;
  // Header frequency
  pArray[22] = 0x06;
  pArray[23] = S2LP_WRITE_REG_BYTE;
  pArray[24] = S2LP_SYNT3_ADDR;
  // Set reg default values
  pArray[18] = 0x47; // PA_Power0
  pArray[19] = 0x03; // PA_Config1
  pArray[20] = 0x8a; // PA_Config0
  pArray[21] = 0xd0; // Synth_Config2
}

void s2lp_init_pk_cfg_array(uint8_t *pArray, uint16_t array_size) {
  // Check size
  assert(array_size == S2LP_PK_CFG_ARRAY_SIZE);
  // Reset memory
  memset(pArray, 0, array_size);
  // Header PCKTCTRL
  pArray[0] = 0x08; // Size
  pArray[1] = S2LP_WRITE_REG_BYTE;
  pArray[2] = S2LP_PCKTCTRL6_ADDR;
  // Header SYNC
  pArray[9] = 0x06;
  pArray[10] = S2LP_WRITE_REG_BYTE;
  pArray[11] = S2LP_SYNC3_ADDR;
  // Header PROTOCOL
  pArray[16] = 0x05;
  pArray[17] = S2LP_WRITE_REG_BYTE;
  pArray[18] = S2LP_PROTOCOL2_ADDR;
  // Header TIMER
  pArray[22] = 0x06;
  pArray[23] = S2LP_WRITE_REG_BYTE;
  pArray[24] = S2LP_TIMERS5_ADDR;
  // Set reg default values
  pArray[19] = 0x40; // PROTOCOL2
  pArray[20] = S2LP_PROTOCOL1_AUTO_PCKT_FLT_REGMASK; // PROTOCOL1
  pArray[21] = 0x08; // PROTOCOL0
  pArray[25] = 0x01; // TIMER5
  pArray[26] = 0x00; // TIMER4
  pArray[27] = 0x01; // TIMER
  pArray[28] = 0x00; // TIMER2
}



// Init config table
const uint8_t s2lp_config[] = {
  // FIFO threshold config
  0x7, S2LP_WRITE_REG_BYTE, S2LP_FIFO_CONFIG3_ADDR, S2LP_FIFO_THRESHOLD, S2LP_FIFO_THRESHOLD, S2LP_FIFO_THRESHOLD, S2LP_FIFO_THRESHOLD,
  S2LP_PCKT_FLT_CRC_FLT_REGMASK,
  // IRQs Masks
  0x6, S2LP_WRITE_REG_BYTE, S2LP_IRQ_MASK3_ADDR, S2LP_IRQ_REG_3_VALUE, S2LP_IRQ_REG_2_VALUE,
  S2LP_IRQ_REG_1_VALUE, S2LP_IRQ_REG_0_VALUE,
  // CS Mode config
  0x3, S2LP_WRITE_REG_BYTE, S2LP_RSSI_FLT_ADDR, (S2LP_RSSI_FLT_DEV_VAL | S2LP_RSSI_CS_MODE_DYN6DB),
  // PM config
  0x3, S2LP_WRITE_REG_BYTE, S2LP_PM_CONF0_ADDR, (S2LP_PM0_DEF_VAL | S2LP_PM0_SLEEP_MODE_SEL_REGMASK),
#ifdef CONFIG_DRIVER_RFPACKET_S2LP_USE_CSMA
  // CSMA config
  0x4, S2LP_WRITE_REG_BYTE, S2LP_CSMA_CONF1_ADDR, (S2LP_CSMA_CONF1_DEF_VAL | S2LP_CSMA_PERIOD_64TBIT),
  (S2LP_CSMA0_CCA_LEN_OFFSET(1) | 0x01),
  // Misc config
  0x4, S2LP_WRITE_REG_BYTE, S2LP_ANT_SELECT_CONF_ADDR, (S2LP_ANT_DEF_VAL), 0x00,
#else
  // Activate CS_Blanking (interfere with CSMA)
  0x4, S2LP_WRITE_REG_BYTE, S2LP_ANT_SELECT_CONF_ADDR, (S2LP_ANT_DEF_VAL | S2LP_ANT_CS_BLANKING_REGMASK), 0x00,  
#endif
  // Dig div config
  0x4, S2LP_WRITE_REG_BYTE, S2LP_XO_RCO_CONF1_ADDR, S2LP_RCO_CONF1_VAL, S2LP_RCO_CONF0_VAL,
  // If config
  0x4, S2LP_WRITE_REG_BYTE, S2LP_IF_OFFSET_ANA_ADDR, S2LP_IF_OFFSET_ANA_VAL, S2LP_IF_OFFSET_DIG_VAL,
  // GPIO_0 nIRQ
  0x3, S2LP_WRITE_REG_BYTE, S2LP_GPIO0_CONF_ADDR, (S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP | S2LP_GPIO_DIG_OUT_IRQ),
  // Config end
  0x0,
};
