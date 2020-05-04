/*
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this file.  If not, see <http://www.gnu.org/licenses/>.

    Copyright (c) 2014
    Implementation by Alexandre Becoulet <alexandre.becoulet@free.fr>
    Based on reverse by Sebastien Cerdan <sebcerdan@gmail.com>

*/

/*
    This file implements a modem calculator which gives the same
    output as the one provided by the silabs tools. It has been highly
    optimized for small footprint and efficiency when used on embedded
    platforms.

    It can be compiled as a standalone C application for test purpose:

      gcc modem_calc.c -o modem_calc
      ./modem_calc 1 868 2400 48000 9600 0

    The following parameters are inputs to this modem calculator:

      API_freq_xo
      API_modulation_type
      API_Rsymb
      API_Fdev
      API_RXBW
      API_fc
      API_Manchester
      API_hi_pfm_div_mode
      API_inputBW            = !API_RXBW
      API_crystal_tol
      API_Ch_Fil_Bw_AFC

    The Rx bandwidth is calculated automatically when 0.

    The following parameters are hardwired:

      API_if_mode            = 2.0
      API_High_perf_Ch_Fil   = 1.0
      API_OSRtune            = 0.0
      API_ant_div            = 0.0
      API_pm_pattern         = 0.0
      API_afc_en             = 1
      API_Max_Rb_Error       = 0.0
      API_Chip_Version       = 2.0
      API_TC                 = 29.0
      API_fhst               = 250000.0
      API_BER_mode           = 0.0
      API_raw_demod          = 0.0
      API_dsource            = 0.0

*/

#define LOGK_MODULE_ID "si44"

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdint.h>

#define CONFIG_DRIVER_RFPACKET_SI446X_PFM 2	/* 1 or 2 */

#ifdef __MUTEKH__

#include <device/class/rfpacket.h>
#include <mutek/printk.h>
#include <hexo/bit.h>

#else

#include <inttypes.h>
#include <stdio.h>

#define CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO 26000000U
#define CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV 0x22
#define CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW
#define CONFIG_DRIVER_RFPACKET_SI446X_MOD_OOK
#define CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK
#define CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK
#define CONFIG_DRIVER_RFPACKET_SI446X_AFC
#define CONFIG_DRIVER_RFPACKET_SI446X_AFCBD

#define __unused__ __attribute__((unused))

#define __MIN(a, b) ({ const typeof(a) __a = (a); const typeof(b) __b = (b); __b < __a ? __b : __a; })
#define __MAX(a, b) ({ const typeof(a) __a = (a); const typeof(b) __b = (b); __b > __a ? __b : __a; })
#define bit_clz32(x) (__builtin_clzll(x) + 32 - sizeof(long long) * 8)
#define logk_trace(...) fprintf(stderr, __VA_ARGS__)

FILE * fp;
typedef uint8_t bool_t;

#endif

#include "modem_calc.h"

static inline uint32_t floor_log2(uint32_t x)
{
	return 31 - bit_clz32(x);
}

static inline uint32_t ceil_log2(uint32_t x)
{
	uint32_t c = bit_clz32(x);
	if ((0x7fffffff >> c) & x)
	  c--;
	return 31 - c;
}

static inline double round(double r)
{
	return (uint32_t)(r + .5);
}

bool_t modem_calc(struct si446x_rf_regs_s *out,
                  uint32_t *synth_ratio,
                  enum si446x_modulation_e mod,
                  uint32_t freq, uint32_t rate,
                  uint32_t fdev, uint32_t rxbw_,
                  uint32_t channel_spacing,
                  uint32_t freq_err,
                  bool_t manchester)
{
  const struct si446x_synth_regs_s *synth;
  struct si446x_freq_ctl_regs_s *freq_ctl = &out->freq;
  struct si446x_modem_regs_s *modem = &out->modem;

#define l1 ((uint16_t)(687500000.000 * 32 / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO - 0x80))
#define l2 ((uint16_t)(425000000.000 * 32 / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO - 0x80))
#define l3 ((uint16_t)(345000000.000 * 32 / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO - 0x80))
#define l4 ((uint16_t)(272916666.667 * 32 / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO - 0x80))
#define l5 ((uint16_t)(193750000.000 * 32 / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO - 0x80))

  assert(l5 <= 0x1ff &&
         (l4 - l5) <= 0x1ff &&
         (l3 - l4) <= 0x1ff &&
         (l2 - l3) <= 0x1ff &&
         (l1 - l2) <= 0x1ff);

  static const uint16_t band_tab[5] = {
    (l5        << 7) + (4 << 3) + 1,
    ((l4 - l5) << 7) + (2 << 3) + 1,
    ((l3 - l4) << 7) + (1 << 3) + 2,
    ((l2 - l3) << 7) + (1 << 3) - 1 /* carry trick */,
    ((l1 - l2) << 7) + (2 << 3) + 2
  };

  uint32_t freq_b = freq * 32. / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO - 0x80;
  uint32_t i, t = (freq_b << 7) | (12 << 3) | 5;
  for (i = 0; i < 5; i++) {
    if (t < band_tab[i])
      break;
    t -= band_tab[i];
  }

#if CONFIG_DRIVER_RFPACKET_SI446X_PFM == 2
  modem->clkgen_band = (t & 0x7) | 8;
#else
  modem->clkgen_band = (t & 0x7);
#endif
  uint32_t freq_band = (t >> 2) & 0x1e;

  freq_ctl->w_size = 32;
  freq_ctl->vcocnt_rx_adj = -(freq_band >> 2);

  double freq_rel = CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO / CONFIG_DRIVER_RFPACKET_SI446X_PFM / (double)(freq_band << 17);
  double fdev_rate = fdev * 2 / (double)rate;
  uint32_t mfdev;
  bool_t ph_src;
  uint8_t detector;
  uint8_t min_osr;

  double mod_bw;
  double rxbw = rxbw_, rxbw_rate;

  static const struct si446x_synth_regs_s synth_tab[4] = {
    { 44, 14, 11, 4, 12, 115, 3 },
    { 52, 4,  11, 4,  7, 112, 3 },
    { 57, 4,  11, 5 , 4,   1, 3 },
    { 1,  5,  11, 5,  2,   0, 3 },
  };

  synth = synth_tab;

  switch (mod) {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW
  case MOD_RAW:
    mfdev = fdev;
    ph_src = 0;
    detector = 3;
    goto raw;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_OOK
  case MOD_OOK:
    mfdev = 0;
    ph_src = 0;
    detector = 0;
    mod_bw = rate * 2;
    if (rxbw == 0)
      rxbw = 2 * freq_err + mod_bw;
    rxbw = __MIN(620000, __MAX(75000, rxbw));
    min_osr = 14;
    break;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK
  case MOD_2FSK:
  case MOD_2GFSK:
    mfdev = fdev;
    ph_src = (fdev_rate >= 2);
    detector = ph_src ^ 3;
    goto fsk;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK
  case MOD_4FSK:
  case MOD_4GFSK:
    mfdev = 3 * fdev;
    ph_src = 0;
    detector = 3;
#endif
#if defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW)
  fsk:
    synth += (rate >= 75000) + (rate >= 125000) + (rate >= 200000);
  raw:
    mod_bw = rate + mfdev * 2;
    double r = ph_src ? 0.1
# ifdef CONFIG_DRIVER_RFPACKET_SI446X_AFC
      : 0.7;
# else
    : 0.5;
# endif
    if (rxbw == 0)
      rxbw = (2 * freq_err > (r * mod_bw) ? (2 * freq_err + ((1 - r) * mod_bw)) : mod_bw);
    rxbw = __MIN(900000, rxbw);
    min_osr = 8;
    break;
#endif
  default:
    return 0;
  }

  logk_trace("rxbw: %u ferr: %u\n", (uint32_t)rxbw, (uint32_t)freq_err);

  rxbw_rate = __MAX(rxbw * 4. / rate, min_osr);
  rxbw /= 1000.;
  mod_bw /= 1000;

  modem->mdm_ctrl = ph_src << 7;

  modem->ook_blopk = 12;
  modem->ook_cnt1 = (!manchester << 5) | 0x84;
  modem->ook_misc = detector;

  /********************* MODEM_TX_NCO_MODE */

#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV < 0x20
  static const bool_t txosr_3 = 0, txosr_2 = 0;
#else
  bool_t rev2_txosr = !(mod == MOD_2GFSK || mod == MOD_4GFSK) || (rate > 200000);
  bool_t txosr_3 = !((rev2_txosr || (rate < 25000)));
  bool_t txosr_2 = !((rev2_txosr || (rate >= 25000)));
#endif

  modem->tx_nco_mode_3 = (txosr_3 << 3) | (txosr_2 << 2) | (CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO >> 24);
  modem->tx_nco_mode_2 = (uint8_t)(CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO >> 16);
  modem->tx_nco_mode_1 = (uint8_t)(CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO >> 8);
  modem->tx_nco_mode_0 = (uint8_t)(CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO);

  /********************* MODEM_DATA_RATE */

  uint32_t data_rate = txosr_3 ? rate * 20 : txosr_2 ? rate * 40 : rate * 10;

  modem->data_rate_2 = data_rate >> 16;
  modem->data_rate_1 = data_rate >> 8;
  modem->data_rate_0 = data_rate;

  /********************* MODEM_FREQ_DEV */

  t = __MIN(131071, round(mfdev / freq_rel));
  modem->freq_dev_2 = t >> 16;
  modem->freq_dev_1 = t >> 8;
  modem->freq_dev_0 = t;

  /********************* FREQ_CONTROL */

  double freq_real = (double)freq_band * freq / (CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO * 4. / CONFIG_DRIVER_RFPACKET_SI446X_PFM);
  freq_ctl->inte = (uint32_t)freq_real - 1;

  uint32_t freq_frac = (freq_real - (uint32_t)freq_real + 1) * 0x80000;

  freq_ctl->frac_2 = freq_frac >> 16;
  freq_ctl->frac_1 = freq_frac >> 8;
  freq_ctl->frac_0 = freq_frac;

  *synth_ratio = CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO * CONFIG_DRIVER_RFPACKET_SI446X_PFM * 16 / freq_band;

  /********************* FREQ_CONTROL_CHANNEL_STEP_SIZE */

  uint32_t ch_step = round((double)channel_spacing
                           * (freq_band * CONFIG_DRIVER_RFPACKET_SI446X_PFM) * 0x20000 / CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO);

  freq_ctl->channel_step_size_1 = ch_step >> 8;
  freq_ctl->channel_step_size_0 = ch_step;

  /********************* MODEM_IF */

#if CONFIG_DRIVER_RFPACKET_SI446X_PFM == 2
  uint32_t modem_if_freq = -(freq_band << 12);
#else
  uint32_t modem_if_freq = -(freq_band << 11);
#endif

  modem->if_control = 8;
  modem->if_freq_2 = (modem_if_freq >> 16) & 3;
  modem->if_freq_1 = modem_if_freq >> 8;
  modem->if_freq_0 = modem_if_freq;

  /********************* decimation */

  uint32_t x, decim_a = 0, decim_b = 1, decim_c = 0, decim_d = 0;
  uint32_t decim = ((double)CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO / 8 / rate) / rxbw_rate;

  static const uint16_t decim_tab[21]= {
    0x0001, 0x0002, 0x0003, 0x0112,
    0x0113, 0x0222, 0x0223, 0x0332,
    0x0333, 0x1342, 0x1343, 0x2352,
    0x2353, 0x3362, 0x3363, 0x3372,
    0x3373, 0x3382, 0x3383, 0x3392,
    0x3393
  };

  for (i = 0; i < 21; i++) {
    x = decim_tab[i];
    if (decim < ((x & 15) << ((x >> 4) & 15)))
      break;
    decim_a = (x >> 4) & 15;
    decim_b = x & 15;
    decim_c = (x >> 8) & 15;
    decim_d = (x >> 12) & 15;
  }

  double decim_e = ((double)CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO / (decim_b * 8)) / (1 << decim_a);
  double decim_f = decim_e / rate;
  uint32_t decim_g = 0;

  /********************* DECIMATION, BCR, ... */

  double bcr_a = 2;
  double spike = 64;
  bool_t rate_th = (fdev_rate >= 2) && (decim_f / fdev_rate) >= 8;

  modem->dec_cfg0 = ((decim_b != 3) << 5) | ((decim_b != 2) << 4);

  modem->bcr_gear = 2;
  modem->bcr_misc1 = 0;
  modem->raw_search2 = 132;
  modem->afc_wait = rate >= 200000 ? 0x34 : 0x12;
  modem->tx_ramp_delay = 0;

  double raw_a = 1.3;

  switch (mod) {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_OOK
  case MOD_OOK: {
    uint32_t ndec = floor_log2(decim_f / 7);
    if (ndec < 1)
      return 0;
    decim_g = __MIN(5, ndec);
    rate_th = 0;
    modem->bcr_misc1 = 0xc0;
    modem->raw_search2 = 148;
    break;
  }
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK
  case MOD_2GFSK:
    spike = 64 * .65;
  case MOD_2FSK:
    modem->tx_ramp_delay = 1;
    if ((fdev_rate > 10) && (mod_bw > 200))
      modem->dec_cfg0 |= (1 << 6);
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW
  case MOD_RAW:
#endif
#if defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW)
    if (ph_src) {
      modem->afc_wait += 0x20;
      modem->bcr_misc1 = 0xc2;
    } else {
      bcr_a = fdev_rate;
    }
    goto fsk2;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK
  case MOD_4GFSK:
    spike = 64 * .65;
    modem->tx_ramp_delay = 1;
  case MOD_4FSK:
    modem->tx_ramp_delay += 4;
    spike *= 3;
    modem->bcr_misc1 = 0x02;
    bcr_a = 3 * fdev_rate;
    goto fsk2;
#endif
#if defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW)
  fsk2:
    if (rate_th) {
      static const double tab[5] = {
        135.7645019878171, 271.5290039756342,
        543.0580079512685, 1086.116015902537,
        2172.232031805074
      };
      while (decim_g < 5 && decim_f > tab[decim_g])
        decim_g++;
      raw_a = 1.1;
    }
    break;
#endif
  default:
    return 0;
  }

  modem->dec_cfg1 = (decim_d << 6) | (decim_c << 4) | (decim_g << 1);

#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV >= 0x20
  modem->dec_cfg2 = (decim_a - decim_c - decim_d) << 5;
  if (rate < 9600)
    modem->dec_cfg2 |= rate > 1000 ? 12 : 20;
#endif

  spike *= fdev_rate / decim_f;
  modem->spike_det = round(__MIN(127, spike));

  double bcr_c = decim_f / (1 << decim_g);

  uint32_t bcr_nco = round(4194304. / bcr_c);

  modem->bcr_nco_offset_2 = bcr_nco >> 16;
  modem->bcr_nco_offset_1 = bcr_nco >> 8;
  modem->bcr_nco_offset_0 = bcr_nco;

  uint32_t bcr_b = round(bcr_c * 8);
  modem->bcr_osr_1 = (bcr_b >> 8) & 15;
  modem->bcr_osr_0 = bcr_b;
  modem->ook_pdtc = __MIN(11, ceil_log2(bcr_b) + 1) + 32;

  uint32_t bcr_gain = __MIN(2047, (uint32_t)round(0x20000 / (bcr_b * bcr_a)));
  modem->bcr_gain_1 = bcr_gain >> 8;
  modem->bcr_gain_0 = bcr_gain;

  /********************* RAW */

  uint32_t raw_d = raw_a * (freq_err * 128 / rate / decim_f + spike) * 48;

  uint32_t raw_gain_tmp = (decim_g > 0) << 2;
  raw_d /= (4096 >> raw_gain_tmp);

  uint32_t raw_gain = raw_d >= 16 ? 0 : (0x5b >> (raw_d << 1)) & 3; /* lookup table [ 3, 2, 1, 1, 0 ... 0 ] */
  modem->raw_control = (ph_src << 7) | raw_gain;

  uint32_t raw_b = 6 << (raw_gain_tmp + raw_gain);

  uint32_t raw_c = __MIN(2047, (uint32_t)(round(((raw_a * raw_b) * spike))));
  modem->raw_eye_1 = (raw_c >> 8) & 7;
  modem->raw_eye_0 = raw_c;

  /********************* AFC, ANT_DIV, FSK4 */

  modem->ant_div_mode = 2;
  modem->ant_div_control = 0x80;

  bool_t rate_100k = (rate < 100000);

  if (detector == 2)
    decim_f = 16;
  uint32_t afc_a = round(rate * 2 / freq_rel * (ph_src ? decim_f / raw_b : 1. / (1 + rate_100k)) / 8.);

  bool_t afc_b = (afc_a > 4095);
  if (afc_b)
    afc_a = (afc_a >> 1) + (afc_a & 1); /* round(afc_a / 2.) */

  uint32_t afc_gain = __MAX(1, __MIN(4095, afc_a));
  modem->afc_gear = ph_src << 2;
  modem->afc_gain_0 = afc_gain;
  modem->afc_misc = 0x80 | (!ph_src << 5);
  modem->afc_gain_1 = (afc_gain >> 8) & 31;
  modem->afc_zifoff = 0;
  modem->adc_ctrl = 0;
  modem->agc_control = 0x62;

  static const uint16_t filters_bw[16] = {
    9157, 8245, 7409, 6611, 5936, 5354, 4839, 4361,
    3932, 3532, 3172, 2825, 2557, 2302, 2075, 0
  };

  double coef_band1 = rxbw * decim_b * (1 << decim_a);
  uint_fast8_t coef_idx1 = 0, coef_idx2 = 0;
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_AFCBD
  double coef_band2 = mod_bw * decim_b * (1 << decim_a);
#endif
  for (i = 1; i < 16; i++) {
    double cbw = CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO / 300000000. * filters_bw[i];
    if (!coef_idx1 && cbw < coef_band1)
      coef_idx1 = i;
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_AFCBD
    if (!coef_idx2 && cbw < coef_band2)
      coef_idx2 = i;
#endif
  }

  coef_idx1--;

  modem->fsk4_gain1 = 0x80;
  modem->fsk4_gain0 = 26;
  modem->fsk4_map = 1;

  uint32_t fsk4_th = __MIN(65535, (uint32_t)round(fdev * 32768. / rate));
  modem->fsk4_th1 = fsk4_th >> 8;
  modem->fsk4_th0 = fsk4_th;

  __unused__ double qual_factor = 2;

  switch (mod) {
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_OOK
  case MOD_OOK:
# ifdef CONFIG_DRIVER_RFPACKET_SI446X_AFCBD
    coef_idx2 = coef_idx1;
    modem->afc_gain_1 |= 0x40;
# endif
    if (manchester)
      modem->agc_control |= 0x80;
    if (rate <= 500)
      modem->agc_control |= 0x08;
    break;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK
  case MOD_2GFSK:
    qual_factor = 1.5;
  case MOD_2FSK:
    if (rate >= 300000)
      coef_idx1++;
    goto fsk3;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK
  case MOD_4GFSK:
    qual_factor = 1.5;
  case MOD_4FSK:
    modem->ant_div_control = 0xf0;
    modem->fsk4_gain1 = 0;
#endif
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW
  case MOD_RAW:
#endif
#if defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_2FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_4FSK) || \
  defined(CONFIG_DRIVER_RFPACKET_SI446X_MOD_RAW)
  fsk3:
# ifdef CONFIG_DRIVER_RFPACKET_SI446X_AFC
#  ifdef CONFIG_DRIVER_RFPACKET_SI446X_AFCBD
    coef_idx2 = __MAX(coef_idx1, coef_idx2 - 1);
    modem->afc_gain_1 |= 0x40;
#  endif
    modem->afc_gain_1 |= 0x80;
    modem->afc_misc |= 0x40;
# endif
    modem->agc_control |= 0x80;
    break;
#endif
  default:
    return 0;
  }

#ifndef CONFIG_DRIVER_RFPACKET_SI446X_AFCBD
  coef_idx2 = coef_idx1;
#endif

  double afc_c = __MAX((CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO / 300000000. * filters_bw[coef_idx1] / (decim_b << decim_a)), rxbw);

  double afc_d = afc_a > 4095 ? .38 : .4;
  if (fdev_rate >= 2)
    afc_d *= 0.714;
  if (rate_100k)
    afc_d += (0.04 - afc_c / 20600.) * rate_100k;
  uint32_t afc_e = round(((afc_c * 64000 * afc_d) / afc_gain) / freq_rel);

  uint32_t afc_f = afc_e >= 16384 ? round(afc_e / 8.) + 16384 : afc_e;

  modem->afc_limiter_1 = afc_f >> 8;
  modem->afc_limiter_0 = afc_f;
  modem->one_shot_afc = 6;

  /********************* AGC */

  uint32_t agc_a = __MAX(1, (uint32_t)round((decim_e / (2000000 << decim_g))));
  uint32_t agc_b = __MAX(1, (uint32_t)round((decim_e / (2500000 << decim_g))));
  modem->agc_window_size = (agc_a << 4) | agc_b;

  uint32_t agc_decay = __MAX(__MIN((uint32_t)round(bcr_b * .4375 / (agc_a + agc_b)), 255), 1);
  modem->agc_rfpd_decay = agc_decay;
  modem->agc_ifpd_decay = agc_decay;

  /********************* DSA */

#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV >= 0x20
  modem->dsa_ctrl1 = 66;
  modem->dsa_ctrl2 = 3;
  modem->dsa_qual = round(spike * qual_factor);
  modem->dsa_rssi = 248;
#endif

  /********************* */

  modem->mod_type = mod;
  modem->map_control = manchester << 7;
  modem->dsm_ctrl = 7;

  memcpy(&out->synth, synth, sizeof(struct si446x_synth_regs_s));

  static const uint8_t si446x_rx_chflt_coe[15][14 + 4] = {
    /* coe                                                                                  coem */
    { 0xff, 0xba, 0x0f, 0x51, 0xcf, 0xa9, 0xc9,  0xfc, 0x1b, 0x1e, 0x0f, 0x01, 0xfc, 0xfd,  0x15, 0xff, 0x00, 0x4f },
    { 0xff, 0xc4, 0x30, 0x7f, 0xf5, 0xb5, 0xb8,  0xde, 0x05, 0x17, 0x16, 0x0c, 0x03, 0x00,  0x15, 0xff, 0x00, 0x40 },
    { 0xcc, 0xa1, 0x30, 0xa0, 0x21, 0xd1, 0xb9,  0xc9, 0xea, 0x05, 0x12, 0x11, 0x0a, 0x04,  0x15, 0xfc, 0x03, 0x40 },
    { 0xa2, 0x81, 0x26, 0xaf, 0x3f, 0xee, 0xc8,  0xc7, 0xdb, 0xf2, 0x02, 0x08, 0x07, 0x03,  0x15, 0xfc, 0x0f, 0x40 },
    { 0x7e, 0x64, 0x1b, 0xba, 0x58, 0x0b, 0xdd,  0xce, 0xd6, 0xe6, 0xf6, 0x00, 0x03, 0x03,  0x15, 0xf0, 0x3f, 0x40 },
    { 0x5b, 0x47, 0x0f, 0xc0, 0x6d, 0x25, 0xf4,  0xdb, 0xd6, 0xdf, 0xec, 0xf7, 0xfe, 0x01,  0x15, 0xf0, 0xff, 0x43 },
    { 0x39, 0x2b, 0x00, 0xc3, 0x7f, 0x3f, 0x0c,  0xec, 0xdc, 0xdc, 0xe3, 0xed, 0xf6, 0xfd,  0x15, 0xc0, 0xff, 0x4f },
    { 0x23, 0x17, 0xf4, 0xc2, 0x88, 0x50, 0x21,  0xff, 0xec, 0xe6, 0xe8, 0xee, 0xf6, 0xfb,  0x05, 0xc0, 0xff, 0x4f },
    { 0x0c, 0x01, 0xe4, 0xb9, 0x86, 0x55, 0x2b,  0x0b, 0xf8, 0xef, 0xef, 0xf2, 0xf8, 0xfc,  0x05, 0x00, 0xff, 0x4f },
    { 0xf9, 0xf0, 0xd7, 0xb1, 0x86, 0x59, 0x33,  0x15, 0x01, 0xf7, 0xf4, 0xf6, 0xf9, 0xfc,  0x00, 0x00, 0xfc, 0x4f },
    { 0xe7, 0xdf, 0xca, 0xaa, 0x84, 0x5d, 0x3a,  0x1e, 0x0a, 0xfe, 0xf9, 0xf9, 0xfa, 0xfd,  0x00, 0x00, 0xfc, 0x4f },
    { 0xd6, 0xd0, 0xbe, 0xa3, 0x82, 0x61, 0x41,  0x27, 0x13, 0x06, 0xff, 0xfd, 0xfd, 0xfd,  0x00, 0x00, 0xf0, 0x4f },
    { 0xc6, 0xc1, 0xb2, 0x9c, 0x80, 0x63, 0x47,  0x2f, 0x1b, 0x0e, 0x05, 0x00, 0xff, 0xfe,  0x00, 0x00, 0x00, 0x4f },
    { 0xb4, 0xb0, 0xa5, 0x93, 0x7d, 0x65, 0x4d,  0x37, 0x25, 0x16, 0x02, 0x05, 0x02, 0xff,  0x00, 0x00, 0x00, 0x4c },
    { 0xa2, 0xa0, 0x97, 0x8a, 0x79, 0x66, 0x52,  0x3f, 0x2e, 0x1f, 0x14, 0x0b, 0x06, 0x02,  0x00, 0x00, 0x00, 0x40 },
  };

  memcpy(out->modem.rx1_coe, si446x_rx_chflt_coe[coef_idx1], 18);
  memcpy(out->modem.rx2_coe, si446x_rx_chflt_coe[coef_idx2], 18);
  out->modem.rx2_coe[17] ^= (!afc_b << 6);

  return 1;
}

#ifndef __MUTEKH__

int main(int argc, char *argv[])
{
  if (argc < 7)
    {
      fprintf(stderr, "usage: %s mod freq dev bw rate manchester\n", argv[0]);
      return -1;
    }

  uint32_t mod = atof(argv[1]);
  uint32_t freq = atof(argv[2]);
  double fdev = atof(argv[3]);
  double rxbw = atof(argv[4]);
  double rate = atof(argv[5]);
  uint32_t manchester = !!atoi(argv[6]);
  uint32_t sr;

  uint32_t freq_err = freq * ((20 /* ppm */ / 1000000.) /* tx side error */ +
                              (20 /* ppm */ / 1000000.) /* rx side error */ );

  struct si446x_rf_regs_s r;
  if (!modem_calc(&r, &sr, mod, freq, rate, fdev, rxbw, 0x200000, freq_err, manchester))
    {
      fprintf(stderr, "error\n");
      return -1;
    }

  fprintf(stderr, "synth ratio 0x%X\n", sr);
  printf("set MODEM_MOD_TYPE 0x%02X\n", r.modem.mod_type);
  printf("set MODEM_MAP_CONTROL 0x%02X\n", r.modem.map_control);
  printf("set MODEM_DSM_CTRL 0x%02X\n", r.modem.dsm_ctrl);
  printf("set MODEM_CLKGEN_BAND 0x%02X\n", r.modem.clkgen_band);
  printf("set SYNTH_PFDCP_CPFF 0x%02X\n", r.synth.pfdcp_cpff);
  printf("set SYNTH_PFDCP_CPINT 0x%02X\n", r.synth.pfdcp_cpint);
  printf("set SYNTH_VCO_KV 0x%02X\n", r.synth.vco_kv);
  printf("set SYNTH_LPFILT3 0x%02X\n", r.synth.lpfilt3);
  printf("set SYNTH_LPFILT2 0x%02X\n", r.synth.lpfilt2);
  printf("set SYNTH_LPFILT1 0x%02X\n", r.synth.lpfilt1);
  printf("set SYNTH_LPFILT0 0x%02X\n", r.synth.lpfilt0);
  printf("set MODEM_DATA_RATE_2 0x%02X\n", r.modem.data_rate_2);
  printf("set MODEM_DATA_RATE_1 0x%02X\n", r.modem.data_rate_1);
  printf("set MODEM_DATA_RATE_0 0x%02X\n", r.modem.data_rate_0);
  printf("set MODEM_TX_NCO_MODE_3 0x%02X\n", r.modem.tx_nco_mode_3);
  printf("set MODEM_TX_NCO_MODE_2 0x%02X\n", r.modem.tx_nco_mode_2);
  printf("set MODEM_TX_NCO_MODE_1 0x%02X\n", r.modem.tx_nco_mode_1);
  printf("set MODEM_TX_NCO_MODE_0 0x%02X\n", r.modem.tx_nco_mode_0);
  printf("set MODEM_FREQ_DEV_2 0x%02X\n", r.modem.freq_dev_2);
  printf("set MODEM_FREQ_DEV_1 0x%02X\n", r.modem.freq_dev_1);
  printf("set MODEM_FREQ_DEV_0 0x%02X\n", r.modem.freq_dev_0);
  printf("set MODEM_TX_RAMP_DELAY 0x%02X\n", r.modem.tx_ramp_delay);
  printf("set PA_TC 0x%02X\n", 93);
  printf("set FREQ_CONTROL_INTE 0x%02X\n", r.freq.inte);
  printf("set FREQ_CONTROL_FRAC_2 0x%02X\n", r.freq.frac_2);
  printf("set FREQ_CONTROL_FRAC_1 0x%02X\n", r.freq.frac_1);
  printf("set FREQ_CONTROL_FRAC_0 0x%02X\n", r.freq.frac_0);
  printf("set FREQ_CONTROL_CHANNEL_STEP_SIZE_1 0x%02X\n", r.freq.channel_step_size_1);
  printf("set FREQ_CONTROL_CHANNEL_STEP_SIZE_0 0x%02X\n", r.freq.channel_step_size_0);
  printf("set FREQ_CONTROL_W_SIZE 0x%02X\n", r.freq.w_size);
  printf("set FREQ_CONTROL_VCOCNT_RX_ADJ 0x%02X\n", r.freq.vcocnt_rx_adj);
  printf("set MODEM_MDM_CTRL 0x%02X\n", r.modem.mdm_ctrl);
  printf("set MODEM_IF_CONTROL 0x%02X\n", r.modem.if_control);
  printf("set MODEM_IF_FREQ_2 0x%02X\n", r.modem.if_freq_2);
  printf("set MODEM_IF_FREQ_1 0x%02X\n", r.modem.if_freq_1);
  printf("set MODEM_IF_FREQ_0 0x%02X\n", r.modem.if_freq_0);
  printf("set MODEM_DECIMATION_CFG1 0x%02X\n", r.modem.dec_cfg1);
  printf("set MODEM_DECIMATION_CFG0 0x%02X\n", r.modem.dec_cfg0);
  printf("set MODEM_BCR_OSR_1 0x%02X\n", r.modem.bcr_osr_1);
  printf("set MODEM_BCR_OSR_0 0x%02X\n", r.modem.bcr_osr_0);
  printf("set MODEM_BCR_NCO_OFFSET_2 0x%02X\n", r.modem.bcr_nco_offset_2);
  printf("set MODEM_BCR_NCO_OFFSET_1 0x%02X\n", r.modem.bcr_nco_offset_1);
  printf("set MODEM_BCR_NCO_OFFSET_0 0x%02X\n", r.modem.bcr_nco_offset_0);
  printf("set MODEM_BCR_GAIN_1 0x%02X\n", r.modem.bcr_gain_1);
  printf("set MODEM_BCR_GAIN_0 0x%02X\n", r.modem.bcr_gain_0);
  printf("set MODEM_BCR_GEAR 0x%02X\n", r.modem.bcr_gear);
  printf("set MODEM_BCR_MISC1 0x%02X\n", r.modem.bcr_misc1);
  printf("set MODEM_AFC_GEAR 0x%02X\n", r.modem.afc_gear);
  printf("set MODEM_AFC_WAIT 0x%02X\n", r.modem.afc_wait);
  printf("set MODEM_AFC_GAIN_1 0x%02X\n", r.modem.afc_gain_1);
  printf("set MODEM_AFC_GAIN_0 0x%02X\n", r.modem.afc_gain_0);
  printf("set MODEM_AFC_LIMITER_1 0x%02X\n", r.modem.afc_limiter_1);
  printf("set MODEM_AFC_LIMITER_0 0x%02X\n", r.modem.afc_limiter_0);
  printf("set MODEM_AFC_MISC 0x%02X\n", r.modem.afc_misc);
  printf("set MODEM_AGC_CONTROL 0x%02X\n", r.modem.agc_control);
  printf("set MODEM_AGC_WINDOW_SIZE 0x%02X\n", r.modem.agc_window_size);
  printf("set MODEM_AGC_RFPD_DECAY 0x%02X\n", r.modem.agc_rfpd_decay);
  printf("set MODEM_AGC_IFPD_DECAY 0x%02X\n", r.modem.agc_ifpd_decay);
  printf("set MODEM_FSK4_GAIN1 0x%02X\n", r.modem.fsk4_gain1);
  printf("set MODEM_FSK4_GAIN0 0x%02X\n", r.modem.fsk4_gain0);
  printf("set MODEM_FSK4_TH1 0x%02X\n",   r.modem.fsk4_th1);
  printf("set MODEM_FSK4_TH0 0x%02X\n",   r.modem.fsk4_th0);
  printf("set MODEM_FSK4_MAP 0x%02X\n",   r.modem.fsk4_map);
  printf("set MODEM_OOK_PDTC 0x%02X\n", r.modem.ook_pdtc);
  printf("set MODEM_OOK_BLOPK 0x%02X\n",r.modem.ook_blopk);
  printf("set MODEM_OOK_CNT1 0x%02X\n", r.modem.ook_cnt1);
  printf("set MODEM_OOK_MISC 0x%02X\n", r.modem.ook_misc);
  printf("set MODEM_RAW_SEARCH2 0x%02X\n", r.modem.raw_search2);
  printf("set MODEM_RAW_CONTROL 0x%02X\n", r.modem.raw_control);
  printf("set MODEM_RAW_EYE_1 0x%02X\n", r.modem.raw_eye_1);
  printf("set MODEM_RAW_EYE_0 0x%02X\n", r.modem.raw_eye_0);
  printf("set MODEM_ANT_DIV_MODE 0x%02X\n", r.modem.ant_div_mode);
  printf("set MODEM_ANT_DIV_CONTROL 0x%02X\n", r.modem.ant_div_control);
  printf("set MODEM_RSSI_JUMP_THRESH 0x%02X\n", 4);
  printf("set MODEM_RSSI_CONTROL 0x%02X\n", 9);
  printf("set MODEM_RSSI_CONTROL2 0x%02X\n", 0);
  printf("set MODEM_RSSI_COMP 0x%02X\n", 64);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE13_7_0 0x%02X\n", r.modem.rx1_coe[ 0]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE12_7_0 0x%02X\n", r.modem.rx1_coe[ 1]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE11_7_0 0x%02X\n", r.modem.rx1_coe[ 2]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE10_7_0 0x%02X\n", r.modem.rx1_coe[ 3]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE9_7_0 0x%02X\n",  r.modem.rx1_coe[ 4]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE8_7_0 0x%02X\n",  r.modem.rx1_coe[ 5]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE7_7_0 0x%02X\n",  r.modem.rx1_coe[ 6]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE6_7_0 0x%02X\n",  r.modem.rx1_coe[ 7]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE5_7_0 0x%02X\n",  r.modem.rx1_coe[ 8]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE4_7_0 0x%02X\n",  r.modem.rx1_coe[ 9]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE3_7_0 0x%02X\n",  r.modem.rx1_coe[10]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE2_7_0 0x%02X\n",  r.modem.rx1_coe[11]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE1_7_0 0x%02X\n",  r.modem.rx1_coe[12]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COE0_7_0 0x%02X\n",  r.modem.rx1_coe[13]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COEM0 0x%02X\n",     r.modem.rx1_coe[14]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COEM1 0x%02X\n",     r.modem.rx1_coe[15]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COEM2 0x%02X\n",     r.modem.rx1_coe[16]);
  printf("set MODEM_CHFLT_RX1_CHFLT_COEM3 0x%02X\n",     r.modem.rx1_coe[17]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE13_7_0 0x%02X\n", r.modem.rx2_coe[ 0]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE12_7_0 0x%02X\n", r.modem.rx2_coe[ 1]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE11_7_0 0x%02X\n", r.modem.rx2_coe[ 2]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE10_7_0 0x%02X\n", r.modem.rx2_coe[ 3]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE9_7_0 0x%02X\n",  r.modem.rx2_coe[ 4]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE8_7_0 0x%02X\n",  r.modem.rx2_coe[ 5]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE7_7_0 0x%02X\n",  r.modem.rx2_coe[ 6]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE6_7_0 0x%02X\n",  r.modem.rx2_coe[ 7]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE5_7_0 0x%02X\n",  r.modem.rx2_coe[ 8]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE4_7_0 0x%02X\n",  r.modem.rx2_coe[ 9]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE3_7_0 0x%02X\n",  r.modem.rx2_coe[10]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE2_7_0 0x%02X\n",  r.modem.rx2_coe[11]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE1_7_0 0x%02X\n",  r.modem.rx2_coe[12]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COE0_7_0 0x%02X\n",  r.modem.rx2_coe[13]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COEM0 0x%02X\n",     r.modem.rx2_coe[14]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COEM1 0x%02X\n",     r.modem.rx2_coe[15]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COEM2 0x%02X\n",     r.modem.rx2_coe[16]);
  printf("set MODEM_CHFLT_RX2_CHFLT_COEM3 0x%02X\n",     r.modem.rx2_coe[17]);
  printf("set MODEM_SPIKE_DET 0x%02X\n", r.modem.spike_det);
  printf("set MODEM_DSA_CTRL1 0x%02X\n", r.modem.dsa_ctrl1);
  printf("set MODEM_DSA_CTRL2 0x%02X\n", r.modem.dsa_ctrl2);
  printf("set MODEM_ONE_SHOT_AFC 0x%02X\n", r.modem.one_shot_afc);
  printf("set MODEM_DSA_QUAL 0x%02X\n", r.modem.dsa_qual);
  printf("set MODEM_DSA_RSSI 0x%02X\n", r.modem.dsa_rssi);
  printf("set MODEM_DECIMATION_CFG2 0x%02X\n", r.modem.dec_cfg2);
  printf("set MODEM_RSSI_MUTE 0x%02X\n", 113);

  return 0;
}

#endif
