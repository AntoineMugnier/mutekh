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

#ifndef SI446X_MODEM_CALC_H_
#define SI446X_MODEM_CALC_H_

struct si446x_modem_regs_s {
	/* 20:00 */
	uint8_t mod_type;
	uint8_t map_control;
	uint8_t dsm_ctrl;
	uint8_t data_rate_2, data_rate_1, data_rate_0;
	uint8_t tx_nco_mode_3, tx_nco_mode_2, tx_nco_mode_1, tx_nco_mode_0;

	/* 20:0a */
	uint8_t freq_dev_2, freq_dev_1, freq_dev_0;

	/* 20:18 */
	uint8_t tx_ramp_delay;
	uint8_t mdm_ctrl;
	uint8_t if_control;
	uint8_t if_freq_2, if_freq_1, if_freq_0;
	uint8_t dec_cfg1, dec_cfg0;
#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV >= 0x20
	uint8_t dec_cfg2;
#endif

	/* 20:22 */
	uint8_t bcr_osr_1, bcr_osr_0;
	uint8_t bcr_nco_offset_2, bcr_nco_offset_1, bcr_nco_offset_0;
	uint8_t bcr_gain_1, bcr_gain_0;
	uint8_t bcr_gear;
	uint8_t bcr_misc1;

	/* 20:2c */
	uint8_t afc_gear;
	uint8_t afc_wait;
	uint8_t afc_gain_1, afc_gain_0;
	uint8_t afc_limiter_1, afc_limiter_0;
	uint8_t afc_misc;
	uint8_t afc_zifoff;
	uint8_t adc_ctrl;
	uint8_t agc_control;

	/* 20:38 */
	uint8_t agc_window_size;
	uint8_t agc_rfpd_decay;
	uint8_t agc_ifpd_decay;
	uint8_t fsk4_gain1, fsk4_gain0;
	uint8_t fsk4_th1, fsk4_th0;
	uint8_t fsk4_map;
	uint8_t ook_pdtc;
	uint8_t ook_blopk;
	uint8_t ook_cnt1;
	uint8_t ook_misc;

	/* 20:45 */
	uint8_t raw_control;
	uint8_t raw_eye_1, raw_eye_0;
	uint8_t ant_div_mode;
	uint8_t ant_div_control;

	/* 20:50 */
	uint8_t raw_search2;
	uint8_t clkgen_band;

	/* 20:54 */
	uint8_t spike_det;
	uint8_t one_shot_afc;

#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV >= 0x20
	/* 20:5b */
	uint8_t dsa_ctrl1, dsa_ctrl2;
	uint8_t dsa_qual;
	uint8_t dsa_rssi;
#endif

        /* 21:00 */
        uint8_t rx1_coe[14 + 4];
        /* 21:12 */
        uint8_t rx2_coe[14 + 4];
} __attribute__((packed));

struct si446x_synth_regs_s {
	/* 23:00 */
	uint8_t pfdcp_cpff, pfdcp_cpint;
	uint8_t vco_kv;
	uint8_t lpfilt3, lpfilt2, lpfilt1, lpfilt0;
} __attribute__((packed));

struct si446x_freq_ctl_regs_s {
	/* 40:00 */
	uint8_t inte;
	uint8_t frac_2, frac_1, frac_0;
	uint8_t channel_step_size_1;
	uint8_t channel_step_size_0;
	uint8_t w_size;
	uint8_t vcocnt_rx_adj;

} __attribute__((packed));

struct si446x_rf_regs_s {
    struct si446x_modem_regs_s modem;
    struct si446x_synth_regs_s synth;
    struct si446x_freq_ctl_regs_s freq;
    uint8_t rssi_th;
} __attribute__((packed));

static const uint8_t si446x_rf_cmd[] = {
  10, 0x20, 0x00, __builtin_offsetof(struct si446x_rf_regs_s, modem.mod_type),
#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV >= 20
  9,  0x20, 0x18, __builtin_offsetof(struct si446x_rf_regs_s, modem.tx_ramp_delay),
#else
  8,  0x20, 0x18, __builtin_offsetof(struct si446x_rf_regs_s, modem.tx_ramp_delay),
#endif
  9,  0x20, 0x22, __builtin_offsetof(struct si446x_rf_regs_s, modem.bcr_osr_1),
  10, 0x20, 0x2c, __builtin_offsetof(struct si446x_rf_regs_s, modem.afc_gear),
  12, 0x20, 0x38, __builtin_offsetof(struct si446x_rf_regs_s, modem.agc_window_size),
  5,  0x20, 0x45, __builtin_offsetof(struct si446x_rf_regs_s, modem.raw_control),
  2,  0x20, 0x54, __builtin_offsetof(struct si446x_rf_regs_s, modem.spike_det),
#if CONFIG_DRIVER_RFPACKET_SI446X_CHIPREV >= 20
  2,  0x20, 0x5b, __builtin_offsetof(struct si446x_rf_regs_s, modem.dsa_ctrl1),
#endif
  3,  0x20, 0x0a, __builtin_offsetof(struct si446x_rf_regs_s, modem.freq_dev_2),
  2,  0x20, 0x50, __builtin_offsetof(struct si446x_rf_regs_s, modem.raw_search2),
  7,  0x23, 0x00, __builtin_offsetof(struct si446x_rf_regs_s, synth.pfdcp_cpff),
  8,  0x40, 0x00, __builtin_offsetof(struct si446x_rf_regs_s, freq.inte),
  1,  0x20, 0x4A, __builtin_offsetof(struct si446x_rf_regs_s, rssi_th),
  12, 0x21, 0x00, __builtin_offsetof(struct si446x_rf_regs_s, modem.rx1_coe),
  12, 0x21, 0x0c, __builtin_offsetof(struct si446x_rf_regs_s, modem.rx1_coe) + 12,
  12, 0x21, 0x18, __builtin_offsetof(struct si446x_rf_regs_s, modem.rx1_coe) + 24,
  0x00
};

struct si446x_rf_regs_s;
struct dev_rfpacket_rf_cfg_s;
struct dev_rfpacket_pk_cfg_s;

size_t si446x_modem_configure(struct si446x_rf_regs_s *out,
			      const struct dev_rfpacket_rf_cfg_s *rf_cfg,
                              const struct dev_rfpacket_pk_cfg_s *pk_cfg);

#endif

