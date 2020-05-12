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

#ifndef S2LP_CONFIG_H_
#define S2LP_CONFIG_H_

#include "s2lp.h"

// Init config table
extern const uint8_t s2lp_config[];

// Rf config structure
// Header Rssi th
// (3) Rssi Threshold
// Header modulation / channel filter
// (7) Ch_Space, Ch_Num, Mod4 to Mod0, Ch_flt
// Header power
// (18) PA_Power0, PA_Config1, PA_Config0, Synth_Config2
// Header frequency
// (25) Synt3 to Synt0
// Header Xo conf
// (31) Xo conf 1



// Packet config structure
// Header PCKTCTRL
// (3) PCKTCTRL6 to PCKTCTRL1
// Header SYNC
// (12) Sync double word (big endian)
// Header PROTOCOL
// (19) PROTOCOL2 to PROTOCOL0
// Header Timer
// (25) TIMERS5 to TIMERS2

// Register offsets
#define S2LP_PKCFG_PROT2_OFFSET 19
#define S2LP_PKCFG_PROT1_OFFSET 20

// LDC time values
#define S2LP_LDC_RXT_US 2500
#define S2LP_LDC_WUT_US 500

// Public functions
error_t s2lp_build_config(struct s2lp_ctx_s *pv);
uint8_t s2lp_build_pwr(struct s2lp_ctx_s *pv, int16_t pwr);
bool s2lp_config_check_fairtx_valid(const struct dev_rfpacket_rf_cfg_s *rfcfg);
uint32_t s2lp_config_get_freq(const struct dev_rfpacket_rf_cfg_s *cfg, uint8_t chan);

#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
    void s2lp_init_rf_cfg_array(uint8_t *pArray, uint16_t array_size);
#endif
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG
    void s2lp_init_pk_cfg_array(uint8_t *pArray, uint16_t array_size);
#endif

#endif