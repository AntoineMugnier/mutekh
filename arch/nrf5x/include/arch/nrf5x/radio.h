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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#ifndef NRF_RADIO_H
#define NRF_RADIO_H

#include "peripheral.h"
#include "ids.h"

enum nrf5x_radio_task {
    NRF_RADIO_TXEN = 0,
    NRF_RADIO_RXEN = 1,
    NRF_RADIO_START = 2,
    NRF_RADIO_STOP = 3,
    NRF_RADIO_DISABLE = 4,
    NRF_RADIO_RSSISTART = 5,
    NRF_RADIO_RSSISTOP = 6,
    NRF_RADIO_BCSTART = 7,
    NRF_RADIO_BCSTOP = 8,
};

enum nrf5x_radio_event {
    NRF_RADIO_READY = 0,
    NRF_RADIO_ADDRESS = 1,
    NRF_RADIO_PAYLOAD = 2,
    NRF_RADIO_END = 3,
    NRF_RADIO_DISABLED = 4,
    NRF_RADIO_DEVMATCH = 5,
    NRF_RADIO_DEVMISS = 6,
    NRF_RADIO_RSSIEND = 7,
    NRF_RADIO_BCMATCH = 10,
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
    NRF_RADIO_CROK = 12,
    NRF_RADIO_CRERROR = 13,
#endif
};

enum nrf5x_radio_register {
    NRF_RADIO_CRCSTATUS = 0,
    NRF_RADIO_CD = 1,
    NRF_RADIO_RXMATCH = 2,
    NRF_RADIO_RXCRC = 3,
    NRF_RADIO_DAI = 4,
    NRF_RADIO_PACKETPTR = 65,
    NRF_RADIO_FREQUENCY = 66,
    NRF_RADIO_TXPOWER = 67,
    NRF_RADIO_MODE = 68,
    NRF_RADIO_PCNF0 = 69,
    NRF_RADIO_PCNF1 = 70,
    NRF_RADIO_BASE0 = 71,
    NRF_RADIO_BASE1 = 72,
    NRF_RADIO_PREFIX0 = 73,
    NRF_RADIO_PREFIX1 = 74,
    NRF_RADIO_TXADDRESS = 75,
    NRF_RADIO_RXADDRESSES = 76,
    NRF_RADIO_CRCCNF = 77,
    NRF_RADIO_CRCPOLY = 78,
    NRF_RADIO_CRCINIT = 79,
    NRF_RADIO_TEST = 80,
    NRF_RADIO_TIFS = 81,
    NRF_RADIO_RSSISAMPLE = 82,
    NRF_RADIO_STATE = 84,
    NRF_RADIO_DATAWHITEIV = 85,
    NRF_RADIO_BCC = 88,

    NRF_RADIO_DAB_0 = 128,
    NRF_RADIO_DAB_1 = 129,
    NRF_RADIO_DAB_2 = 130,
    NRF_RADIO_DAB_3 = 131,
    NRF_RADIO_DAB_4 = 132,
    NRF_RADIO_DAB_5 = 133,
    NRF_RADIO_DAB_6 = 134,
    NRF_RADIO_DAB_7 = 135,
#define NRF_RADIO_DAB(x) (NRF_RADIO_DAB_0 + (x))

    NRF_RADIO_DAP_0 = 136,
    NRF_RADIO_DAP_1 = 137,
    NRF_RADIO_DAP_2 = 138,
    NRF_RADIO_DAP_3 = 139,
    NRF_RADIO_DAP_4 = 140,
    NRF_RADIO_DAP_5 = 141,
    NRF_RADIO_DAP_6 = 142,
    NRF_RADIO_DAP_7 = 143,
#define NRF_RADIO_DAP(x) (NRF_RADIO_DAP_0 + (x))

    NRF_RADIO_DACNF = 144,

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
    NRF_RADIO_MODECNF0 = 148,
#endif

    NRF_RADIO_OVERRIDE_0 = 201,
    NRF_RADIO_OVERRIDE_1 = 202,
    NRF_RADIO_OVERRIDE_2 = 203,
    NRF_RADIO_OVERRIDE_3 = 204,
    NRF_RADIO_OVERRIDE_4 = 205,
#define NRF_RADIO_OVERRIDE(x) (NRF_RADIO_OVERRIDE_0 + (x))

    NRF_RADIO_POWER = 767,
};

enum nrf5x_radio_shorts {
    NRF_RADIO_READY_START = 0,
    NRF_RADIO_END_DISABLE,
    NRF_RADIO_DISABLED_TXEN,
    NRF_RADIO_DISABLED_RXEN,
    NRF_RADIO_ADDRESS_RSSISTART,
    NRF_RADIO_END_START,
    NRF_RADIO_ADDRESS_BCSTART,
    NRF_RADIO_DISABLED_RSSISTOP = 8,
};

#define NRF_RADIO_CRCSTATUS_ERROR 0
#define NRF_RADIO_CRCSTATUS_OK 1

#define NRF_RADIO_MODE_NRF_1MBIT 0
#define NRF_RADIO_MODE_NRF_2MBIT 1
#define NRF_RADIO_MODE_NRF_250_KBIT 2
#define NRF_RADIO_MODE_BLE_1MBIT 3
#define NRF_RADIO_MODE_BLE_2MBIT 4
#define NRF_RADIO_MODE_BLE_LR125K 5
#define NRF_RADIO_MODE_BLE_LR500K 6
#define NRF_RADIO_MODE_802_15_4_250K 15

#define NRF_RADIO_CRCCNF_LEN_MASK 0x3
#define NRF_RADIO_CRCCNF_LEN_OFFSET 0
#define NRF_RADIO_CRCCNF_SKIPADDR 0x100
#define NRF_RADIO_CRCCNF_802_15_4 0x200

#define NRF_RADIO_PCNF0_LFLEN_MASK 0xf
#define NRF_RADIO_PCNF0_LFLEN_OFFSET 0

#define NRF_RADIO_PCNF0_PLEN_MASK (3 << 24)
#define NRF_RADIO_PCNF0_PLEN_8 (0 << 24)
#define NRF_RADIO_PCNF0_PLEN_16 (1 << 24)
#define NRF_RADIO_PCNF0_PLEN_32 (2 << 24)
#define NRF_RADIO_PCNF0_PLEN_LR (3 << 24)

#define NRF_RADIO_PCNF0_CRCINC_MASK    (1 << 26)
#define NRF_RADIO_PCNF0_CRCINC_INCLUDE (1 << 26)

#define NRF_RADIO_PCNF0_S0LEN_MASK 0x100
#define NRF_RADIO_PCNF0_S0LEN_OFFSET 8

#define NRF_RADIO_PCNF0_CILEN_MASK 0xc00000
#define NRF_RADIO_PCNF0_CILEN(x) (((x) & 0x3) << 22)

#define NRF_RADIO_PCNF0_TERMLEN_MASK 0x60000000
#define NRF_RADIO_PCNF0_TERMLEN(x) (((x) & 0x3) << 29)

#define NRF_RADIO_PCNF0_S1LEN_MASK 0xf0000
#define NRF_RADIO_PCNF0_S1LEN_OFFSET 16

#define NRF_RADIO_PCNF1_MAXLEN_MASK 0xff
#define NRF_RADIO_PCNF1_MAXLEN_OFFSET 0

#define NRF_RADIO_PCNF1_STATLEN_MASK 0xff00
#define NRF_RADIO_PCNF1_STATLEN_OFFSET 8

#define NRF_RADIO_PCNF1_BALEN_MASK 0x70000
#define NRF_RADIO_PCNF1_BALEN_OFFSET 16

#define NRF_RADIO_PCNF1_ENDIAN_MASK (1 << 24)
#define NRF_RADIO_PCNF1_ENDIAN_LITTLE 0
#define NRF_RADIO_PCNF1_ENDIAN_BIG (1 << 24)
#define NRF_RADIO_PCNF1_ENDIAN_OFFSET 24

#define NRF_RADIO_PCNF1_WHITEEN_MASK (1 << 25)
#define NRF_RADIO_PCNF1_WHITEEN_DISABLED 0
#define NRF_RADIO_PCNF1_WHITEEN_ENABLED (1 << 25)
#define NRF_RADIO_PCNF1_WHITEEN_OFFSET 25

#define NRF_RADIO_TEST_CONSTCARRIER (1 << 0)
#define NRF_RADIO_TEST_PLLLOCK (1 << 1)

#define NRF_RADIO_PREFIX0_AP0_MASK 0xff
#define NRF_RADIO_PREFIX0_AP0_OFFSET 0
#define NRF_RADIO_PREFIX0_AP1_MASK 0xff00
#define NRF_RADIO_PREFIX0_AP1_OFFSET 8
#define NRF_RADIO_PREFIX0_AP2_MASK 0xff0000
#define NRF_RADIO_PREFIX0_AP2_OFFSET 16
#define NRF_RADIO_PREFIX0_AP3_MASK 0xff000000
#define NRF_RADIO_PREFIX0_AP3_OFFSET 24
#define NRF_RADIO_PREFIX1_AP4_MASK 0xff
#define NRF_RADIO_PREFIX1_AP4_OFFSET 0
#define NRF_RADIO_PREFIX1_AP5_MASK 0xff00
#define NRF_RADIO_PREFIX1_AP5_OFFSET 8
#define NRF_RADIO_PREFIX1_AP6_MASK 0xff0000
#define NRF_RADIO_PREFIX1_AP6_OFFSET 16
#define NRF_RADIO_PREFIX1_AP7_MASK 0xff000000
#define NRF_RADIO_PREFIX1_AP7_OFFSET 24

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
#define NRF_RADIO_MODECNF0_RU_NORMAL 0x0
#define NRF_RADIO_MODECNF0_RU_FAST 0x1
#define NRF_RADIO_MODECNF0_DTX_B1 0x000
#define NRF_RADIO_MODECNF0_DTX_B0 0x100
#define NRF_RADIO_MODECNF0_DTX_CENTER 0x200
#endif

enum nrf5x_radio_state {
  NRF_RADIO_STATE_DISABLED = 0,
  NRF_RADIO_STATE_RXRU = 1,
  NRF_RADIO_STATE_RXIDLE = 2,
  NRF_RADIO_STATE_RX = 3,
  NRF_RADIO_STATE_RXDISABLE = 4,
  NRF_RADIO_STATE_TXRU = 9,
  NRF_RADIO_STATE_TXIDLE = 10,
  NRF_RADIO_STATE_TX = 11,
  NRF_RADIO_STATE_TXDISABLE = 12,
};

#define NRF_RADIO_DACNF_ENA(x) (1 << (x))
#define NRF_RADIO_DACNF_TXADD(x) (1 << ((x) + 8))

#endif
