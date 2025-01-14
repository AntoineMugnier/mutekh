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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#ifndef ARCH_NRF_NFCT_H_
#define ARCH_NRF_NFCT_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_ccm_task {
    NRF_NFCT_ACTIVATE = 0,
    NRF_NFCT_DISABLE = 1,
    NRF_NFCT_SENSE = 2,
    NRF_NFCT_STARTTX = 3,
    NRF_NFCT_ENABLERXDATA = 7,
    NRF_NFCT_GOIDLE = 9,
    NRF_NFCT_GOSLEEP = 10,
};

enum nrf5x_ccm_event {
    NRF_NFCT_READY = 0,
    NRF_NFCT_FIELDDETECT = 1,
    NRF_NFCT_FIELDLOST = 2,
    NRF_NFCT_TXFRAMESTART = 3,
    NRF_NFCT_TXFRAMEEND = 4,
    NRF_NFCT_RXFRAMESTART = 5,
    NRF_NFCT_RXFRAMEEND = 6,
    NRF_NFCT_ERROR = 7,
    NRF_NFCT_RXERROR = 10,
    NRF_NFCT_ENDRX = 11,
    NRF_NFCT_ENDTX = 12,
    NRF_NFCT_AUTOCOLRESST = 14,
    NRF_NFCT_COLLISION = 18,
    NRF_NFCT_SELECTED = 19,
    NRF_NFCT_STARTED = 20,
};

enum nrf5x_ccm_short {
    NRF_NFCT_FIELDDETECT_ACTIVATE = 0,
    NRF_NFCT_FIELDLOST_SENSE = 1,
    NRF_NFCT_TXFRAMEEND_ENABLERXDATA = 5,
};

enum nrf5x_ccm_register {
  NRF_NFCT_ERRORSTATUS = 1,
  NRF_NFCT_FRAMESTATUS_RX = 3,
  NRF_NFCT_CURRENTLOADCTRL = 12,
  NRF_NFCT_FIELDPRESENT    = 15,
  NRF_NFCT_FRAMEDELAYMIN   = 65,
  NRF_NFCT_FRAMEDELAYMAX   = 66,
  NRF_NFCT_FRAMEDELAYMODE  = 67,
  NRF_NFCT_PACKETPTR       = 68,
  NRF_NFCT_MAXLEN          = 69,
  NRF_NFCT_TXD_FRAMECONFIG = 70,
  NRF_NFCT_TXD_AMOUNT      = 71,
  NRF_NFCT_RXD_FRAMECONFIG = 72,
  NRF_NFCT_RXD_AMOUNT      = 73,
  NRF_NFCT_NFCID1_LAST     = 100,
  NRF_NFCT_NFCID1_2ND_LAST = 101,
  NRF_NFCT_NFCID1_3RD_LAST = 102,
  NRF_NFCT_SENSRES         = 104,
  NRF_NFCT_SELRES          = 105,
};

#define NRF_NFCT_ERRORSTATUS_FRAMEDELAYTIMEOUT 1
#define NRF_NFCT_ERRORSTATUS_NFCFIELDTOOSTRONG 4
#define NRF_NFCT_ERRORSTATUS_NFCFIELDTOOWEAK   8

#define NRF_NFCT_FRAMESTATUS_RX_CRCERROR     1
#define NRF_NFCT_FRAMESTATUS_RX_PARITYSTATUS 4
#define NRF_NFCT_FRAMESTATUS_RX_OVERRUN      8

#define NRF_NFCT_FIELDPRESENT_FIELDPRESENT 1
#define NRF_NFCT_FIELDPRESENT_LOCKDETECT   2

#define NRF_NFCT_FRAMEDELAYMODE_FREERUN    0
#define NRF_NFCT_FRAMEDELAYMODE_WINDOW     1
#define NRF_NFCT_FRAMEDELAYMODE_EXACTVAL   2
#define NRF_NFCT_FRAMEDELAYMODE_WINDOWGRID 3

#define NRF_NFCT_TXD_FRAMECONFIG_PARITY        1
#define NRF_NFCT_TXD_FRAMECONFIG_DISCARD_END   0
#define NRF_NFCT_TXD_FRAMECONFIG_DISCARD_START 2
#define NRF_NFCT_TXD_FRAMECONFIG_SOF           4
#define NRF_NFCT_TXD_FRAMECONFIG_CRC16TX       16

#define NRF_NFCT_RXD_FRAMECONFIG_PARITY        1
#define NRF_NFCT_RXD_FRAMECONFIG_SOF           4
#define NRF_NFCT_RXD_FRAMECONFIG_CRC16RX       16

#define NRF_NFCT_SENSRES_BITFRAMESDD_00000 (0  << 0)
#define NRF_NFCT_SENSRES_BITFRAMESDD_00001 (1  << 0)
#define NRF_NFCT_SENSRES_BITFRAMESDD_00010 (2  << 0)
#define NRF_NFCT_SENSRES_BITFRAMESDD_00100 (4  << 0)
#define NRF_NFCT_SENSRES_BITFRAMESDD_01000 (8  << 0)
#define NRF_NFCT_SENSRES_BITFRAMESDD_10000 (16 << 0)
#define NRF_NFCT_SENSRES_NFCIDSIZE_SINGLE  (0 << 6)
#define NRF_NFCT_SENSRES_NFCIDSIZE_DOUBLE  (1 << 6)
#define NRF_NFCT_SENSRES_NFCIDSIZE_TRIPLE  (2 << 6)
#define NRF_NFCT_SENSRES_PLATCONFIG(x)     ((x) << 8)

#define NRF_NFCT_SELRES_CASCASE 4
#define NRF_NFCT_SELRES_PROTOCOL(x)     ((x) << 5)

#endif
