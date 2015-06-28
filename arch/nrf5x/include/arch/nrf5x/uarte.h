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

#ifndef ARCH_NRF_UARTE_H_
#define ARCH_NRF_UARTE_H_

#include "peripheral.h"
#include "ids.h"

enum nrf_uarte_task {
    NRF_UARTE_STARTRX = 0,
    NRF_UARTE_STOPRX = 1,
    NRF_UARTE_STARTTX = 2,
    NRF_UARTE_STOPTX = 3,
    NRF_UARTE_FLUSHRX = 11,
};

enum nrf_uarte_event {
    NRF_UARTE_CTS = 0,
    NRF_UARTE_NCTS = 1,
    NRF_UARTE_ENDRX = 4,
    NRF_UARTE_ENDTX = 8,
    NRF_UARTE_ERROR = 9,
    NRF_UARTE_RXTO = 17,
    NRF_UARTE_RXSTARTED = 19,
    NRF_UARTE_TXSTARTED = 20,
    NRF_UARTE_TXSTOPPED = 22,
};

enum nrf_uarte_shorts {
    NRF_UARTE_ENDRX_STARTRX = 5,
    NRF_UARTE_ENDRX_STOPRX = 6,
};

enum nrf_uarte_register {
    NRF_UARTE_ERRORSRC = 32,
    NRF_UARTE_ENABLE = 64,
    NRF_UARTE_PSEL_RTS = 66,
    NRF_UARTE_PSEL_TXD = 67,
    NRF_UARTE_PSEL_CTS = 68,
    NRF_UARTE_PSEL_RXD = 69,
    NRF_UARTE_BAUDRATE = 73,
    NRF_UARTE_RXD_PTR = 77,
    NRF_UARTE_RXD_MAXCNT = 78,
    NRF_UARTE_RXD_AMOUNT = 79,
    NRF_UARTE_TXD_PTR = 81,
    NRF_UARTE_TXD_MAXCNT = 82,
    NRF_UARTE_TXD_AMOUNT = 83,
    NRF_UARTE_CONFIG = 91,
};

#define NRF_UARTE_ERRORSRC_OVERRUN 0x1
#define NRF_UARTE_ERRORSRC_PARITY  0x2
#define NRF_UARTE_ERRORSRC_FRAMING 0x4
#define NRF_UARTE_ERRORSRC_BREAK   0x8

#define NRF_UARTE_CONFIG_CTSRTS_MASK   0x1
#define NRF_UARTE_CONFIG_CTSRTS_DISABLED   0x0
#define NRF_UARTE_CONFIG_CTSRTS_ENABLED   0x1

#define NRF_UARTE_CONFIG_PARITY_MASK  0xe
#define NRF_UARTE_CONFIG_PARITY_DISABLED  0x0
#define NRF_UARTE_CONFIG_PARITY_ENABLED  0xe

#define NRF_UARTE_ENABLE_MASK   0xf
#define NRF_UARTE_ENABLE_DISABLED   0x0
#define NRF_UARTE_ENABLE_ENABLED   0x8

#define NRF_UARTE_BAUDRATE_(x)  ((x) * 268)

#endif
