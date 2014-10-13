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

#ifndef ARCH_NRF_SPIS_H_
#define ARCH_NRF_SPIS_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_spis_task {
    NRF_SPIS_ACQUIRE = 9,
    NRF_SPIS_RELEASE = 10,
};

enum nrf5x_spis_event {
    NRF_SPIS_END = 2,
    NRF_SPIS_ACQUIRED = 10,
};

enum nrf5x_spis_register {
    NRF_SPIS_SEMSTAT = 0,
    NRF_SPIS_STATUS = 16,
    NRF_SPIS_ENABLE = 64,
    NRF_SPIS_PSELSCK = 66,
    NRF_SPIS_PSELMISO = 67,
    NRF_SPIS_PSELMOSI = 68,
    NRF_SPIS_PSELCSN = 69,
    NRF_SPIS_RXDPTR = 77,
    NRF_SPIS_MAXRX = 78,
    NRF_SPIS_AMOUNTRX = 79,
    NRF_SPIS_TXDPTR = 81,
    NRF_SPIS_MAXRX = 82,
    NRF_SPIS_AMOUNTRX = 83,
    NRF_SPIS_CONFIG = 85,
    NRF_SPIS_DEF = 87,
    NRF_SPIS_ORC = 112,
};

#define NRF_SPIS_SEMSTAT_FREE 0
#define NRF_SPIS_SEMSTAT_CPU 1
#define NRF_SPIS_SEMSTAT_SPIS 2
#define NRF_SPIS_SEMSTAT_CPUPENDING 3

#define NRF_SPIS_OVERREAD 1
#define NRF_SPIS_OVERFLOW 2

#define NRF_SPIS_ENABLE_MASK       0xf
#define NRF_SPIS_ENABLE_DISABLED   0x0
#define NRF_SPIS_ENABLE_ENABLED    0x2

#define NRF_SPIS_CONFIG_ORDER_MSBFIRST 0
#define NRF_SPIS_CONFIG_ORDER_LSBFIRST 1
#define NRF_SPIS_CONFIG_CPHA_LEADING 0
#define NRF_SPIS_CONFIG_CPHA_TRAILING 1
#define NRF_SPIS_CONFIG_CPOL_HIGH 0
#define NRF_SPIS_CONFIG_CPOL_LOW 1

#endif
