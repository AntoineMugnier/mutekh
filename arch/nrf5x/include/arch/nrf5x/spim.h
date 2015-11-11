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

#ifndef ARCH_NRF_SPIM_H_
#define ARCH_NRF_SPIM_H_

#include "peripheral.h"
#include "ids.h"

enum nrf_spim_task {
  NRF_SPIM_START = 4,
  NRF_SPIM_STOP = 5,
  NRF_SPIM_SUSPEND = 7,
  NRF_SPIM_RESUME = 8,
};

enum nrf_spim_event {
  NRF_SPIM_STOPPED = 1,
  NRF_SPIM_ENDRX = 4,
  NRF_SPIM_END = 6,
  NRF_SPIM_ENDTX = 8,
  NRF_SPIM_STARTED = 19,
};

enum nrf_spim_short {
  NRF_SPIM_END_START = 17,
};

enum nrf_spim_register {
  NRF_SPIM_ENABLE = 64,
  NRF_SPIM_PSEL_SCK = 66,
  NRF_SPIM_PSEL_MOSI = 67,
  NRF_SPIM_PSEL_MISO = 68,
  NRF_SPIM_FREQUENCY = 73,
  NRF_SPIM_RXD_PTR = 77,
  NRF_SPIM_RXD_MAXCNT = 78,
  NRF_SPIM_RXD_AMOUNT = 79,
  NRF_SPIM_RXD_LIST = 80,
  NRF_SPIM_TXD_PTR = 81,
  NRF_SPIM_TXD_MAXCNT = 82,
  NRF_SPIM_TXD_AMOUNT = 83,
  NRF_SPIM_TXD_LIST = 84,
  NRF_SPIM_CONFIG = 85,
  NRF_SPIM_ORC = 112,
};

#define NRF_SPIM_CONFIG_ORDER_MASK      0x1
#define NRF_SPIM_CONFIG_ORDER_MSBFIRST  0x0
#define NRF_SPIM_CONFIG_ORDER_LSBFIRST  0x1

#define NRF_SPIM_CONFIG_CPHA_MASK       0x2
#define NRF_SPIM_CONFIG_CPHA_LEADING    0x0
#define NRF_SPIM_CONFIG_CPHA_TRAILING   0x2

#define NRF_SPIM_CONFIG_CPOL_MASK       0x4
#define NRF_SPIM_CONFIG_CPOL_ACTIVEHIGH 0x0
#define NRF_SPIM_CONFIG_CPOL_ACTIVELOW  0x4

#define NRF_SPIM_ENABLE_MASK       0xf
#define NRF_SPIM_ENABLE_DISABLED   0x0
#define NRF_SPIM_ENABLE_ENABLED    0x7

#define NRF_SPIM_RXD_LIST_DISABLED 0
#define NRF_SPIM_RXD_LIST_ARRAYLIST 1

#define NRF_SPIM_TXD_LIST_DISABLED 0
#define NRF_SPIM_TXD_LIST_ARRAYLIST 1

#define NRF_SPIM_FREQUENCY_(x)  ((x) * 268)

#endif
