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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014-2020
*/

#ifndef ARCH_NRF_TWIS_H_
#define ARCH_NRF_TWIS_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_twis_task {
    NRF_TWIS_STOP = 5,
    NRF_TWIS_SUSPEND = 7,
    NRF_TWIS_RESUME = 8,
    NRF_TWIS_PREPARERX = 12,
    NRF_TWIS_PREPARETX = 13,
};

enum nrf5x_twis_event {
    NRF_TWIS_STOPPED = 1,
    NRF_TWIS_ERROR = 9,
    NRF_TWIS_RXSTARTED = 19,
    NRF_TWIS_TXSTARTED = 20,
    NRF_TWIS_WRITE = 25,
    NRF_TWIS_READ = 26,
};

enum nrf5x_twis_shorts {
    NRF_TWIS_WRITE_SUSPEND = 13,
    NRF_TWIS_READ_SUSPEND = 14,
};

enum nrf5x_twis_register {
    NRF_TWIS_ERRORSRC = 52,
    NRF_TWIS_MATCH = 53,
    NRF_TWIS_ENABLE = 64,
    NRF_TWIS_PSEL_SCL = 66,
    NRF_TWIS_PSEL_SDA = 67,
    NRF_TWIS_RXD_PTR = 77,
    NRF_TWIS_RXD_MAXCOUNT = 78,
    NRF_TWIS_RXD_AMOUNT = 79,
    NRF_TWIS_TXD_PTR = 81,
    NRF_TWIS_TXD_MAXCOUNT = 82,
    NRF_TWIS_TXD_AMOUNT = 83,
    NRF_TWIS_ADDR0 = 98,
    NRF_TWIS_ADDR1 = 99,
    NRF_TWIS_CONFIG = 101,
    NRF_TWIS_ORC = 112,

    NRF_TWIS_POWER = 767, // See PAN-56
};

#define NRF_TWIS_ERRORSRC_OVERFLOW 0x1
#define NRF_TWIS_ERRORSRC_DNACK  0x4
#define NRF_TWIS_ERRORSRC_OVERREAD  0x8

#define NRF_TWIS_CONFIG_ADDR0 1
#define NRF_TWIS_CONFIG_ADDR1 2

#define NRF_TWIS_ENABLE_MASK       0xf
#define NRF_TWIS_ENABLE_DISABLED   0x0
#define NRF_TWIS_ENABLE_ENABLED    0x9

#endif
