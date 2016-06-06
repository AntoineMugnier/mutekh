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

#ifndef ARCH_NRF_TWIM_H_
#define ARCH_NRF_TWIM_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_twim_shorts {
    NRF_TWIM_LASTTX_STARTRX = 7,
    NRF_TWIM_LASTTX_SUSPEND = 8,
    NRF_TWIM_LASTTX_STOP = 9,
    NRF_TWIM_LASTRX_STARTTX = 10,
    NRF_TWIM_LASTRX_STOP = 12,
};

enum nrf5x_twim_task {
    NRF_TWIM_STARTRX = 0,
    NRF_TWIM_STARTTX = 2,
    NRF_TWIM_STOP = 5,
    NRF_TWIM_SUSPEND = 7,
    NRF_TWIM_RESUME = 8,
};

enum nrf5x_twim_event {
    NRF_TWIM_STOPPED = 1,
    NRF_TWIM_ERROR = 9,
    NRF_TWIM_SUSPENDED = 18,
    NRF_TWIM_RXSTARTED = 19,
    NRF_TWIM_TXSTARTED = 20,
    NRF_TWIM_LASTRX = 23,
    NRF_TWIM_LASTTX = 24,
};

enum nrf5x_twim_register {
    NRF_TWIM_ERRORSRC = 49,
    NRF_TWIM_ENABLE = 64,
    NRF_TWIM_PSELSCL = 66,
    NRF_TWIM_PSELSDA = 67,
    NRF_TWIM_FREQUENCY = 73,
    NRF_TWIM_RXD_PTR = 77,
    NRF_TWIM_RXD_MAXCOUNT = 78,
    NRF_TWIM_RXD_AMOUNT = 79,
    NRF_TWIM_RXD_LIST = 80,
    NRF_TWIM_TXD_PTR = 81,
    NRF_TWIM_TXD_MAXCOUNT = 82,
    NRF_TWIM_TXD_AMOUNT = 83,
    NRF_TWIM_TXD_LIST = 84,
    NRF_TWIM_ADDR = 98,

    NRF_TWIM_POWER = 767, // See PAN-56
};

#define NRF_TWIM_ERRORSRC_OVERRUN 0x1
#define NRF_TWIM_ERRORSRC_ANACK  0x2
#define NRF_TWIM_ERRORSRC_DNACK  0x4

#define NRF_TWIM_ENABLE_MASK       0xf
#define NRF_TWIM_ENABLE_DISABLED   0x0
#define NRF_TWIM_ENABLE_ENABLED    0x6

#define NRF_TWIM_FREQUENCY_(x)  ((x) * 268)

#endif
