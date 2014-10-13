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

#ifndef ARCH_NRF_I2C_H_
#define ARCH_NRF_I2C_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_i2c_shorts {
    NRF_I2C_BB_SUSPEND = 0,
    NRF_I2C_BB_STOP = 1,
};

enum nrf5x_i2c_task {
    NRF_I2C_STARTRX = 0,
    NRF_I2C_UNDOC_TASK1 = 1,
    NRF_I2C_STARTTX = 2,
    NRF_I2C_UNDOC_TASK3 = 3,
    NRF_I2C_UNDOC_TASK4 = 4,
    NRF_I2C_STOP = 5,
    NRF_I2C_SUSPEND = 7,
    NRF_I2C_RESUME = 8,
    NRF_I2C_UNDOC_TASK9 = 9,
    NRF_I2C_UNDOC_TASK10 = 10,
};

enum nrf5x_i2c_event {
    NRF_I2C_UNDOC_EVENT0 = 0,
    NRF_I2C_STOPPED = 1,
    NRF_I2C_RXDRDY = 2,
    NRF_I2C_UNDOC_EVENT3 = 3,
    NRF_I2C_UNDOC_EVENT4 = 4,
    NRF_I2C_UNDOC_EVENT5 = 5,
    NRF_I2C_UNDOC_EVENT6 = 6,
    NRF_I2C_TXDSENT = 7,
    NRF_I2C_UNDOC_EVENT8 = 8,
    NRF_I2C_ERROR = 9,
    NRF_I2C_UNDOC_EVENT10 = 10,
    NRF_I2C_UNDOC_EVENT11 = 11,
    NRF_I2C_UNDOC_EVENT12 = 12,
    NRF_I2C_UNDOC_EVENT13 = 13,
    NRF_I2C_BB = 14,
    NRF_I2C_UNDOC_EVENT15 = 15,
    NRF_I2C_UNDOC_EVENT16 = 16,
    NRF_I2C_SUSPENDED = 18, // ???
};

enum nrf5x_i2c_register {
    NRF_I2C_ERRORSRC = 49,
    NRF_I2C_ENABLE = 64,
    NRF_I2C_PSELSCL = 66,
    NRF_I2C_PSELSDA = 67,
    NRF_I2C_RXD = 70,
    NRF_I2C_TXD = 71,
    NRF_I2C_FREQUENCY = 73,
    NRF_I2C_ADDR = 98,

    NRF_I2C_POWER = 767, // See PAN-56
};

#define NRF_I2C_ERRORSRC_OVERRUN 0x1
#define NRF_I2C_ERRORSRC_ANACK  0x2
#define NRF_I2C_ERRORSRC_DNACK  0x4

#define NRF_I2C_ENABLE_MASK       0xf
#define NRF_I2C_ENABLE_DISABLED   0x0
#define NRF_I2C_ENABLE_ENABLED    0x5

#define NRF_I2C_FREQUENCY_(x)  ((x) * 268)

#endif
