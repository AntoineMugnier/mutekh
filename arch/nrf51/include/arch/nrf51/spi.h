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

#ifndef ARCH_NRF51_SPI_H_
#define ARCH_NRF51_SPI_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_spi_event {
    NRF51_SPI_READY = 2,
};

enum nrf51_spi_register {
    NRF51_SPI_ENABLE = 64,
    NRF51_SPI_PSELSCK = 66,
    NRF51_SPI_PSELMOSI = 67,
    NRF51_SPI_PSELMISO = 68,
    NRF51_SPI_RXD = 70,
    NRF51_SPI_TXD = 71,
    NRF51_SPI_FREQUENCY = 73,
    NRF51_SPI_CONFIG = 85,
};

#define NRF51_SPI_CONFIG_ORDER_MASK      0x1
#define NRF51_SPI_CONFIG_ORDER_MSBFIRST  0x0
#define NRF51_SPI_CONFIG_ORDER_LSBFIRST  0x1

#define NRF51_SPI_CONFIG_CPOL_MASK       0x2
#define NRF51_SPI_CONFIG_CPOL_ACTIVEHIGH 0x0
#define NRF51_SPI_CONFIG_CPOL_ACTIVELOW  0x2

#define NRF51_SPI_CONFIG_CPHA_MASK       0x4
#define NRF51_SPI_CONFIG_CPHA_LEADING    0x0
#define NRF51_SPI_CONFIG_CPHA_TRAILING   0x4

#define NRF51_SPI_ENABLE_MASK       0x7
#define NRF51_SPI_ENABLE_DISABLED   0x0
#define NRF51_SPI_ENABLE_ENABLED    0x1

#define NRF51_SPI_FREQUENCY_(x)  ((x) * 268)

#endif
