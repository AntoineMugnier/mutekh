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

#ifndef ARCH_NRF51_NVMC_H_
#define ARCH_NRF51_NVMC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_nvmc_register {
    NRF51_NVMC_READY = 0,
    NRF51_NVMC_CONFIG = 65,
    NRF51_NVMC_ERASEPAGE = 66,
    NRF51_NVMC_ERASEALL = 67,
    NRF51_NVMC_ERASEPCR0 = 68,
    NRF51_NVMC_ERASEUICR = 69,
};

#define NRF51_NVMC_CONFIG_WRITE (1 << 0)
#define NRF51_NVMC_CONFIG_ERASE (1 << 1)

#endif
