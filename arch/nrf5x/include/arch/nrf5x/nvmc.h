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

#ifndef ARCH_NRF_NVMC_H_
#define ARCH_NRF_NVMC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_nvmc_register {
    NRF_NVMC_READY = 0,
    NRF_NVMC_CONFIG = 65,
    NRF_NVMC_ERASEPAGE = 66,
    NRF_NVMC_ERASEPCR1 = 66,
    NRF_NVMC_ERASEALL = 67,
    NRF_NVMC_ERASEPCR0 = 68,
    NRF_NVMC_ERASEUICR = 69,
    NRF_NVMC_ICACHECNF = 80,
    NRF_NVMC_IHIT = 82,
    NRF_NVMC_IMISS = 83,
};

#define NRF_NVMC_CONFIG_WRITE (1 << 0)
#define NRF_NVMC_CONFIG_ERASE (1 << 1)

#define NRF_NVMC_ICACHECNF_CACHEEN_ENABLED (1 << 0)
#define NRF_NVMC_ICACHECNF_CACHEEN_DISABLED (0 << 0)
#define NRF_NVMC_ICACHECNF_CACHEPROFEN_ENABLED (1 << 8)
#define NRF_NVMC_ICACHECNF_CACHEPROFEN_DISABLED (0 << 8)

size_t nrf5x_flash_page_size(void);
size_t nrf5x_flash_page_count(void);
void nrf5x_flash_page_erase(uintptr_t page);
void nrf5x_flash_write(uintptr_t dest, const void *data, size_t word_count);

#endif
