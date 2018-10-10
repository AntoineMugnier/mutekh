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

#include <hexo/flash.h>
#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <arch/nrf5x/nvmc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/uicr.h>

#define NVMC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_NVMC)

reg_t flash_page_erase(uintptr_t page)
{
#if CONFIG_NRF5X_MODEL <= 51999
  uint32_t cr0_len = cpu_mem_read_32(NRF_FICR_CLENR0);
  if (cr0_len == 0xffffffff)
    cr0_len = 0;
  uint8_t reg = page < cr0_len ? NRF_NVMC_ERASEPCR0 : NRF_NVMC_ERASEPCR1;
#else
  uint8_t reg = NRF_NVMC_ERASEPAGE;
#endif

#if CONFIG_NRF5X_MODEL == 52840
  if (page == NRF_UICR_BASE) {
    reg = NRF_NVMC_ERASEUICR;
    page = 0x1;
  }
#endif

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, NRF_NVMC_CONFIG_ERASE);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  nrf_reg_set(NVMC_ADDR, reg, page);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, 0);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

reg_t flash_page_write(uintptr_t dest, const uint8_t *data, size_t size)
{
  uint32_t i;
  volatile uint32_t *dst = (volatile uint32_t *)dest;
  const uint32_t *src = (void*)data;

  if (size & 3)
    return 1;

  size_t word_count = size >> 2;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, NRF_NVMC_CONFIG_WRITE);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  if (((uintptr_t)src & 3) == 0) {
    for (i = 0; i < word_count; ++i)
      dst[i] = src[i];
  } else {
    for (i = 0; i < word_count; ++i)
      dst[i] = endian_32_na_load(src + i);
  }

  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, 0);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

