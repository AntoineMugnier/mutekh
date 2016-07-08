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

#include <mutek/startup.h>

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

#include <arch/nrf5x/nvmc.h>
#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/gpio.h>
#include <arch/nrf5x/peripheral.h>

#if defined(CONFIG_ARCH_NRF52)
static void nrf52_icache_init(void)
{
  uintptr_t nvmc = NRF_PERIPHERAL_ADDR(NRF5X_NVMC);

  nrf_reg_set(nvmc, NRF_NVMC_ICACHECNF, NRF_NVMC_ICACHECNF_CACHEEN_ENABLED);
}
#endif

#if defined(CONFIG_ARCH_NRF52)
static bool_t is_nrf52_0(void)
{
  // Nothing else exists for now
  return 1;

  static const uint8_t const mask[]     = {0xff, 0x0f, 0xf0, 0xf0};
  static const uint8_t const expected[] = {0x06, 0x00, 0x30, 0x00};
  const uint32_t *base = (const uint32_t*)0xf0000fe0;

  for (uint8_t i = 0; i < ARRAY_SIZE(mask); ++i) {
    if ((base[i] & mask[i]) != expected[i])
      return 0;
  }

  return 1;
}

void nrf52_init(void)
{
  uintptr_t clock = NRF_PERIPHERAL_ADDR(NRF5X_CLOCK);
  uintptr_t demcr = 0xe000edfc;

  nrf52_icache_init();

  if (is_nrf52_0()) {
    // FTPAN 32
    cpu_mem_write_32(demcr, cpu_mem_read_32(demcr) & ~0x01000000);

    // FTPAN 37
    *(volatile uint32_t *)0x400005A0 = 0x3;

    // PAN 16
    *(uint32_t *)0x4007C074 = 0xbaadf00d;

    // PAN 31
    // CLOCK: Calibration values are not correctly loaded from FICR at reset
    *(volatile uint32_t *)0x4000053C = ((*(volatile uint32_t *)0x10000244) & 0x0000E000) >> 13;

    // FTPAN 36
    nrf_event_clear(clock, NRF_CLOCK_DONE);
    nrf_event_clear(clock, NRF_CLOCK_CTTO);
  }
}
#endif
