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
#include <arch/nrf5x/uicr.h>
#include <arch/nrf5x/peripheral.h>
#include <cpu/arm32m/v7m.h>

__unused__
static uint32_t nrf_build_tag(void)
{
  return (cpu_mem_read_8(0xf0000fe0) << 12)
    | ((cpu_mem_read_8(0xf0000fe4) & 0xf) << 8)
    | (cpu_mem_read_8(0xf0000fe8) & 0xf0)
    | (cpu_mem_read_8(0xf0000fec) >> 4);
}

#define NRF51_G0 0x01040 // QFAAG0_1407 r2
#define NRF51_H0 0x01070 // QFAAH0_1513 r3
#define NRF51_A1 0x01090 // QFACA1_1503 r3

#define NRF52_ENGA 0x06030 // QFAAAA_xxxx engA
#define NRF52_ENGB 0x06040 // QFAABA_1536 engB
#define NRF52_R1   0x06050 // QFAAB0_1614 r1

#if defined(CONFIG_ARCH_NRF52)
static void nrf52_icache_init(void)
{
  uintptr_t nvmc = NRF_PERIPHERAL_ADDR(NRF5X_NVMC);

  nrf_reg_set(nvmc, NRF_NVMC_ICACHECNF, NRF_NVMC_ICACHECNF_CACHEEN_ENABLED);
}
#endif

#if defined(CONFIG_ARCH_NRF52)
void nrf52_init(void)
{
  uintptr_t clock = NRF_PERIPHERAL_ADDR(NRF5X_CLOCK);
  uintptr_t demcr = 0xe000edfc;
  uint32_t tag = nrf_build_tag();

#ifndef CONFIG_DRIVER_NRF52_NFCT
  if (cpu_mem_read_32(NRF_UICR_NFCPINS) & NRF_UICR_NFCPINS_PROTECT_NFC) {
    nrf5x_flash_write(NRF_UICR_NFCPINS, (const uint32_t[]){ ~NRF_UICR_NFCPINS_PROTECT_NFC }, 1);
    cpu_mem_write_32(ARMV7M_AIRCR_ADDR, 0x5FA0004);
  }
#endif

  nrf52_icache_init();

  if (tag < NRF52_ENGB) {
    // FTPAN 32
    cpu_mem_write_32(demcr, cpu_mem_read_32(demcr) & ~0x01000000);

    // FTPAN 37
    cpu_mem_write_32(0x400005A0, 0x3);
  }

  // PAN 16
  cpu_mem_write_32(0x4007C074, 0xbaadf00d);

  // PAN 31
  // CLOCK: Calibration values are not correctly loaded from FICR at reset
  cpu_mem_write_32(0x4000053C, (cpu_mem_read_32(0x10000244) & 0x0000E000) >> 13);

  // FTPAN 36
  nrf_event_clear(clock, NRF_CLOCK_DONE);
  nrf_event_clear(clock, NRF_CLOCK_CTTO);

  // PAN-108
  cpu_mem_write_32(0x40000ee4, cpu_mem_read_32(0x10000258));
}
#endif
