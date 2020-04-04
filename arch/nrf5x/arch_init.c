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
#include <hexo/flash.h>

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
  return 0
    | ((cpu_mem_read_8(0xf0000fe4) & 0xf) << 12) // Part ID 1
    | (cpu_mem_read_8(0xf0000fe0) << 8) // Part ID 0
    | (cpu_mem_read_8(0xf0000fec) >> 4) // RevAnd
    | (cpu_mem_read_8(0xf0000fe8) & 0xf0) // Revision number
    ;
}

#define NRF51x22_G0   0x00140   // nRF51x22 QFAAG0_1407 r2
#define NRF51x22_H0   0x00170   // nRF51x22 QFAAH0_1513 r3
#define NRF51x22_A1   0x00190   // nRF51x22 QFACA1_1503 r3
#define NRF52832_ENGA 0x00630   // nRF52832 QFAAAA_xxxx engA
#define NRF52832_ENGB 0x00640   // nRF52832 QFAABA_1536 engB
#define NRF52832_R1   0x00650   // nRF52832 QFAAB0_1614 r1
#define NRF52840_ENGA 0x00800   // nRF52840 QIAAAA_1644 engA

void nrf5x_init(void)
{
#if CONFIG_NRF5X_MODEL >= 52000 && \
    CONFIG_NRF5X_MODEL < 53000


  uintptr_t nvmc = NRF_PERIPHERAL_ADDR(NRF5X_NVMC);
  nrf_reg_set(nvmc, NRF_NVMC_ICACHECNF, NRF_NVMC_ICACHECNF_CACHEEN_ENABLED);

  if (nrf_build_tag() < NRF52832_ENGB) {
    // FTPAN 32
    uintptr_t demcr = 0xe000edfc;
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
  nrf_event_clear(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_DONE);
  nrf_event_clear(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_CTTO);

  // PAN-108
  cpu_mem_write_32(0x40000ee4, cpu_mem_read_32(0x10000258));
#endif
}

#if CONFIG_NRF5X_MODEL == 52840

__attribute__((section(".uicr")))
const uint32_t uicr[0x400] = {
  [0 ... 0x3ff] = 0xffffffff,
# ifdef CONFIG_NRF5X_VREGH
  [0x304/4] = (CONFIG_NRF5X_VREGH / 3) - 6,
# endif  
};

#endif

