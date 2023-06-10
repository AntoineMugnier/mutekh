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

#if CONFIG_NRF5X_MODEL == 52832
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

  // PAN-108
  cpu_mem_write_32(0x40000ee4, cpu_mem_read_32(0x10000258));
#endif

  // FTPAN 36
  nrf_event_clear(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_DONE);
  nrf_event_clear(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_CTTO);
  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_CTIV, 0);

# if defined(CONFIG_CPU_ARM32M_TRACE)
  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_TRACECONFIG, 0
#  if CONFIG_CPU_ARM32M_TRACE_CLKIN_RATE == 32000000
              | NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_32MHZ
#  elif CONFIG_CPU_ARM32M_TRACE_CLKIN_RATE == 16000000
              | NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_16MHZ
#  elif CONFIG_CPU_ARM32M_TRACE_CLKIN_RATE == 8000000
              | NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_8MHZ
#  elif CONFIG_CPU_ARM32M_TRACE_CLKIN_RATE == 4000000
              | NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_4MHZ
#  else
#   error Unsupported CLKIN rate
#  endif
#  if CONFIG_CPU_ARM32M_TRACE_PARALLEL == 0
              | NRF_CLOCK_TRACECONFIG_TRACEMUX_SERIAL
#  else
              | NRF_CLOCK_TRACECONFIG_TRACEMUX_PARALLEL
#  endif
              );

  // P1.00 is TRACEDATA0 / SWO
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(32+0), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1);

#  if CONFIG_CPU_ARM32M_TRACE_PARALLEL > 0
  // P0.12 is TRACEDATA1
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(12), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1);
  // P0.11 is TRACEDATA2
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(11), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1);
  // P1.09 is TRACEDATA3
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(32+9), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1);
  // P0.07 is TRACECLK
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(7), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1);
#  endif
# else
  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_TRACECONFIG, 0);
# endif
#endif
}

#if CONFIG_NRF5X_MODEL >= 52000

__attribute__((section(".uicr")))
const uint32_t uicr[0x400] = {
  [0 ... 0x3ff] = 0xffffffff,
# if (CONFIG_NRF5X_MODEL == 52832 || CONFIG_NRF5X_MODEL == 52840) && !defined(CONFIG_DRIVER_NRF5X_NFC)
  [0x20c/4] = 0,
# endif  
# if CONFIG_NRF5X_MODEL == 52840 && defined(CONFIG_NRF5X_VREGH)
  [0x304/4] = (CONFIG_NRF5X_VREGH / 3) - 6,
# endif
# if defined(CONFIG_NRF52_RESET_PIN)
#  if CONFIG_NRF5X_MODEL == 52840 || CONFIG_NRF5X_MODEL == 52820 || CONFIG_NRF5X_MODEL == 52833
  [0x200/4] = 0x7fffff12,
  [0x204/4] = 0x7fffff12,
#  elif CONFIG_NRF5X_MODEL == 52810 || CONFIG_NRF5X_MODEL == 52832 || CONFIG_NRF5X_MODEL == 52811
  [0x200/4] = 0x7fffff15,
  [0x204/4] = 0x7fffff15,
#  else
#   error Not supported
#  endif
# endif
};

#endif
