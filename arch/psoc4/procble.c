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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <device/resources.h>
#include <device/class/cpu.h>
#include <device/irq.h>
#include <device/class/cmu.h>
#include <arch/psoc4/variant/procble.h>

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
#if defined(CONFIG_DRIVER_PSOC4_CLOCK)
                   DEV_STATIC_RES_CLK_SRC("/clock", PSOC4_CLOCK_SRC_SYSCLK, 0),
#else
                   DEV_STATIC_RES_FREQ(24000000, 1),
#endif
                   );

#endif

#if defined(CONFIG_DRIVER_PSOC4_CLOCK)

#define IDL 0b0001 // Mode 0, Idle: No LFCLK, IMO=24MHz
#define SLP 0b0010 // Mode 1, Sleep: Sleep-unprecise ILO, IMO=16MHz
#define FUL 0b1000 // Mode 3, Full power: LFCLK, IMO=48MHz

static const uint8_t pclk_src[16] = {
  [0 ... 15] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_IMO_SS] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_SCB0] = PSOC4_DIV(16_5, 0),
  [PSOC4_CLOCK_SCB1] = PSOC4_DIV(16_5, 1),
  [PSOC4_CLOCK_PUMP] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_CSD0] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_CSD1] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_SAR] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_TCPWM0] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_TCPWM1] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_TCPWM2] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_TCPWM3] = PSOC4_DIV_NONE,
  [PSOC4_CLOCK_LCD] = PSOC4_DIV_NONE,
};

DEV_DECLARE_STATIC(
  clock_dev, "clock", 0, psoc4_clock_drv,

  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_IMO, PSOC4_CLOCK_SRC_HFCLK, FUL|SLP|IDL, 1, 1),
  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_ILO, PSOC4_CLOCK_SRC_LFCLK, SLP|IDL|FUL, 1, 1),

  DEV_STATIC_RES_CMU_OSC(PSOC4_CLOCK_OSC_IMO, FUL, 48000000, 1),
  DEV_STATIC_RES_CMU_OSC(PSOC4_CLOCK_OSC_IMO, SLP|IDL, 16000000, 1),
  DEV_STATIC_RES_CMU_OSC(PSOC4_CLOCK_OSC_ILO, SLP|IDL, 32768, 1),

  DEV_STATIC_RES_MEM(PSOC4_SRSS_ADDR, PSOC4_SRSS_ADDR + 0x1000),
  DEV_STATIC_RES_MEM(PSOC4_PERI_ADDR, PSOC4_PERI_ADDR + 0x1000),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_BLOB_PARAM("pclk_src", pclk_src),
  );

#endif
