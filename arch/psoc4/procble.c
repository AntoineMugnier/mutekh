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
#include <device/class/iomux.h>
#include <arch/psoc4/variant.h>

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

#define IDL 0b001 // Mode 0, Idle: Sleep-unprecise ILO, IMO=16MHz
#define SLP 0b010 // Mode 1, Sleep: Sleep-precise ILO, IMO=16MHz
#define FUL 0b100 // Mode 3, Full power: Max CPU speed
#define ALL 0b111

DEV_DECLARE_STATIC(
  clock_dev, "clock", 0, psoc4_clock_drv,

  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_IMO, PSOC4_CLOCK_SRC_HFCLK, ALL, 1, 1),
  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_IMO, PSOC4_CLOCK_SRC_SYSCLK, ALL, 1, 1),
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_ECO, PSOC4_CLOCK_SRC_LLCLK, ALL, 1, 1),
  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_ILO, PSOC4_CLOCK_SRC_LFCLK, IDL, 1, 1),
  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_SINK_WCO, PSOC4_CLOCK_SRC_LFCLK, SLP|FUL, 1, 1),
#else
  DEV_STATIC_RES_CMU_MUX(PSOC4_CLOCK_OSC_ILO, PSOC4_CLOCK_SRC_LFCLK, ALL, 1, 1),
#endif

  DEV_STATIC_RES_CMU_OSC(PSOC4_CLOCK_OSC_IMO, FUL, 48000000, 1),
  DEV_STATIC_RES_CMU_OSC(PSOC4_CLOCK_OSC_IMO, SLP|IDL, 16000000, 1),
  DEV_STATIC_RES_CMU_OSC(PSOC4_CLOCK_OSC_ILO, ALL, 32768, 1),
#if defined(CONFIG_DRIVER_PSOC4_BLE)
  DEV_STATIC_RES_CMU_OSC_ACC(PSOC4_CLOCK_OSC_ECO, ALL, 24000000, 1, 7, 16), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(PSOC4_CLOCK_OSC_ILO, ALL, 32768, 1, 2, 15), // 20ppm
#endif

  DEV_STATIC_RES_MEM(PSOC4_SRSS_ADDR, PSOC4_SRSS_ADDR + 0x1000),
  );

#endif

#if defined(CONFIG_DRIVER_PSOC4_PERI)

static const uint8_t pclk_src[16] = {
  [0 ... 15] = PSOC4_PERI_DIV_NONE,
  //[PSOC4_PERI_IMO_SS] = ,
  [PSOC4_PERI_SCB0] = PSOC4_PERI_DIV(16_5, 0),
  [PSOC4_PERI_SCB1] = PSOC4_PERI_DIV(16_5, 1),
  //[PSOC4_PERI_PUMP] = ,
  //[PSOC4_PERI_CSD0] = ,
  //[PSOC4_PERI_CSD1] = ,
  //[PSOC4_PERI_SAR] = ,
  //[PSOC4_PERI_TCPWM0] = ,
  //[PSOC4_PERI_TCPWM1] = ,
  //[PSOC4_PERI_TCPWM2] = ,
  //[PSOC4_PERI_TCPWM3] = ,
  //[PSOC4_PERI_LCD] = ,
};

DEV_DECLARE_STATIC(
  peri_dev, "peri", 0, psoc4_peri_drv,
  DEV_STATIC_RES_MEM(PSOC4_PERI_ADDR, PSOC4_PERI_ADDR + 0x1000),
  DEV_STATIC_RES_BLOB_PARAM("pclk_src", pclk_src),
  DEV_STATIC_RES_CLK_SRC("/clock", PSOC4_CLOCK_SRC_HFCLK, 0),
  );

#endif

#if defined(CONFIG_DRIVER_PSOC4_GPIO)

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, psoc4_gpio_drv,
                   DEV_STATIC_RES_IRQ(0, PSOC4_IRQ_GPIO(0), DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(1, PSOC4_IRQ_GPIO(1), DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(2, PSOC4_IRQ_GPIO(2), DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(3, PSOC4_IRQ_GPIO(3), DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(4, PSOC4_IRQ_GPIO(4), DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(5, PSOC4_IRQ_GPIO(5), DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_PSOC4_RTC)

DEV_DECLARE_STATIC(rtc0, "rtc0", 0, psoc4_rtc_drv,
                   DEV_STATIC_RES_MEM(PSOC4_SRSS_ADDR, PSOC4_SRSS_ADDR + 0x1000),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, PSOC4_IRQ_WDT, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/clock", PSOC4_CLOCK_SRC_LFCLK, 0),
# else
                   DEV_STATIC_RES_FREQ(32768, 1),
# endif
                   );

#endif

#ifdef CONFIG_DRIVER_PSOC4_BLE

DEV_DECLARE_STATIC(ble_dev, "ble", 0, psoc4_ble_drv,
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, PSOC4_IRQ_BLESS, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_CLK_SRC("/clock", PSOC4_CLOCK_SRC_LFCLK, PSOC4_BLE_CLK_SINK_LFCLK),
                   DEV_STATIC_RES_CMU_OSC_ACC(PSOC4_BLE_CLK_OSC_ECO, 1, 24000000, 1, 4, 16), // 49ppm
                   DEV_STATIC_RES_CMU_OSC_ACC(PSOC4_BLE_CLK_SRC_WCO, 1, 32768, 1, 4, 16), // 49ppm
                   );

#endif
