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

#include <device/resources.h>
#include <device/irq.h>
#include <device/class/iomux.h>
#include <device/class/clock.h>
#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/gpiote.h>

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
#if defined(CONFIG_ARCH_NRF52) && defined(CONFIG_DRIVER_CLOCK)
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_HF, 0),
#endif
                   );

#endif

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TEMP),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
                   DEV_STATIC_RES_IRQ(1, NRF5X_TEMP, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#endif
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO)

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, nrf5x_gpio_drv,
                   DEV_STATIC_RES_MEM(0x50000000, 0x50001000),
# if defined(CONFIG_DRIVER_NRF5X_GPIO_ICU)
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_GPIOTE),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_GPIOTE, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
# endif
                   );

#endif

#if (CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT + CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT) > NRF_GPIOTE_COUNT
# error nRF51 ICU channel count + GPIO_PWM channel count may not use more than total GPIOTE channel count on the platform
#endif

#if defined(CONFIG_DRIVER_NRF5X_RTC)

DEV_DECLARE_STATIC(rtc1, "rtc1", 0, nrf5x_rtc_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_RTC1),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_RTC1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_LF, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_NVMC) || defined(CONFIG_DRIVER_NRF5X_PERSIST)

DEV_DECLARE_STATIC(nvmc_dev, "nvmc", 0, nrf5x_nvmc_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_NVMC),
#if defined(CONFIG_DRIVER_NRF5X_PERSIST)
                   DEV_STATIC_RES_MEM(CONFIG_LOAD_ROM_RO_ADDR + CONFIG_LOAD_ROM_RO_SIZE - CONFIG_DRIVER_NRF5X_PERSIST_SIZE,
                                      CONFIG_DRIVER_NRF5X_PERSIST_SIZE)
#endif
                   );

#endif

#ifdef CONFIG_DRIVER_NRF5X_RAM

DEV_DECLARE_STATIC(ram_dev, "ram", 0, nrf5x_ram_drv);

#endif

#if defined(CONFIG_DRIVER_NRF5X_TIMER)

DEV_DECLARE_STATIC(timer1, "timer1", 0, nrf5x_timer_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER1),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#if defined(CONFIG_DEVICE_CLOCK)
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_HF, 0),
#endif
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_BLE_RADIO)

DEV_DECLARE_STATIC(ble_radio, "ble-radio", 0, nrf5x_ble_radio_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_RADIO),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER0),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_RTC0),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(NRF5X_BLE_RADIO_IRQ_RADIO, NRF5X_RADIO, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(NRF5X_BLE_RADIO_IRQ_TIMER, NRF5X_TIMER0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(NRF5X_BLE_RADIO_IRQ_RTC, NRF5X_RTC0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_LF_PRECISE, NRF5X_BLE_RADIO_CLK_SLEEP),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_HF_PRECISE, NRF5X_BLE_RADIO_CLK_RADIO)
                   );

#if defined(CONFIG_DRIVER_BLE_RADIO)

DEV_DECLARE_STATIC(ble_net_dev, "ble", 0, ble_net_drv,
                   DEV_STATIC_RES_DEV_PARAM("radio", "/ble-radio"),
#if defined(CONFIG_BLE_CRYPTO)
                   DEV_STATIC_RES_DEV_PARAM("crypto", "/aes"),
                   DEV_STATIC_RES_STR_PARAM("rng", "/aes"),
                   DEV_STATIC_RES_STR_PARAM("seed", "/rng"),
#endif
                   );

#endif

#endif

#if defined(CONFIG_DRIVER_NRF5X_AES)

DEV_DECLARE_STATIC(aes_dev, "aes", 0, nrf5x_aes_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_ECB),
#if defined(CONFIG_DRIVER_NRF5X_AES_CCM)
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_CCM, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#endif
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_RNG)

DEV_DECLARE_STATIC(rng_dev, "rng", 0, nrf5x_rng_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_RNG),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_RNG, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_BLE)

DEV_DECLARE_STATIC(ble_radio, "ble", 0, nrf5x_ble_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_RADIO),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER0),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_RTC0),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(NRF5X_BLE_IRQ_RADIO, NRF5X_RADIO, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(NRF5X_BLE_IRQ_TIMER, NRF5X_TIMER0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(NRF5X_BLE_IRQ_RTC, NRF5X_RTC0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#if defined(CONFIG_DEVICE_CLOCK)
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_LF_PRECISE, NRF5X_BLE_CLK_SLEEP),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_HF_PRECISE, NRF5X_BLE_CLK_RADIO),
#endif
#if defined(CONFIG_BLE_CRYPTO)
                   DEV_STATIC_RES_DEV_PARAM("crypto", "/aes"),
                   DEV_STATIC_RES_STR_PARAM("rng", "/aes"),
                   DEV_STATIC_RES_STR_PARAM("seed", "/rng"),
#endif
                   );

#endif