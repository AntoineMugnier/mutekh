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
#include <device/class/iomux.h>
#include <device/class/clock.h>
#include <arch/nrf51/peripheral.h>
#include <arch/nrf51/ids.h>

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_NRF51_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf51_clock_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_CLOCK),
                   DEV_STATIC_RES_IRQ(0, NRF51_CLOCK, 0, "/cpu")
                   );

#endif

#if defined(CONFIG_DRIVER_NRF51_GPIO)

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, nrf51_gpio_drv,
                   DEV_STATIC_RES_MEM(0x50000000, 0x50001000),
# if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_GPIOTE),
                   DEV_STATIC_RES_IRQ(0, NRF51_GPIOTE, 0, "/cpu"),
# endif
                   );

#endif

#if defined(CONFIG_DRIVER_NRF51_RTC)

DEV_DECLARE_STATIC(rtc1, "rtc1", 0, nrf51_rtc_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_RTC1),
                   DEV_STATIC_RES_IRQ(0, NRF51_RTC1, 0, "/cpu"),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF51_CLOCK_LF_CALIBRATED, NRF51_BLE_RADIO_CLK_SLEEP)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF51_NVMC) || defined(CONFIG_DRIVER_NRF51_PERSIST)

DEV_DECLARE_STATIC(nvmc_dev, "nvmc", 0, nrf51_nvmc_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_NVMC),
#if defined(CONFIG_DRIVER_NRF51_PERSIST)
                   DEV_STATIC_RES_MEM(CONFIG_LOAD_ROM_RO_SIZE - CONFIG_DRIVER_NRF51_PERSIST_SIZE,
                                      CONFIG_DRIVER_NRF51_PERSIST_SIZE)
#endif
                   );

#endif

#ifdef CONFIG_DRIVER_NRF51_RAM

DEV_DECLARE_STATIC(ram_dev, "ram", 0, nrf51_ram_drv);

#endif

#if defined(CONFIG_DRIVER_NRF51_TIMER)

DEV_DECLARE_STATIC(timer1, "timer1", 0, nrf51_timer_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_TIMER1),
                   DEV_STATIC_RES_IRQ(0, NRF51_TIMER1, 0, "/cpu")
                   );

#endif

#if defined(CONFIG_DRIVER_NRF51_BLE_RADIO)

DEV_DECLARE_STATIC(ble_radio, "ble-radio", 0, nrf51_ble_radio_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_RADIO),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_TIMER0),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_RTC0),
                   DEV_STATIC_RES_IRQ(NRF51_BLE_RADIO_IRQ_RADIO, NRF51_RADIO, 0, "/cpu"),
                   DEV_STATIC_RES_IRQ(NRF51_BLE_RADIO_IRQ_TIMER, NRF51_TIMER0, 0, "/cpu"),
                   DEV_STATIC_RES_IRQ(NRF51_BLE_RADIO_IRQ_RTC, NRF51_RTC0, 0, "/cpu"),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF51_CLOCK_LF_CALIBRATED, NRF51_BLE_RADIO_CLK_SLEEP),
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF51_CLOCK_HF_PRECISE, NRF51_BLE_RADIO_CLK_RADIO)
                   );

#endif
