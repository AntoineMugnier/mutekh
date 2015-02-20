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

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_CLOCK)

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TEMP),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
                   DEV_STATIC_RES_CMU_OSC(NRF5X_LFXO, 0b0001, 32768, 1),
#else
                   DEV_STATIC_RES_IRQ(1, NRF5X_TEMP, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#endif
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ
                   DEV_STATIC_RES_CMU_OSC(NRF5X_HFXO, 0b0001, CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ * 1000000, 1),
#endif
                   );

#endif
