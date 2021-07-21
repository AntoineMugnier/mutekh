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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2021
*/

#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/onewire.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DRIVER_NRF5X_ONEWIRE

DEV_DECLARE_STATIC(onewire_dev, "1wire", 0, nrf5x_1wire_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#if defined(CONFIG_DEVICE_CLOCK)
                   DEV_STATIC_RES_CLK_SRC("/clock", NRF_CLOCK_SRC_HFCLK, 0),
# else
                   DEV_STATIC_RES_FREQ_ACC(16000000, 1, 2, 25),
# endif
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("dq", 0, 42, 0, 0),
                   DEV_STATIC_RES_IOMUX("dqpw", 0, 45, 0, 0),
                   );

#endif
