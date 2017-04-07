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

    Copyright (c) 2017 Nicolas Pouillon <nipo@ssji.net>
*/

#include <device/resources.h>
#include <device/class/i2c.h>
#include <device/class/gpio.h>
#include <arch/pinmap/arduino.h>

DEV_DECLARE_STATIC(ntag_dev, "ntag", 0, ntag_drv,
                   DEV_STATIC_RES_I2C_ADDR("i2c0", 0x55),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_GPIO("fd", PINMAP_ARDUINO_D9, 1),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, PINMAP_ARDUINO_D9, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   );
