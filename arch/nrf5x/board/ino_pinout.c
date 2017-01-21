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
#include <device/class/iomux.h>
#include <device/class/uart.h>
#include <device/class/gpio.h>
#include <device/class/cmu.h>
#include <device/class/i2c.h>
#include <arch/nrf5x/ids.h>
#include <arch/ino_pinout.h>

#if defined(CONFIG_DRIVER_NRF5X_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi0", 0, nrf5x_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPI1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("mosi", 0, ARCH_INO_PINOUT_MOSI, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, ARCH_INO_PINOUT_MISO, 0, 0),
                   DEV_STATIC_RES_IOMUX("clk", 0, ARCH_INO_PINOUT_SCK, 0, 0),
                   DEV_STATIC_RES_DEV_TIMER("rtc* timer*"),
                   );
#endif

#if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c0_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_DEV_TIMER("/rtc* /timer*"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_I2C_BITRATE(100000),
                   DEV_STATIC_RES_IOMUX(",scl", 0, ARCH_INO_PINOUT_SCL, 0, 0),
                   DEV_STATIC_RES_IOMUX(",sda", 0, ARCH_INO_PINOUT_SDA, 0, 0)
                   );

# if defined(CONFIG_DRIVER_GPIO_PCAL6408A)
DEV_DECLARE_STATIC(pcal6408a_dev, "gpio1", 0, pcal6408a_drv,
                   DEV_STATIC_RES_I2C_ADDR("i2c0", 0x20),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, ARCH_NRF_PIN_INT_EXT, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1),
                   );

#  if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)
DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                     DEV_STATIC_RES_DEV_GPIO("/gpio1"),
                     DEV_STATIC_RES_GPIO("pins", 0, 4),
                     DEV_STATIC_RES_UINT_PARAM("active", 0),
                     );
#  endif
# endif
#endif
