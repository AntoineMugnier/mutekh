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
#include <device/class/iomux.h>
#include <device/class/i2c.h>
#include <device/class/gpio.h>
#include <arch/nrf5x/ids.h>

#if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_TIMER("timer* rtc*"),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX(",scl", 0, 2, 0, 0),
                   DEV_STATIC_RES_IOMUX(",sda", 0, 3, 0, 0),
                   DEV_STATIC_RES_I2C_BITRATE(400000),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi0", 0, nrf5x_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPI0),
                   DEV_STATIC_RES_DEV_TIMER("timer* rtc*"),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 5, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 6, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 1, 0, 0),
                   );

#endif

# if defined(CONFIG_DRIVER_NFC_MICORE2)

DEV_DECLARE_STATIC(micore2_dev, "nfc0", 0, micore2_drv,
# if defined(CONFIG_DRIVER_NFC_MICORE2_I2C)
                   DEV_STATIC_RES_I2C_ADDR("/i2c0", 0x28),
# elif defined(CONFIG_DRIVER_NFC_MICORE2_SPI)
                   DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
                   DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", 3),
# endif
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, 4, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   DEV_STATIC_RES_GPIO("resetn", 1, 1),
                   );

# endif
