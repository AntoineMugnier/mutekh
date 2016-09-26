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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DEVICE
# include <device/class/spi.h>
# include <device/class/nfc.h>
# include <device/class/timer.h>
# include <device/class/iomux.h>
# include <device/class/gpio.h>
#endif

#if defined(CONFIG_DRIVER_NRF52_SPIM) || defined(CONFIG_DRIVER_NRF5X_SPI)

#if defined(CONFIG_DRIVER_NRF52_SPIM)

DEV_DECLARE_STATIC(spi_dev, "spi1", 0, nrf5x_spim_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPIM1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_DEV_TIMER("/timer1"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPIM1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 25, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 24, 0, 0),
                   DEV_STATIC_RES_IOMUX("clk", 0, 23, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi1", 0, nrf5x_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPI1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_DEV_TIMER("/timer1"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 5, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 4, 0, 0),
                   DEV_STATIC_RES_IOMUX("clk", 0, 6, 0, 0),
                   );

#endif

# if defined(CONFIG_DRIVER_PN512)

DEV_DECLARE_STATIC(pn512_dev, "pn512", 0, pn512_drv,
                   DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
                   DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, 1, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   //DEV_STATIC_RES_GPIO("nirq", 1, 1),
                   DEV_STATIC_RES_GPIO("resetn", 3, 1),
                   DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", 2),
                   );

# endif
#endif
