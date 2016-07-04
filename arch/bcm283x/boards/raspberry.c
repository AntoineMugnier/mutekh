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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Jeremie Brunel <jeremie.brunel@telecom-paristech.fr> (c) 2013
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012

*/

#ifdef CONFIG_DEVICE
# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>
# include <device/irq.h>
# include <device/class/iomux.h>
# include <device/class/timer.h>
# include <device/class/i2c.h>
#endif

#include <arch/bcm283x/gpio.h>

#define RASPBERRY_B1     1
#define RASPBERRY_A2     2
#define RASPBERRY_B2     3
#define RASPBERRY_A1PLUS 4
#define RASPBERRY_B1PLUS 5

#ifdef CONFIG_DRIVER_CPU_ARM32

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(700000000, 1)
                   );

#endif

#ifdef CONFIG_DRIVER_BCM283X_SPI

DEV_DECLARE_STATIC(spi_dev, "spi0", 0, bcm283x_spi_drv,
                   DEV_STATIC_RES_MEM(0x20204000, 0x20204020),

                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+54, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_DEV_TIMER("/timer"),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk",  0, 11, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 9,  BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 10, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
#if 1
                   DEV_STATIC_RES_IOMUX("cs0",  0, 8,  BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("cs1",  0, 7,  BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0)
#endif
                   );

#endif

#ifdef CONFIG_DRIVER_BCM283X_I2C

# if CONFIG_ARCH_BCM283X_BOARD == RASPBERRY_B1

/* i2c on P1 header */
DEV_DECLARE_STATIC(i2c0_dev, "i2c0", 0, bcm283x_i2c_drv,
                   DEV_STATIC_RES_MEM(0x20205000, 0x20804020),

                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+53, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_DEV_TIMER("/timer"),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, 1, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 0, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0)
                   );

# endif

# if CONFIG_ARCH_BCM283X_BOARD == RASPBERRY_B2 || \
     CONFIG_ARCH_BCM283X_BOARD == RASPBERRY_A2

/* i2c on P5 header */
DEV_DECLARE_STATIC(i2c0_dev, "i2c0", 0, bcm283x_i2c_drv,
                   DEV_STATIC_RES_MEM(0x20205000, 0x20804020),

                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+53, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_I2C_BITRATE(100000),

                   DEV_STATIC_RES_DEV_TIMER("/timer"),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, 29, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 28, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0)
                   );

/* i2c on P1 header */
DEV_DECLARE_STATIC(i2c1_dev, "i2c1", 0, bcm283x_i2c_drv,
                   DEV_STATIC_RES_MEM(0x20804000, 0x20804020),

                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+53, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_I2C_BITRATE(100000),

                   DEV_STATIC_RES_DEV_TIMER("/timer"),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, 3, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 2, BCM283X_GPIO_GPFSEL_FSEL_FUNCTION0, 0)
                   );

# endif

#endif
