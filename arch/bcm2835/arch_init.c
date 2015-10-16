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

/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>
#include <mutek/startup.h>

void bcm2835_mem_init(void)
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

# ifdef CONFIG_DEVICE
# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>
# include <device/irq.h>
# include <device/class/iomux.h>
#endif

#include <arch/bcm2835_gpio.h>

#ifdef CONFIG_DRIVER_CPU_ARM32

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_BCM2835_ICU

DEV_DECLARE_STATIC(icu_dev, "icu", 0, bcm2835_icu_drv,
                   DEV_STATIC_RES_MEM(0x2000b000, 0x2000b400),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1),
                   );
#endif


#ifdef CONFIG_DRIVER_CHAR_PL011

DEV_DECLARE_STATIC(uart_dev, "uart", 0, pl011uart_drv,
                   DEV_STATIC_RES_MEM(0x20201000, 0x20202000),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+57, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),

                   DEV_STATIC_RES_IOMUX("rx",  0, 15,  BCM2835_GPIO_GPFSEL_FSEL_FUNCTION0, 0),
                   DEV_STATIC_RES_IOMUX("tx",  0, 14,  BCM2835_GPIO_GPFSEL_FSEL_FUNCTION0, 0)
                   );

#endif


#ifdef CONFIG_DRIVER_BCM2835_SYSTIMER

DEV_DECLARE_STATIC(systimer_dev, "timer", 0, bcm2835_systimer_drv,
                   DEV_STATIC_RES_MEM(0x20003000, 0x20003020),
                   DEV_STATIC_RES_FREQ(1000000, 1),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(1, 8+1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(2, 8+2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(3, 8+3, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   );

#endif


#ifdef CONFIG_DRIVER_BCM2835_GPIO

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, bcm2835_gpio_drv,
                   DEV_STATIC_RES_MEM(0x20200000, 0x20003020),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/icu"),
                   DEV_STATIC_RES_IRQ(0, 8+49, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(1, 8+50, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   );

#endif


