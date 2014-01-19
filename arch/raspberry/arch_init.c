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

#include <mutek/startup.h>

#include <string.h>


/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void raspberry_mem_init()
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>

DEV_DECLARE_STATIC_RESOURCES(cpu_dev_res, 1,
  DEV_STATIC_RES_ID(0, 0),
);

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm_drv, cpu_dev_res);

#ifdef CONFIG_DRIVER_ICU_BCM2835

DEV_DECLARE_STATIC_RESOURCES(icu_dev_res, 2,
  DEV_STATIC_RES_MEM(0x2000b000, 0x2000b400),
  DEV_STATIC_RES_IRQ(0, 0, 0, "/cpu"),
);

DEV_DECLARE_STATIC(icu_dev, "icu", 0, bcm2835_icu_drv, icu_dev_res);

#endif


#ifdef CONFIG_DRIVER_CHAR_PL011

DEV_DECLARE_STATIC_RESOURCES(uart_dev_res, 2,
  DEV_STATIC_RES_MEM(0x20201000, 0x20202000),
  DEV_STATIC_RES_IRQ(0, 8+57, 0, "/icu"),
);

DEV_DECLARE_STATIC(uart_dev, "uart", 0, pl011uart_drv, uart_dev_res);

#endif


#ifdef CONFIG_DRIVER_SYSTIMER_BCM2835

DEV_DECLARE_STATIC_RESOURCES(systimer_dev_res, 6,
  DEV_STATIC_RES_MEM(0x20003000, 0x20003020),
  DEV_STATIC_RES_FREQ((uint64_t)1000000 << 24),
  DEV_STATIC_RES_IRQ(0, 8+0, 0, "/icu"),
  DEV_STATIC_RES_IRQ(1, 8+1, 0, "/icu"),
  DEV_STATIC_RES_IRQ(2, 8+2, 0, "/icu"),
  DEV_STATIC_RES_IRQ(3, 8+3, 0, "/icu"),
);

DEV_DECLARE_STATIC(systimer_dev, "timer", 0, bcm2835_systimer_drv, systimer_dev_res);

#endif

