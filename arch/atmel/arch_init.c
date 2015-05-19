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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012

*/

#include <mutek/startup.h>

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void atmel_mem_init()
{
    default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                           (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                   CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>
# include <device/class/cpu.h>

DEV_DECLARE_STATIC_RESOURCES(cpu_dev_res, 1,
  DEV_STATIC_RES_ID(0, 0),
);

#if defined (CONFIG_CPU_AVR32)
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, avr32_drv, &cpu_dev_res);

#elif defined (CONFIG_CPU_ARM32)
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32_drv, &cpu_dev_res);

#elif defined (CONFIG_CPU_ARM32M)
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv, &cpu_dev_res);

# error Unknown ATMEL cpu
#endif

