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
# include <device/device.h>
# include <device/class/cpu.h>

void atmel_hw_enum_init()
{
  static struct device_s cpu_dev;

  device_init(&cpu_dev);
  cpu_dev.node.flags |= DEVICE_FLAG_CPU;
  device_res_add_id(&cpu_dev, 0, 0);
  device_attach(&cpu_dev, NULL);

#if defined (CONFIG_CPU_AVR32)
  extern const struct driver_s avr32_drv;

  device_bind_driver(&cpu_dev, &avr32_drv);
  device_init_driver(&cpu_dev);

#elif defined (CONFIG_CPU_ARM)
  extern const struct driver_s arm_drv;

  device_bind_driver(&cpu_dev, &arm_drv);
  device_init_driver(&cpu_dev);
#else
# error Unknown ATMEL cpu
#endif
}

