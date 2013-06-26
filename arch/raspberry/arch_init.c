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
# include <device/device.h>
# include <device/class/cpu.h>

void raspberry_hw_enum_init()
{
  static struct device_s cpu_dev;

  device_init(&cpu_dev);
  cpu_dev.node.flags |= DEVICE_FLAG_CPU;
  device_set_name(&cpu_dev, "cpu");
  device_res_add_id(&cpu_dev, 0, 0);
  device_attach(&cpu_dev, NULL);

  extern const struct driver_s arm_drv;

  device_bind_driver(&cpu_dev, &arm_drv);
  device_init_driver(&cpu_dev);

#ifdef CONFIG_DRIVER_ICU_BCM2835
  static struct device_s icu_dev;

  device_init(&icu_dev);
  device_set_name(&icu_dev, "icu");
  device_res_add_mem(&icu_dev, 0x2000b000, 0x2000b400);
  device_res_add_irq(&icu_dev, 0, 0, 0, &cpu_dev);  // irq
  //  device_res_add_irq(&icu_dev, 1, 1, 0, &cpu_dev);  // fiq
  device_attach(&icu_dev, NULL);

  extern const struct driver_s bcm2835_icu_drv;

  device_bind_driver(&icu_dev, &bcm2835_icu_drv);
  device_init_driver(&icu_dev);
#endif

#ifdef CONFIG_DRIVER_CHAR_PL011
  static struct device_s uart_dev;

  device_init(&uart_dev);
  device_set_name(&uart_dev, "uart");
  device_res_add_mem(&uart_dev, 0x20201000, 0x20202000);
#ifdef CONFIG_DRIVER_ICU_BCM2835
  device_res_add_irq(&uart_dev, 0, 8+57, 0, &icu_dev);
#endif
  device_attach(&uart_dev, NULL);

  extern const struct driver_s pl011uart_drv;

  device_bind_driver(&uart_dev, &pl011uart_drv);
  device_init_driver(&uart_dev);
#endif
}

