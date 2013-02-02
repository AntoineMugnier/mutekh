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

void raspberry_bss_section_init()
{
  extern __ldscript_symbol_t __bss_start;
  extern __ldscript_symbol_t __bss_end;

  memset((uint8_t*)&__bss_start, 0,
         (uint8_t*)&__bss_end - (uint8_t*)&__bss_start);
}

/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void raspberry_mem_init()
{
    extern __ldscript_symbol_t __system_heap_start, __system_heap_end;

    default_region = memory_allocator_init(NULL, 
                                           &__system_heap_start, 
                                           (void*)((uintptr_t)&__system_heap_end -
                                                   (1 << CONFIG_HEXO_RESET_STACK_SIZE) * CONFIG_CPU_MAXCOUNT));    
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
  device_res_add_id(&cpu_dev, 0, 0);
  device_attach(&cpu_dev, NULL);

  extern const struct driver_s arm_drv;

  device_bind_driver(&cpu_dev, &arm_drv);
  device_init_driver(&cpu_dev);
}

