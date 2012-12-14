/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.

   Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
   Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

#include <mutek/startup.h>

#include <string.h>

/////////////////////////////////////////////////////////////////////

void gaisler_bss_section_init()
{
    extern __ldscript_symbol_t __bss_start;
    extern __ldscript_symbol_t __bss_end;

    memset(
        (uint8_t*)&__bss_start,
        0,
        (uint8_t*)&__bss_end-(uint8_t*)&__bss_start);
}

/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/mem_region.h>
#include <mutek/memory_allocator.h>

void gaisler_mem_init()
{
    extern __ldscript_symbol_t __system_uncached_heap_start, __system_uncached_heap_end;

    default_region = memory_allocator_init(NULL, 
                                           &__system_uncached_heap_start, 
                                           (void*)((uintptr_t)&__system_uncached_heap_end -
                                                   (1 << CONFIG_HEXO_RESET_STACK_SIZE) * CONFIG_CPU_MAXCOUNT));    
}

/////////////////////////////////////////////////////////////////////

#ifdef CONFIG_DEVICE_IRQ
struct device_s *gaisler_icu = NULL;
#endif

#ifdef CONFIG_GAISLER_AHB_ENUM

# include <device/driver.h>
# include <device/device.h>
# include <device/class/enum.h>

void gaisler_ahb_enum_init()
{
    extern const struct driver_s ahbctrl_drv;
    static struct device_s ahbctrl_dev;

    device_init(&ahbctrl_dev);
    device_attach(&ahbctrl_dev, NULL);
    device_res_add_mem(&ahbctrl_dev, CONFIG_GAISLER_AHB_ENUM_ADDR,
                       CONFIG_GAISLER_AHB_ENUM_ADDR + 0xe00);
    device_bind_driver(&ahbctrl_dev, &ahbctrl_drv);
    device_init_driver(&ahbctrl_dev);
}

#endif


#ifdef CONFIG_ARCH_SMP

#include <hexo/iospace.h>

void gaisler_start_cpus()
{
// start other 4 CPUs hack
#warning SMP start hack
    cpu_mem_write_32(0x80000010, (1 << CONFIG_CPU_MAXCOUNT) - 1);
}

#endif

/////////////////////////////////////////////////////////////////////

#if defined(CONFIG_DEVICE_IRQ) && defined(CONFIG_ARCH_SMP)
bool_t arch_cpu_irq_affinity_test(struct device_s *cpu, struct dev_irq_ep_s *src)
{
    return 1;
}
#endif

