/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#include <hexo/alloc.h>
#include <hexo/init.h>
#include <hexo/segment.h>
#include <hexo/cpu.h>
#include <hexo/local.h>
#include <hexo/interrupt.h>

#include <drivers/device/icu/mips/icu-mips.h>
#include <device/device.h>
#include <device/driver.h>

CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;
CPU_LOCAL cpu_exception_handler_t  *cpu_exception_handler;

/** pointer to context local storage in cpu local storage */
CPU_LOCAL void *__cpu_context_data_base;

#ifdef CONFIG_DRIVER_ICU_MIPS
CPU_LOCAL struct device_s cpu_icu_dev;
#endif

#ifdef CONFIG_SMP
void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

/* CPU Local Descriptor structure */

error_t
cpu_global_init(void)
{
  return 0;
}

extern __ldscript_symbol_t  	__segment_excep_start;
extern __ldscript_symbol_t __exception_base_ptr;

void cpu_init(void)
{
  /* Set exception vector */
	cpu_mips_mtc0(15, 1, (reg_t)&__exception_base_ptr);

#ifdef CONFIG_SMP
  void			*cls;

  /* setup cpu local storage */

  cls = arch_cpudata_alloc();

  cpu_local_storage[cpu_id()] = cls;

  /* set cpu local storage register base pointer */
  asm volatile("move $27, %0" : : "r" (cls));
#endif

/* #ifdef CONFIG_HEXO_MMU */
/*   mmu_vpage_set(0x80000180, (uintptr_t)&__segment_excep_start, MMU_PAGE_ATTR_RX | MMU_PAGE_ATTR_PRESENT); */
/* #endif */

#ifdef CONFIG_DRIVER_ICU_MIPS
  device_init(CPU_LOCAL_ADDR(cpu_icu_dev));
  icu_mips_init(CPU_LOCAL_ADDR(cpu_icu_dev), NULL);
#endif
}

void cpu_start_other_cpu(void)
{
#ifdef CONFIG_SMP

#endif
}

