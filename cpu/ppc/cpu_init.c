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

CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;
CPU_LOCAL cpu_exception_handler_t  *cpu_exception_handler;

/** pointer to context local storage in cpu local storage */
CPU_LOCAL void *__cpu_context_data_base;

#ifdef CONFIG_SMP
void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

/* CPU Local Descriptor structure */

error_t
cpu_global_init(void)
{
  return 0;
}

void cpu_init(void)
{
#ifdef CONFIG_SMP
  void			*cls;

  /* setup cpu local storage */

  cls = arch_cpudata_alloc();

  cpu_local_storage[cpu_id()] = cls;

  /* set cpu local storage register base pointer */
  asm volatile("mtspr 0x115, %0" : : "r" (cls)); /* SPRG5 is cls */
#endif
}

void cpu_start_other_cpu(void)
{
#ifdef CONFIG_SMP

#endif
}

