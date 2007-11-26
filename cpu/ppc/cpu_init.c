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

struct cpu_cld_s	*cpu_cld_list[CONFIG_CPU_MAXCOUNT];

/* CPU Local Descriptor structure */

error_t
cpu_global_init(void)
{
  return 0;
}

static CPU_LOCAL struct cpu_cld_s	*cpu_cld;

struct cpu_cld_s * cpu_init(void)
{
  struct cpu_cld_s	*cld;
  void			*cls;

  /* setup cpu local descriptor */

  if (!(cld = mem_alloc(sizeof (struct cpu_cld_s), MEM_SCOPE_SYS)))
    goto err_cld;

  cld->id = cpu_id();

#ifdef CONFIG_SMP
  /* setup cpu local storage */

  if (!(cls = arch_cpudata_alloc()))
    goto err_cls;

  cld->cpu_local_storage = cls;

  /* set cpu local storage register base pointer */
  asm volatile("mtspr 5, %0" : : "r" (cls));
#endif

  CPU_LOCAL_SET(cpu_cld, cld);

  return cld;

#if 0
  mem_free(cls);
#endif
 err_cls:
  mem_free(cld);
 err_cld:
  return NULL;
}

void cpu_start_other_cpu(void)
{
#ifdef CONFIG_SMP

#endif
}

