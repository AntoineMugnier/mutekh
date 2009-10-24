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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/


#include <hexo/types.h>
#include <hexo/init.h>
#include <hexo/cpu.h>
#include <hexo/lock.h>
#include <hexo/alloc.h>

#include <arch/hexo/emu_syscalls.h>

#ifdef CONFIG_SMP
static uint_fast8_t	cpu_count = 1;
#endif

/* architecture specific init function */
void arch_init()
{
#ifdef CONFIG_SMP
  if (cpu_isbootstrap())
    /* First CPU */
    {
#endif
      mem_init();

	  hexo_global_init();

      cpu_global_init();

      /* configure first CPU */
      cpu_init();

#if defined(CONFIG_MUTEK_SCHEDULER)
      sched_global_init();
      sched_cpu_init();
#endif

      /* run mutek_start() */
      mutek_start(0, 0);
#ifdef CONFIG_SMP
    }
  else
    /* Other CPUs */
    {
      /* configure other CPUs */

      #error init lock missing
      cpu_init();
      cpu_count++;

      /* run mutek_start_smp() */

#if defined(CONFIG_MUTEK_SCHEDULER)
      sched_cpu_init();
#endif

      mutek_start_smp();
    }
#endif

  emu_do_syscall(EMU_SYSCALL_EXIT, 0);  
}

void arch_start_other_cpu(void)
{
}

cpu_id_t arch_get_cpu_count(void)
{
#ifdef CONFIG_SMP
  return cpu_count;
#else
  return 1;
#endif
}

