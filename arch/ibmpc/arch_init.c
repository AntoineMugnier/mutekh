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


#include <hexo/types.h>
#include <hexo/init.h>
#include <hexo/cpu.h>
#include <hexo/lock.h>
#include <hexo/alloc.h>

#include "multiboot.h"

/* conform to Multiboot Specification */

__attribute__((section (".multiboot")))
struct multiboot_header_s multiboot_header =
{
  .magic = MULTIBOOT_MAGIC,
  .flags = 0,
  .checksum = 0 - MULTIBOOT_MAGIC,
};

#ifdef CONFIG_SMP
static lock_t		cpu_init_lock;	/* cpu intialization lock */
static lock_t		cpu_start_lock;	/* cpu wait for start lock */
#endif

/* architecture specific init function */
void arch_init() 
{
#ifdef CONFIG_SMP
  if (cpu_isbootstrap())
    /* First CPU */
    {
      lock_init(&cpu_init_lock);
      lock_init(&cpu_start_lock);

#endif

      mem_init();

      /* configure system wide cpu data */
      cpu_global_init();

      /* configure first CPU */
      cpu_init();

#ifdef CONFIG_HEXO_VMEM
      vmem_global_init();
      vmem_cpu_init();
#endif

      /* send reset/init signal to other CPUs */
#ifdef CONFIG_SMP
      lock_try(&cpu_start_lock);
      cpu_start_other_cpu();
#endif

#if defined(CONFIG_HEXO_SCHED)
      sched_global_init();
      sched_cpu_init();
#endif

      /* run mutek_main() */
      mutek_main(0, 0);
#ifdef CONFIG_SMP
    }
  else
    /* Other CPUs */
    {
      /* configure other CPUs */

      lock_spin(&cpu_init_lock);

      cpu_init();

      lock_release(&cpu_init_lock);

#ifdef CONFIG_HEXO_VMEM
      vmem_cpu_init();
#endif

      /* wait for start signal */
      while (lock_state(&cpu_start_lock))
	;

#if defined(CONFIG_HEXO_SCHED)
      sched_cpu_init();
#endif

      /* run mutek_main_smp() */
      mutek_main_smp();
    }
#endif

  while (1)
    ;
}

void arch_start_other_cpu(void)
{
#ifdef CONFIG_SMP
  lock_release(&cpu_start_lock);
#endif
}


