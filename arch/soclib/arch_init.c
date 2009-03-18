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

#include <assert.h>

#include <hexo/types.h>
#include <hexo/init.h>
#include <hexo/lock.h>
#include <hexo/cpu.h>
#include <hexo/alloc.h>
#include <mutek/scheduler.h>
#include <mutek/vmem_palloc.h>
#include <mutek/vmem_kalloc.h>

#ifdef CONFIG_ARCH_SOCLIB_RAMLOCK
extern __ldscript_symbol_t __ramlock_base_start;
uintptr_t __ramlock_base = (uintptr_t)&__ramlock_base_start;
#endif

#ifdef CONFIG_HEXO_MMU
extern __ldscript_symbol_t __system_uncached_heap_start, __system_uncached_heap_end;
#endif

#ifdef CONFIG_SMP
static uint_fast8_t	cpu_count = 1;
volatile bool_t		cpu_init_flag = 0;
volatile bool_t		cpu_start_flag = 0;
static lock_t		cpu_init_lock;	/* cpu intialization lock */
/* integer atomic operations global spin lock */
lock_t			__atomic_arch_lock;
#endif

/* architecture specific init function */
void arch_init() 
{
#ifdef CONFIG_SMP
  if (cpu_isbootstrap())	/* FIXME */
    /* First CPU */
    {
      lock_init(&__atomic_arch_lock);
      lock_init(&cpu_init_lock);

#endif

      /* configure system wide cpu data */
      cpu_global_init();

      mem_init();

#ifdef CONFIG_HEXO_MMU
	uint32_t t0=(uint32_t)(&__system_uncached_heap_start);//FIXME: pile?
	uint32_t t1=(uint32_t)(&__system_uncached_heap_end);
#ifdef CONFIG_SMP
	t0+=0x20000;//pas touche a l'espace deja alloué pour la region system et la region cpu
	t1-=0x4000;//pas touche au pile des proc
#else
	t0+=0x10000;//pas touche a l'espace deja alloué pour la region system
	t1-=0x1000;//pas touche a la pile du proc
#endif
	
	mmu_ppage_region_init(t0, t1); 
	mmu_global_init(vmem_vpage_kalloc, vmem_ppage_alloc);

#endif

      /* configure first CPU */
      cpu_init();

#ifdef CONFIG_HEXO_MMU
      mmu_cpu_init();
#endif

#ifdef CONFIG_SMP
      /* send reset/init signal to other CPUs */
      cpu_init_flag = 1;
#endif

#if defined(CONFIG_MUTEK_SCHEDULER)
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
      assert(cpu_id() < CONFIG_CPU_MAXCOUNT);

      while (cpu_init_flag == 0)
	;

      /* configure other CPUs */
      lock_spin(&cpu_init_lock);
      
#ifdef CONFIG_HEXO_MMU
      mmu_cpu_init();
#endif      
      cpu_init();

      lock_release(&cpu_init_lock);

      /* wait for start signal */

      while (cpu_start_flag == 0)
	;

      /* run mutek_main_smp() */

#if defined(CONFIG_MUTEK_SCHEDULER)
      sched_cpu_init();
#endif

      mutek_main_smp();
    }
#endif

  while (1)
    ;
}

void arch_start_other_cpu(void)
{
#ifdef CONFIG_SMP
  cpu_start_flag++;
#endif
}

inline cpu_id_t arch_get_cpu_count(void)
{
#ifdef CONFIG_SMP
  return cpu_count;
#else
  return 1;
#endif
}

