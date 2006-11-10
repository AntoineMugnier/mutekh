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

struct cpu_cld_s	*cpu_cld[1];

/* architecture specific init function */
void arch_init()
{
#ifdef CONFIG_SMP
  if (cpu_isbootstrap())
    /* First CPU */
    {
#endif
      mem_init();

      cpu_global_init();

      /* configure first CPU */
      cpu_cld[0] = cpu_init(0);

      /* run mutek_main() */
      mutek_main(0, 0);
#ifdef CONFIG_SMP
    }
  else
    /* Other CPUs */
    {
      /* configure other CPUs */

      #error init lock missing
      cpu_cld[cpu_count] = cpu_init(cpu_count);
      cpu_count++;

      /* run mutek_main_smp() */
      mutek_main_smp();
    }
#endif

  assert(!"mutek_main*() should not return");
}

void arch_start_other_cpu(void)
{
}

inline uint_fast8_t arch_get_cpu_count(void)
{
#ifdef CONFIG_SMP
  return cpu_count;
#else
  return 1;
#endif
}

