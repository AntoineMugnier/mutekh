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


#include <mutek/types.h>
#include <mutek/init.h>
#include <mutek/lock.h>

#include <cpu/mutek/apic.h>

#include "multiboot.h"

/* conform to Multiboot Specification */

__attribute__((section (".multiboot")))
struct multiboot_header_s multiboot_header =
{
  .magic = MULTIBOOT_MAGIC,
  .flags = 0,
  .checksum = 0 - MULTIBOOT_MAGIC,
};

static uint_fast8_t		cpu_count = 0;

struct cpu_cld_s	*cpu_cld[256];

static lock_t		cpu_init_lock;	/* cpu intialization lock */
static lock_t		cpu_start_lock;	/* cpu wait for start lock */

/* architecture specific init function */
void arch_init() 
{
  if (cpu_apic_isbootstrap())
    /* First CPU */
    {
      lock_init(&cpu_init_lock);
      lock_init(&cpu_start_lock);

      /* configure system wide cpu data */
      cpu_global_init();

      /* configure first CPU */
      cpu_cld[0] = cpu_init(0);

      /* send reset/init signal to other CPUs */
      lock_try(&cpu_start_lock);
      cpu_start_other_cpu();

      /* run mutek_main() */
      mutek_main(0, 0);
    }
  else
    /* Other CPUs */
    {
      /* configure other CPUs */
      lock_spin(&cpu_init_lock);

      cpu_count++;
      cpu_cld[cpu_count] = cpu_init(cpu_count);

      lock_release(&cpu_init_lock);

      /* wait for start signal */
      while (lock_state(&cpu_start_lock))
	;

      /* run mutek_main_smp() */
      mutek_main_smp();
    }
}

void arch_start_other_cpu(void)
{
  lock_release(&cpu_start_lock);
}

inline uint_fast8_t arch_get_cpu_count(void)
{
  return cpu_count;
}

