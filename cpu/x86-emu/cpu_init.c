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
#include <string.h>
#include <hexo/interrupt.h>
#include <hexo/init.h>
#include <hexo/iospace.h>
#include <hexo/lock.h>
#include <hexo/segment.h>

CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

/* cpu interrupts state */
volatile CPU_LOCAL bool_t cpu_irq_state = 0;

error_t
cpu_global_init(void)
{
  return 0;
}

struct cpu_cld_s
{
  /* CPU id */
  uint32_t			id;
};

//static CPU_LOCAL struct cpu_cld_s	*cpu_cld;

struct cpu_cld_s *cpu_init(uint_fast8_t cpu_id)
{
  struct cpu_cld_s	*cld;
  //void			*cls;

  if (!(cld = mem_alloc(sizeof (struct cpu_cld_s), MEM_SCOPE_SYS)))
    return NULL;

  cld->id = cpu_id;

#if defined(CONFIG_DEBUG)
  /* enable alignment check */
  asm volatile("	pushf						\n"
	       "	orl	$0x40000, (%esp)			\n"
	       "	popf						\n");
#endif

  return cld;
}

void cpu_start_other_cpu(void)
{
}

uint_fast8_t cpu_id(void)
{
#ifdef CONFIG_SMP
  struct cpu_cld_s	*cld = CPU_LOCAL_GET(cpu_cld);

  return cld->id;
#else
  return 0;
#endif
}

