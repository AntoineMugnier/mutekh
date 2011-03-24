/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#include <mutek/mem_alloc.h>
#include <hexo/init.h>
#include <hexo/segment.h>
#include <hexo/cpu.h>
#include <hexo/local.h>
#include <hexo/interrupt.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#ifdef CONFIG_CPU_ARM_CUSTOM_IRQ_HANDLER
CPU_LOCAL
static uint32_t arm_irq_stack[128/4];
#endif

#ifdef CONFIG_ARCH_SMP
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
#ifdef CONFIG_ARCH_SMP
# if !defined(CONFIG_CPU_ARM_TLS_IN_C15)
#  error SMP and TLS unsupported
# endif

	void			*cls;

	cls = cpu_local_storage[cpu_id()];
		
	asm volatile ("mcr p15,0,%0,c13,c0,3":: "r" (cls));
#endif
}

