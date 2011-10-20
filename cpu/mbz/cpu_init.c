/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Luc Delecroix <luc D delecroix A thalesgroup D com> (c) 2011
    Copyright Laurent Gantel <laurent D gantel A ensea D fr> (c) 2011
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


#ifdef CONFIG_ARCH_SMP
void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

error_t
cpu_global_init(void)
{
  return 0;
}


void cpu_init(void)
{
#ifdef CONFIG_ARCH_SMP

	void			*cls;

	cls = cpu_local_storage[cpu_id()];

	asm volatile ("mcr p15,0,%0,c13,c0,3":: "r" (cls)); //TODO a voir
#endif

}
