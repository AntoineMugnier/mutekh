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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

#include <hexo/cpu.h>
#include <hexo/context.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <device/device.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

/////////////////////////////////// cpu main context intialization

void hexo_context_initsmp()
{
  const struct cpu_tree_s *cpu = cpu_tree_lookup(cpu_id());
  assert(cpu != NULL && "processor id not found in the cpu tree.");

  struct context_s *context = CPU_LOCAL_ADDR(cpu_main_context);

  context_bootstrap(context, cpu->stack, CONFIG_HEXO_CPU_STACK_SIZE);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_change_id(cpu->stack, (uint32_t)context);
#endif

  /* enable interrupts from now */
  cpu_interrupt_enable();
}

