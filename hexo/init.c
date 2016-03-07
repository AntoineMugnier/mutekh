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
#include <mutek/startup.h>

#ifdef CONFIG_DEVICE
# include <device/device.h>
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

/////////////////////////////////// cpu main context intialization

void hexo_context_initsmp(void)
{
  struct context_s *context = CPU_LOCAL_ADDR(cpu_main_context);

#if defined(CONFIG_DEVICE_CPU)
  const struct cpu_tree_s *cpu = cpu_tree_lookup(cpu_id());
  assert(cpu != NULL && "processor id not found in the cpu tree.");

  ensure(context_bootstrap(context, cpu->stack, CONFIG_HEXO_CPU_STACK_SIZE) == 0);
# ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_change_id(cpu->stack, (uint32_t)context);
# endif

#else
  ensure(context_bootstrap(context, CONFIG_STARTUP_STACK_ADDR, CONFIG_STARTUP_STACK_SIZE) == 0);
#endif

  /* enable interrupts from now */
  cpu_interrupt_enable();
}

