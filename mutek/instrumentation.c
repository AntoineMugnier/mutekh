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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <mutek/startup.h>
#include <hexo/cpu.h>
#include <mutek/instrumentation.h>

void instrumentation_globals_init(void)
{
  extern __ldscript_symbol_t __data_start;
  extern __ldscript_symbol_t __data_load_start;
  extern __ldscript_symbol_t __data_load_end;

  instrumentation_memory_region_state_change(
      (uintptr_t)&__data_start,
      (uintptr_t)&__data_load_end - (uintptr_t)&__data_load_start,
      INSTRUMENTATION_MEMORY_REGION_GLOBAL);

  extern __ldscript_symbol_t __bss_start;
  extern __ldscript_symbol_t __bss_end;

  instrumentation_memory_region_state_change(
      (uintptr_t)&__bss_start,
      (uintptr_t)&__bss_end - (uintptr_t)&__bss_start,
      INSTRUMENTATION_MEMORY_REGION_GLOBAL);
}

void instrumentation_cpu_context_init(void)
{
  instrumentation_context_bootstrap(0, CONFIG_STARTUP_STACK_ADDR,
                                    CONFIG_STARTUP_STACK_SIZE);
}

#ifdef CONFIG_ARCH_SMP
void instrumentation_cpu_context_initsmp(void)
{
  if (!cpu_isbootstrap())
    {
      const struct cpu_tree_s *cpu = cpu_tree_lookup(cpu_id());
      assert(cpu != NULL && "processor id not found in the cpu tree.");
      instrumentation_context_bootstrap(cpu->stack, cpu->stack,
                                        CONFIG_HEXO_CPU_STACK_SIZE);
    }
}
#endif
