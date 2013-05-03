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

/////////////////////////////////// cpu stacks intialization

#include <mutek/mem_alloc.h>

#define BAD_CPU_STACK_ADDR 0xa5a5a5a5

uintptr_t *cpu_stacks_pool = (void*)0xa5a5a5a5;

static DEVICE_TREE_WALKER(mutek_cpus_stack_walk)
{
  uintptr_t *pool = priv;

  if (dev->node.flags & DEVICE_FLAG_CPU &&
      dev->status == DEVICE_DRIVER_INIT_DONE)
    {
      uintptr_t maj, min;
      if (device_res_get_uint(dev, DEV_RES_ID, 0, &maj, &min))
#ifdef CONFIG_ARCH_SMP
        {
          printk("error: missing processor id in the device tree\n");
          abort();
        }
#else
        min = maj = 0;
#endif

      if (maj > CONFIG_ARCH_LAST_CPU_ID)
        {
          printk("error: found out of range processor id `%u' in the device tree\n", maj);
          abort();
        }

      if (pool[maj] != BAD_CPU_STACK_ADDR)
        {
          printk("error: found multiple processors with the same id `%u' in the device tree\n", maj);
          abort();
        }

      void *s = mem_alloc_cpu(CONFIG_HEXO_CPU_STACK_SIZE, mem_scope_cpu, maj);

      if (!s)
        {
          printk("error: Unable allocate the startup stack for processor id %u\n", maj);
          abort();
        }

      pool[maj] = (uintptr_t)s;
    }

  return 0;
}

void hexo_cpus_stack_init()
{
  uintptr_t *pool = mem_alloc((CONFIG_ARCH_LAST_CPU_ID + 1) * sizeof(void*), mem_scope_sys);

  if (!pool)
    {
      printk("error: Unable allocate the startup stacks pool\n");
      abort();
    }

  cpu_id_t i;
  for (i = 0; i <= CONFIG_ARCH_LAST_CPU_ID; i++)
    pool[i] = BAD_CPU_STACK_ADDR;

  device_tree_walk(NULL, &mutek_cpus_stack_walk, pool);

  cpu_stacks_pool = pool;
}

/////////////////////////////////// cpu main context intialization

void hexo_context_initsmp()
{
  uintptr_t stack = cpu_stacks_pool[cpu_id()];
  struct context_s *context = CPU_LOCAL_ADDR(cpu_main_context);

  context_bootstrap(context, stack, CONFIG_HEXO_CPU_STACK_SIZE);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_change_id(stack, (uint32_t)context);
#endif

  /* enable interrupts from now */
  cpu_interrupt_enable();
}

