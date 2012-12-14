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

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <hexo/interrupt.h>
#include <hexo/lock.h>
#include <hexo/context.h>

static lock_t fault_lock;

static CPU_EXCEPTION_HANDLER(fault_handler)
{
  int_fast8_t		i;
  reg_t			*sp = (reg_t*)stackptr;
#ifdef CPU_CONTEXT_REG_NAMES
  static const char		*reg_names[] = { CPU_CONTEXT_REG_NAMES };
#endif

#ifdef CPU_FAULT_NAMES
  static const char *const fault_names[CPU_FAULT_COUNT] = CPU_FAULT_NAMES;
  const char *name = type < CPU_FAULT_COUNT ? fault_names[type] : "unknown";
#else
  const char *name = "unknown";
#endif

  lock_spin(&fault_lock);

  printk("CPU Fault: cpuid(%u) faultid(%u-%s)\n", cpu_id(), type, name);
  printk("Execution pointer: %p, Bad address (if any): %p\n"
	 "Registers:"
		 , (void*)execptr, (void*)dataptr);

  for (i = CPU_CONTEXT_REG_FIRST; i < CPU_CONTEXT_REG_COUNT; i++)
#ifdef CPU_CONTEXT_REG_NAMES
    printk("%s=%p%c", reg_names[i], (reg_t*)(uintptr_t)regs + i, (i + 1) % 4 ? ' ' : '\n');
#else
    printk("%p%c", (void*)(uintptr_t)regs[i], (i + 1) % 4 ? ' ' : '\n');
#endif

  printk("Stack top (%p):\n", (void*)stackptr);

  for (i = 0; i < 12; i++)
	  printk("%p%c", (void*)(uintptr_t)sp[i], (i + 1) % 4 ? ' ' : '\n');

  lock_release(&fault_lock);

  while (1)
    ;
}

void mutek_fault_initsmp()
{
  lock_init(&fault_lock);
  cpu_exception_sethandler(fault_handler);
}

