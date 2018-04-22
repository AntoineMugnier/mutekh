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

#define LOGK_MODULE_ID "faul"

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <hexo/interrupt.h>
#include <hexo/lock.h>
#include <hexo/context.h>
#include <hexo/power.h>

#ifdef CONFIG_MUTEK_PRINTK
static lock_t fault_lock;
#endif

static CPU_EXCEPTION_HANDLER(fault_handler)
{
#ifdef CONFIG_MUTEK_PRINTK
  int_fast8_t		i;
  reg_t			*sp = (reg_t*)stackptr;
# ifdef CPU_CONTEXT_REG_NAMES
  static const char		*reg_names[] = { CPU_CONTEXT_REG_NAMES };
# endif

# ifdef CPU_FAULT_NAMES
  static const char *const fault_names[CPU_FAULT_COUNT] = CPU_FAULT_NAMES;
  const char *name = type < CPU_FAULT_COUNT ? fault_names[type] : "unknown";
# else
  const char *name = "unknown";
# endif

  lock_spin(&fault_lock);

  logk_error("CPU Fault: cpuid(%u) faultid(%u-%s)", cpu_id(), type, name);
  logk_error("Execution pointer: %p, Bad address (if any): %p",
             (void*)execptr, (void*)dataptr);
  logk_error("Registers:");

  reg_t *r = regs->gpr;
  for (i = 0; i < CPU_CONTEXT_REG_COUNT; i++)
# ifdef CPU_CONTEXT_REG_NAMES
    logk_error(" %s=%p", reg_names[i], (void*)*(r + i));
# else
    logk_error(" %p", (void*)*(r + i));
# endif

  logk_error("Stack:");

  for (i = 0; i < 12; i++)
    logk_error(" %p: %p", &sp[i], (void*)(uintptr_t)sp[i]);

  lock_release(&fault_lock);
#endif

#ifdef CONFIG_RELEASE
  power_reboot();
#else
  while (1)
    ;
#endif
}

void mutek_fault_initsmp(void)
{
#ifdef CONFIG_MUTEK_PRINTK
  lock_init(&fault_lock);
#endif
  cpu_exception_sethandler(fault_handler);
}

