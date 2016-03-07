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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006-2013

*/

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

#include <hexo/cpu.h>

#include "asm.h"

/** @multiple @this specify context save mask values */
#define CPU_ARM_CONTEXT_RESTORE_CALLEE   1
#define CPU_ARM_CONTEXT_RESTORE_CALLER   2

#define CPU_ARM_CONTEXT_RESTORE_NONE     (~3)

/** @multiple @this describes @ref cpu_context_s field offset */
#define CPU_ARM_CONTEXT_GPR(n)         ((n)*4)
#define CPU_ARM_CONTEXT_CPSR           CPU_ARM_CONTEXT_GPR(16)
#define CPU_ARM_CONTEXT_SAVE_MASK      CPU_ARM_CONTEXT_GPR(17)

struct cpu_context_s
{
    reg_t gpr[16];
    reg_t cpsr;
    reg_t save_mask;       //< what is being saved and restored
};

/** name of registers accessible using cpu_context_s::gpr */
# define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "cpsr"
/** number of registers in cpu_context_s::gpr */
# define CPU_CONTEXT_REG_COUNT 17

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
/** @internal */
extern CPU_LOCAL context_preempt_t *cpu_preempt_handler;

ALWAYS_INLINE error_t context_set_preempt(context_preempt_t *func)
{
  context_preempt_t **f = CPU_LOCAL_ADDR(cpu_preempt_handler);
  if (*f != NULL)
    return -EBUSY;
  *f = func;
  return 0;
}
# endif

#endif

