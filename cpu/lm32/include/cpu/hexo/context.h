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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2011
*/

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

# include <hexo/types.h>

# define CPU_LM32_CONTEXT_RESTORE_CALLEE   1
# define CPU_LM32_CONTEXT_RESTORE_CALLER   2

/** @multiple @this describes @ref cpu_context_s field offset */
#define CPU_LM32_CONTEXT_SAVE_MASK       0
#define CPU_LM32_CONTEXT_GPR(n)  	((n) * INT_REG_SIZE/8)
# define CPU_LM32_CONTEXT_IE            (CPU_LM32_CONTEXT_GPR(32))
# define CPU_LM32_CONTEXT_PC           (CPU_LM32_CONTEXT_GPR(33))
/** */

# include <hexo/cpu.h>

/** Lm32 processor context state */
struct cpu_context_s
{
  union {
    reg_t save_mask;       //< what is being saved and restored
    reg_t gpr[32];
  };
  reg_t ei;
  reg_t pc;
};

/** name of registers accessible using cpu_context_s::gpr */
# define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "ei", "pc"
/** number of registers in cpu_context_s::gpr */
# define CPU_CONTEXT_REG_COUNT 34

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

