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

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

# include <hexo/types.h>

/** @multiple @this specify context save mask values */
# define CPU_MIPS_CONTEXT_RESTORE_CALLEE   1
# define CPU_MIPS_CONTEXT_RESTORE_CALLER   2

# define CPU_MIPS_CONTEXT_RESTORE_NONE     (~3)

/** @multiple @this describes @ref cpu_context_s field offset */
#define CPU_MIPS_CONTEXT_SAVE_MASK      0
#define CPU_MIPS_CONTEXT_GPR(n)         (CPU_MIPS_CONTEXT_SAVE_MASK + INT_REG_SIZE/8 + (n - 1) * INT_REG_SIZE/8)
#define CPU_MIPS_CONTEXT_LO             CPU_MIPS_CONTEXT_GPR(32)
#define CPU_MIPS_CONTEXT_HI             (CPU_MIPS_CONTEXT_LO + INT_REG_SIZE/8)
#define CPU_MIPS_CONTEXT_SR             (CPU_MIPS_CONTEXT_HI + INT_REG_SIZE/8)
#define CPU_MIPS_CONTEXT_PC      	(CPU_MIPS_CONTEXT_SR + INT_REG_SIZE/8)
#ifdef CONFIG_HEXO_FPU
# define CPU_MIPS_CONTEXT_FR(n)   	(CPU_MIPS_CONTEXT_PC + INT_REG_SIZE/8 + n * CONFIG_CPU_MIPS_FPU/8)
# define CPU_MIPS_CONTEXT_FCSR   	CPU_MIPS_CONTEXT_FR(32)
# define CPU_MIPS_CONTEXT_SIZE   	(CPU_MIPS_CONTEXT_FCSR + INT_REG_SIZE/8)
#else
# define CPU_MIPS_CONTEXT_SIZE   	(CPU_MIPS_CONTEXT_PC + INT_REG_SIZE/8)
#endif
/** */

# include <hexo/cpu.h>

/** Mips processor context state */
struct cpu_context_s
{
  union {
    reg_t save_mask;       //< what is being saved and restored
    reg_t gpr[32];
  };
  reg_t lo;
  reg_t hi;
  reg_t sr;
  reg_t pc;
#ifdef CONFIG_HEXO_FPU
# if CONFIG_CPU_MIPS_FPU == 32
   float fr[32];
# elif CONFIG_CPU_MIPS_FPU == 64
   double fr[32];
# else
#  error
# endif
  reg_t fcsr;
#endif
} __attribute__((aligned(16)));

/** name of registers accessible using cpu_context_s::gpr */
# define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "lo", "hi", "sr", "pc"
/** number of registers in cpu_context_s::gpr */
# define CPU_CONTEXT_REG_COUNT 36

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

# ifdef CONFIG_HEXO_CONTEXT_IRQEN
extern CPU_LOCAL context_irqen_t *cpu_irqen_handler;

ALWAYS_INLINE void context_set_irqen(context_irqen_t *func)
{
  CPU_LOCAL_SET(cpu_irqen_handler, func);
}
# endif

#endif

