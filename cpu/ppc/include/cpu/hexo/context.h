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
# define CPU_PPC_CONTEXT_RESTORE_CALLEE   1
# define CPU_PPC_CONTEXT_RESTORE_CALLER   2

# define CPU_PPC_CONTEXT_RESTORE_NONE     (~3)

/** @multiple @this describes @ref cpu_context_s field offset */
#define CPU_PPC_CONTEXT_SAVE_MASK       0
#define CPU_PPC_CONTEXT_GPR(n)  	(CPU_PPC_CONTEXT_SAVE_MASK + INT_REG_SIZE/8 + n * INT_REG_SIZE/8)
#define CPU_PPC_CONTEXT_CR      	CPU_PPC_CONTEXT_GPR(32)
#define CPU_PPC_CONTEXT_CTR     	(CPU_PPC_CONTEXT_CR + INT_REG_SIZE/8)
#define CPU_PPC_CONTEXT_MSR     	(CPU_PPC_CONTEXT_CTR + INT_REG_SIZE/8)
#define CPU_PPC_CONTEXT_LR      	(CPU_PPC_CONTEXT_MSR + INT_REG_SIZE/8)
#define CPU_PPC_CONTEXT_PC      	(CPU_PPC_CONTEXT_LR + INT_REG_SIZE/8)
#ifdef CONFIG_HEXO_FPU
# define CPU_PPC_CONTEXT_FR(n)   	(CPU_PPC_CONTEXT_PC + INT_REG_SIZE/8 + n * 8)
# define CPU_PPC_CONTEXT_FPSCR   	CPU_PPC_CONTEXT_FR(32)
# define CPU_PPC_CONTEXT_XER     	(CPU_PPC_CONTEXT_FPSCR + 8)
#endif
/** */

# include <hexo/cpu.h>

/** PowerPc processor context state */
struct cpu_context_s
{
  reg_t save_mask;       //< what is being saved and restored
  reg_t gpr[32];
  reg_t cr;
  reg_t ctr;
  reg_t msr;
  reg_t lr;
  reg_t pc;
# ifdef CONFIG_HEXO_FPU
  /* 64 bits aligned here */
  double fpr[32];
  uint64_t fpscr;
  reg_t xer;
# endif
};

/** name of registers accessible using cpu_context_s::gpr */
# define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "cr", "ctr", "msr", "lr", "pc"
/** number of registers in cpu_context_s::gpr */
# define CPU_CONTEXT_REG_COUNT 37

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

