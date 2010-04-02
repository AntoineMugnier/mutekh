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

/** @multiple @this describes @ref context_regs_s field offset */
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

#ifndef __MUTEK_ASM__

# include <hexo/cpu.h>

/** PowerPc processor context state */
struct context_regs_s
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

# endif  /* __MUTEK_ASM__ */

#endif

