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
#endif
/** */

#ifndef __MUTEK_ASM__

# include <hexo/cpu.h>

/** PowerPc processor context state */
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
# if CONFIG_CPU_MIPS_FPU == 32
   float fr[32];
# elif CONFIG_CPU_MIPS_FPU == 64
   double fr[32];
# else
#  error
# endif
};

# endif  /* __MUTEK_ASM__ */

#endif

