/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Luc Delecroix <luc D delecroix A thalesgroup D com> (c) 2011
    Copyright Laurent Gantel <laurent D gantel A ensea D fr> (c) 2011
*/

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

#include <hexo/cpu.h>

#define OFFSET_MBZ_CONTEXT_REG	0x0C	//ATTENTION VALEUR EN DUR

/** @multiple @this specify context save mask values */
# define CPU_MBZ_CONTEXT_RESTORE_CALLEE   1
# define CPU_MBZ_CONTEXT_RESTORE_CALLER   2

# define CPU_MBZ_CONTEXT_RESTORE_NONE     (~3)

/** @multiple @this describes @ref cpu_context_s field offset */
#define CPU_MBZ_CONTEXT_SAVE_MASK		0
#define CPU_MBZ_CONTEXT_GPR(n)			((CPU_MBZ_CONTEXT_SAVE_MASK + 1 + n)*4)
#define CPU_MBZ_CONTEXT_SP         		(CPU_MBZ_CONTEXT_GPR(1))
#define CPU_MBZ_CONTEXT_MSR				(CPU_MBZ_CONTEXT_GPR(32))
#define CPU_MBZ_CONTEXT_PC				(CPU_MBZ_CONTEXT_MSR + INT_REG_SIZE/8)
#define CPU_MBZ_COUNT					(CPU_MBZ_CONTEXT_PC + INT_REG_SIZE/8)
#define CPU_MBZ_CONTEXT_FSR				(CPU_MBZ_COUNT + INT_REG_SIZE/8)

#define MBZ_MSR_USERMODE    			0x00000800  //microblaze in user mode

#ifndef __MUTEK_ASM__

/** Microblaze processor context state */
struct cpu_context_s
{
  reg_t save_mask;       //< what is being saved and restored
  reg_t gpr[32];
  reg_t msr;
  reg_t pc;
  reg_t count; // for debug
# ifdef CONFIG_HEXO_FPU
  reg_t fsr;
# endif
};

#endif

#endif



