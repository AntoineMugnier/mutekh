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

#include <hexo/cpu.h>
#include "cpu/hexo/specific.h"

/** @multiple @this specify context save mask values */
# define CPU_ARM_CONTEXT_RESTORE_CALLEE   1
# define CPU_ARM_CONTEXT_RESTORE_CALLER   2

# define CPU_ARM_CONTEXT_RESTORE_NONE     (~3)

/** @multiple @this describes @ref cpu_context_s field offset */
#define CPU_ARM_CONTEXT_SAVE_MASK      0
#define CPU_ARM_CONTEXT_GPR(n)         ((CPU_ARM_CONTEXT_SAVE_MASK + 1 + n)*4)
#define CPU_ARM_CONTEXT_CPSR           CPU_ARM_CONTEXT_GPR(16)

#ifndef __MUTEK_ASM__

struct cpu_context_s
{
    reg_t save_mask;       //< what is being saved and restored
    reg_t gpr[16];
    reg_t cpsr;
};

#endif

#endif

