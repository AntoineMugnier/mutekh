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

/** @multiple @this specify context save mask values */
#define CPU_ARM_CONTEXT_RESTORE_CALLEE   1
#define CPU_ARM_CONTEXT_RESTORE_CALLER   2

#define CPU_ARM_CONTEXT_RESTORE_NONE     (~3)


/** @multiple @this describes @ref cpu_context_s field offset */
# define CPU_ARM_CONTEXT_R0             0
# define CPU_ARM_CONTEXT_R1             4
# define CPU_ARM_CONTEXT_R2             8
# define CPU_ARM_CONTEXT_R3             12
# define CPU_ARM_CONTEXT_R4             16
# define CPU_ARM_CONTEXT_R5             20
# define CPU_ARM_CONTEXT_R6             24
# define CPU_ARM_CONTEXT_R7             28
# define CPU_ARM_CONTEXT_R8             32
# define CPU_ARM_CONTEXT_R9             36
# define CPU_ARM_CONTEXT_R10            40
# define CPU_ARM_CONTEXT_R11            44
# define CPU_ARM_CONTEXT_R12            48
# define CPU_ARM_CONTEXT_SP             52
# define CPU_ARM_CONTEXT_LR             56
# define CPU_ARM_CONTEXT_PC             60
# define CPU_ARM_CONTEXT_SYS            64
# if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
#  define CPU_ARM_CONTEXT_CFSR          68
# endif

# ifndef __MUTEK_ASM__

struct cpu_context_s
{
    reg_t gpr[16];
    struct {
        uint16_t primask;
        uint16_t exc_mode; /* indicate if the context has been interrupted */
    }           sys;
#  if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
    uint32_t cfsr;
#  endif
};

#  define CPU_CONTEXT_REG_FIRST 0
#  define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "xpsr"
#  define CPU_CONTEXT_REG_COUNT 17

# endif

#endif

