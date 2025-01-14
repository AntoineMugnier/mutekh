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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech
*/

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

#include <hexo/asm.h>

/** sparc psr trap enabled bit */
#define SPARC_PSR_TRAP_ENABLED     0x20
/** sparc psr previous super user mode */
#define SPARC_PSR_PREV_SUSER_MODE 0x40
/** sparc psr current super user mode */
#define SPARC_PSR_SUSER_MODE      0x80
/** sparc psr proc interrupt level bits */
#define SPARC_PSR_PIL_MASK        0xf00
/** sparc psr fpu enabled */
#define SPARC_PSR_FPU_ENABLED     0x1000
/** sparc current window pointer mask */
#define SPARC_PSR_CWP_MASK        0x1f

#define SPARC_TRAP_USERBREAK    1
#define SPARC_TRAP_WINFLUSH     3

/** register window save area on stack */
#define SPARC_STACK_REDZONE     (16 * INT_REG_SIZE/8)

/** general purpose regsiters count */
# define CPU_GPREG_COUNT	32

# define CPU_GPREG_NAMES                                                          \
                "%g1", "%g2", "%g3", "%g4", "%g5", "%g6", "%g7",             \
                "%o0", "%o1", "%o2", "%o3", "%o4", "%o5", "%o6", "%o7",             \
                "%l0", "%l1", "%l2", "%l3", "%l4", "%l5", "%l6", "%l7",             \
                "%i0", "%i1", "%i2", "%i3", "%i4", "%i5", "%i6", "%i7"

ALWAYS_INLINE cpu_id_t
cpu_id(void)
{
  reg_t ret;
  asm("CPU_ID %0" : "=r" (ret));
  return ret;
}

ALWAYS_INLINE
reg_t cpu_get_stackptr(void)
{
    reg_t ret;
    asm("mov %%o6, %0"
        : "=r" (ret));
    return ret;
}

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == CONFIG_ARCH_BOOTSTRAP_CPU_ID;
}

ALWAYS_INLINE void
cpu_trap(void)
{
  asm volatile ("ta %0 \n"
                "nop \n"
                : : "i" (SPARC_TRAP_USERBREAK) );
}

ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
}

ALWAYS_INLINE void cpu_dcache_flush(void *ptr)
{
}

ALWAYS_INLINE size_t cpu_dcache_line_size(void)
{
  return CONFIG_CPU_CACHE_LINE;
}

#endif

