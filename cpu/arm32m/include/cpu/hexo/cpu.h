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

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define ARM_PSR_IRQ_DIS     0x80
#define ARM_PSR_FIQ_DIS     0x40

#define ARM_PSR_MODE_USER   0x10
#define ARM_PSR_MODE_FIQ    0x11
#define ARM_PSR_MODE_IRQ    0x12
#define ARM_PSR_MODE_SUPER  0x13
#define ARM_PSR_MODE_ABORT  0x17
#define ARM_PSR_MODE_UNDEF  0x1b

#define ARM_PSR_EE          0x200

#include <hexo/endian.h>

#define CPU_CPU_H_

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	16

#define CPU_GPREG_NAMES                                 \
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",     \
    "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"

ALWAYS_INLINE cpu_id_t
cpu_id(void)
{
  return 0;
}

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  return 1;
}

ALWAYS_INLINE
reg_t cpu_get_stackptr()
{
    reg_t ret;
    asm("mov %0, sp": "=r"(ret));
    return ret;
}

ALWAYS_INLINE void
cpu_trap()
{
  asm volatile ("bkpt");
}

ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
}

ALWAYS_INLINE size_t cpu_dcache_line_size()
{
  return 0;
}

#endif

