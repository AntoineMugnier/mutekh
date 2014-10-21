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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012
*/

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

#ifndef __MUTEK_ASM__

/** general purpose regsiters count */
# define CPU_GPREG_COUNT	16

# define CPU_GPREG_NAMES                                       \
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",             \
    "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"

ALWAYS_INLINE cpu_id_t
cpu_id(void)
{
  /** FIXME */

  return 0;
}

ALWAYS_INLINE
reg_t cpu_get_stackptr()
{
    reg_t ret;
    asm ("mov %0, sp" : "=r" (ret));
    return ret;
}

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == CONFIG_ARCH_BOOTSTRAP_CPU_ID;
}

ALWAYS_INLINE void
cpu_trap()
{
  asm volatile ("breakpoint");
}

ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
}

ALWAYS_INLINE size_t cpu_dcache_line_size()
{
  reg_t ret;
  asm ("mfsr %0, 260" : "=r" (ret)); /* CONFIG1 register */
  ret = (ret >> 3) & 7;
  return ret ? 2 << ret : 0;
}

#endif  /* __MUTEK_ASM__ */

#endif

