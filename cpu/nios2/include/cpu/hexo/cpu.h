/*
 *   This file is part of MutekH.
 *   
 *   MutekH is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation; version 2.1 of the
 *   License.
 *   
 *   MutekH is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *   
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with MutekH; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 *   02110-1301 USA.
 *
 *   Copyright Francois Charot <charot@irisa.fr>  (c) 2008
 *   INRIA Rennes Bretagne Atlantique
 *
 */


#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

/* named registers */
#define CPU_NIOS2_ZERO 0
#define CPU_NIOS2_AT 1
#define CPU_NIOS2_ET 24
#define CPU_NIOS2_BT 25
#define CPU_NIOS2_GP 26
#define CPU_NIOS2_SP 27
#define CPU_NIOS2_FP 28
#define CPU_NIOS2_EA 29
#define CPU_NIOS2_BA 30
#define CPU_NIOS2_RA 31

#define CPU_NIOS2_CLS_REG r26

#include <hexo/endian.h>

/** general purpose registers count */
# define CPU_GPREG_COUNT	32

# define CPU_GPREG_NAMES 					\
      "zero", "at", "r2", "r3", "r4", "r5", "r6", "r7",           \
      "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",     \
      "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",   \
      "et", "bt", "gp", "sp", "fp", "ea", "ba", "ra"

# define cpu_nios2_read_ctrl_reg(id)              \
  ({                                            \
    reg_t _reg;                                 \
                                                \
    __asm__ volatile ("rdctl	%0, ctl%1 \n"   \
                      : "=r" (_reg)             \
                      : "i" (id)                \
                      );                        \
    _reg;                                       \
  })


# define cpu_nios2_write_ctrl_reg(id, val)        \
  ({                                            \
    reg_t _reg = val;                           \
                                                \
    __asm__ volatile ("wrctl	ctl%1, %0  \n"  \
                      :: "r" (_reg)             \
                      , "i" (id)                \
                     );                         \
  })

ALWAYS_INLINE cpu_id_t
cpu_id(void)
{
  reg_t		reg;

  __asm__ volatile (
		    "	rdctl	%0, cpuid     \n"
		    : "=r" (reg)
		    );

  return reg;
}

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == CONFIG_ARCH_BOOTSTRAP_CPU_ID;
}

ALWAYS_INLINE void
cpu_trap(void)
{
  __asm__ volatile ("trap");
}


ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
#ifdef CONFIG_ARCH_SOCLIB
  __asm__ volatile (
		    " ldw zero, (%0)"
		    : : "r" (ptr)
		    : "memory"
		    );
#endif
}

ALWAYS_INLINE void cpu_dcache_flush(void *ptr)
{
}

ALWAYS_INLINE size_t cpu_dcache_line_size(void)
{
  return 8;
}

#endif

