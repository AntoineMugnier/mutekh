/*
 *   This file is part of MutekH.
 *
 *   MutekH is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   MutekH is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with MutekH; if not, write to the Free Software Foundation,
 *   Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 *   Copyright Francois Charot <charot@irisa.fr>  (c) 2008
 *   INRIA Rennes Bretagne Atlantique
 *
 */


#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#include <hexo/endian.h>

#define CPU_CPU_H_

#ifdef CONFIG_ARCH_SMP
extern void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

/** general purpose registers count */
#define CPU_GPREG_COUNT	32

#define CPU_GPREG_NAMES {					\
    "pc", "at", "r2", "r3", "r4", "r5", "r6", "r7",		     \
      "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",	 \
      "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",\
      "et", "bt", "gp", "sp", "fp", "ea", "ba", "ra",		 \
      }

#define CPU_FAULT_COUNT 16

#define CPU_FAULT_NAMES	 {                      \
      "Interrupt",                              \
      "Address error (Load)",                   \
      "Address error (Store)",                  \
      "Instruction bus error",                  \
      "Data bus error",                         \
      "Syscall",                                \
      "Break point",                            \
      "Reserved instruction",                   \
      "Overflow",                               \
      "Trap",                                   \
      "Reserved exception",                     \
      "Floating point",                         \
      }

#define cpu_nios_read_ctrl_reg(id)              \
  ({                                            \
    reg_t _reg;                                 \
                                                \
    __asm__ volatile ("rdctl	%0, ctl%1 \n"   \
                      : "=r" (_reg)             \
                      : "i" (id)                \
                      );                        \
    _reg;                                       \
  })


#define cpu_nios_write_ctrl_reg(id, val)        \
  ({                                            \
    reg_t _reg = val;                           \
                                                \
    __asm__ volatile ("wrctl	ctl%1, %0  \n"  \
                      :: "r" (_reg)             \
                      , "i" (id)                \
                     );                         \
  })

#define CPU_TYPE_NAME nios

static inline cpu_id_t
cpu_id(void)
{
  reg_t		reg;

  __asm__ volatile (
		    "	rdctl	%0, cpuid     \n"
		    : "=r" (reg)
		    );

  return reg;
}

static inline bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == 0;
}

/**
   cpu cycle touner type
*/

typedef uint32_t cpu_cycle_t;

/**
   cpu cycle counter read function
*/

static inline cpu_cycle_t
cpu_cycle_count(void)
{
  return cpu_nios_read_ctrl_reg(31);
}


static inline void
cpu_trap()
{
  __asm__ volatile ("trap");
}

static inline void *cpu_get_cls(cpu_id_t cpu_id)
{
#ifdef CONFIG_ARCH_SMP
  return cpu_local_storage[cpu_id];
#endif
  return NULL;
}


static inline void cpu_dcache_invld(void *ptr)
{
  __asm__ volatile (
# ifdef CONFIG_ARCH_SOCLIB
		    " ldw zero, (%0)"
		    : : "r" (ptr)
# else
		      "nop"::
# endif
		    : "memory"
		    );
}

static inline size_t cpu_dcache_line_size()
{
  return 8;
}

#endif

