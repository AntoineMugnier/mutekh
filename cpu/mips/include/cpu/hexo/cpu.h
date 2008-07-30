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

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#include <hexo/endian.h>

#define CPU_CPU_H_

struct cpu_cld_s
{
#ifdef CONFIG_SMP
  /* pointer to CPU local storage */
  void				*cpu_local_storage;
#endif
  /* CPU id */
  uint_fast8_t			id;
};

extern struct cpu_cld_s	*cpu_cld_list[CONFIG_CPU_MAXCOUNT];

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	32

#define CPU_GPREG_NAMES {											   \
"pc", "at", "v0", "v1", "a0", "a1", "a2", "a3",						   \
"t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",						   \
"s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",						   \
"t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra",						   \
}

#define CPU_FAULT_COUNT 16
#define CPU_FAULT_NAMES {											   \
"Interrupt",														   \
"TLB Modification",													   \
"TLB Load error",													   \
"TLB Store error",													   \
"Address error (Load)",												   \
"Address error (Store)",											   \
"Instruction bus error",											   \
"Data bus error",													   \
"Syscall",															   \
"Break point",														   \
"Reserved instruction",												   \
"Coproc unusable",													   \
"Overflow",															   \
"Trap",																   \
"Reserved exception",												   \
"Floating point",													   \
}

#if __mips >= 32 

#define cpu_mips_mfc0(id, sel)			\
({						\
  reg_t _reg;					\
						\
  asm volatile ("mfc0	%0,	$%1, %2	\n"	\
		: "=r" (_reg)			\
		: "i" (id)			\
		, "i" (sel)			\
		);				\
						\
  _reg;						\
})

#else

#define cpu_mips_mfc0(id, sel)			\
({						\
  reg_t _reg;					\
						\
  asm volatile ("mfc0	%0,	$%1 \n"		\
		: "=r" (_reg)			\
		: "i" (id)			\
		);				\
						\
  _reg;						\
})

#endif

static inline cpu_id_t
cpu_id(void)
{
  return cpu_mips_mfc0(15, 1) & 0x000003ff;
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
  return cpu_mips_mfc0(9, 0);
}

static inline void
cpu_trap()
{
  asm volatile ("break 0");
}

static inline void *cpu_get_cls(cpu_id_t cpu_id)
{
#ifdef CONFIG_SMP
  return cpu_cld_list[cpu_id]->cpu_local_storage;
#endif
  return NULL;
}

static inline void cpu_dcache_invld(void *ptr)
{
  asm volatile (
#if __mips >= 32
		" cache %0, %1"
		: : "i" (0x11) , "R" (*(uint8_t*)(ptr))
#else
# ifdef CONFIG_ARCH_SOCLIB
		" lw $0, (%0)"
		: : "r" (ptr)
# else
		"nop"::
# endif
#endif
		: "memory"
		);
}

static inline size_t cpu_dcache_line_size()
{
  reg_t r0 = cpu_mips_mfc0(16, 0);
  reg_t r1 = cpu_mips_mfc0(16, 1);

  if (BIT_EXTRACT(r0, 31))
    {
      r1 = BITS_EXTRACT_FL(r1, 10, 12);

      if (r1)
	return 2 << r1;
    }

  return 8;
}

#endif

