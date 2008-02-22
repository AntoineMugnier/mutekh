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

static inline reg_t
cpu_mips_mfc0(const uint8_t id, const uint8_t sel)
{
  reg_t		reg;

  asm volatile (
#if __mips >= 32 
		"mfc0	%0,	$%1, %2	\n"
#else
		"mfc0	%0,	$%1		\n"
#endif
		: "=r" (reg)
		: "i" (id)
#if __mips >= 32 
		, "i" (sel)
#endif
		);

  return reg;
}

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
#ifdef CONFIG_ARCH_SOCLIB
		" lw $0, (%0)"
		: : "r" (ptr)
#else
		" cache %0, %1"
		: : "i" (0x11) , "R" (*(uint8_t*)(ptr))
#endif
		: "memory"
		);
}

static inline size_t cpu_dcache_line_size()
{
  reg_t r = cpu_mips_mfc0(16, 1);

  if (BIT_EXTRACT(r, 31))
    {
      r = BITS_EXTRACT_FL(r, 10, 12);

      if (r)
	return 2 << r;
    }

  return 0;
}

#endif

