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

#define CPU_CPU_H_

#include <hexo/interrupt.h>

#include "pmode.h"

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	8

#define CPU_GPREG_NAMES { "edi", "esi", "ebp", "esp", "ebx", "edx", "ecx", "eax" }

/**
   x86 apic boot strap processor (BSP)
   @return true if processor is the bootstrap processor
*/

static inline bool_t
cpu_isbootstrap(void)
{
  uint64_t	msr;

  asm ("rdmsr\n"
       : "=A" (msr)
       : "c" (0x1b)
       );

  return msr & 0x100 ? 1 : 0;
}

/**
   cpu cycle touner type
*/

typedef uint64_t cpu_cycle_t;

/**
   cpu cycle counter read function
*/

static inline cpu_cycle_t
cpu_cycle_count(void)
{
  uint32_t      low, high;

  asm volatile("rdtsc" : "=a" (low), "=d" (high));

  return (low | ((uint64_t)high << 32));
}

struct cpu_cld_s
{
#ifdef CONFIG_SMP
  /* pointer to CPU local storage */
  void				*cpu_local_storage;
#endif
  /* CPU id */
  uint32_t			id;
  /* CPU Interrupt descriptor table */
  struct cpu_x86_gatedesc_s	idt[CPU_MAX_INTERRUPTS];
};

#endif

