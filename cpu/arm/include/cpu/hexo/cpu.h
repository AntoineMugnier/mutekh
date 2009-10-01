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

#ifdef CONFIG_SMP
extern void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	16

#define CPU_GPREG_NAMES {											   \
"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",						   \
"r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",						   \
}

#define CPU_FAULT_COUNT 7

#define CPU_FAULT_NAMES {			\
"None",					\
"Reset",				\
"Data abort",				\
"FIQ",				\
"IRQ",				\
"Prefetch abort",			\
"Software",			\
}

static inline cpu_id_t
cpu_id(void)
{
#if defined(__ARM_ARCH_6K__)
	uint32_t ret;
	asm volatile("mrc  p15,0,%0,c0,c0,5":"=r"(ret));
	return ret;
#else
# ifdef CONFIG_SMP
#  error Cant compile SMP code without cpuid support
# endif
	return 0;
#endif
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
#if defined(__ARM_ARCH_6K__)
	uint32_t ret;
	asm("mrc  p15,0,%0,c15,c12,1":"=r"(ret));
	return ret;
#elif defined(__ARM_ARCH_4T__)
	/* who cares ? */
	return 0;
#else
# warning No CPU cycle counter
	return 0;
#endif
}

static inline void
cpu_trap()
{
	asm volatile ("swi 0");
}

static inline void *cpu_get_cls(cpu_id_t cpu_id)
{
#ifdef CONFIG_SMP
  return cpu_local_storage[cpu_id];
#endif
  return NULL;
}

static inline void cpu_dcache_invld(void *ptr)
{
#if defined(CONFIG_CPU_CACHE)
	asm("mcr p15, 0, %0, c7, c6, 1"::"r"(ptr));
#endif
}

static inline size_t cpu_dcache_line_size()
{
#if defined(CONFIG_CPU_CACHE)
	uint32_t ret;
	asm("mrc p15, 0, %0, c0, c0, 1":"=r"(ret));

	return (((ret >> 12) & 0x3) + 3) << 1;
#else
	return 16;
#endif
}

#endif

