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

/**
   @file CPU atomic operations
*/

#if !defined(ATOMIC_H_) || defined(CPU_ATOMIC_H_)
#error This file can not be included directly
#else

#if defined(CONFIG_SMP)

#define CPU_ATOMIC_H_

#define HAS_CPU_ATOMIC_INC

static inline bool_t
cpu_atomic_inc(volatile atomic_int_t *a)
{
	reg_t tmp = 0, tmp2;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"add     %[tmp], %[tmp], #1   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		: [atomic] "r" (a)
		);

	return tmp != 0;
}

#define HAS_CPU_ATOMIC_DEC

static inline bool_t
cpu_atomic_dec(volatile atomic_int_t *a)
{
	reg_t tmp = 0, tmp2;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"sub     %[tmp], %[tmp], #1   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		: [atomic] "r" (a)
		);

	return tmp != 0;
}

#define HAS_CPU_ATOMIC_TESTSET

static inline bool_t
cpu_atomic_bit_testset(volatile atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2, tmp3;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"bne     2f           \n\t"
		"orr     %[tmp2], %[tmp], %[mask]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
		"2:                   \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
		, [tmp3] "=&r"(tmp3), [clobber] "=m" (*a)
		: [mask] "r" (mask), [atomic] "r" (a)
		);

	return tmp != 0;
}

#define HAS_CPU_ATOMIC_WAITSET

static inline void
cpu_atomic_bit_waitset(volatile atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp = 0, tmp2;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"bne     1b           \n\t"
		"orr     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %2, %[tmp], [%[atomic]]   \n\t"
		"tst     %2, #1       \n\t"
		"bne     1b           \n\t"
		: [tmp] "=&r" (tmp), [clobber] "=m" (*a)
		, [tmp2] "=&r" (tmp2)
		: [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_TESTCLR

static inline bool_t
cpu_atomic_bit_testclr(volatile atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2, tmp3;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"beq     2f           \n\t"
		"bic     %[tmp2], %[tmp], %[mask]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
		"2:                   \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		, [tmp3] "=&r"(tmp3)
		: [mask] "r" (mask), [atomic] "r" (a)
		);

	return tmp != 0;
}

#define HAS_CPU_ATOMIC_WAITCLR

static inline void
cpu_atomic_bit_waitclr(volatile atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp = 0, tmp2;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"beq     1b           \n\t"
		"bic     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %2, %[tmp], [%[atomic]]   \n\t"
		"tst     %2, #1       \n\t"
		"bne     1b           \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		: [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_SET

static inline void
cpu_atomic_bit_set(volatile atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"orr     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		: [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_CLR

static inline void
cpu_atomic_bit_clr(volatile atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2;

	asm volatile(
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"bic     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		: [mask] "r" (mask), [atomic] "r" (a)
		);
}

#else /* SMP */

#include "cpu/common/include/cpu/hexo/atomic_na.h"

#endif

#endif
