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

/**
   @file CPU atomic operations
*/

#if !defined(ATOMIC_H_) || defined(CPU_ATOMIC_H_)
#error This file can not be included directly
#else

#include "asm.h"

#define CPU_ATOMIC_H_

#define HAS_CPU_ATOMIC_ADD

ALWAYS_INLINE atomic_int_t
__cpu_atomic_add(atomic_int_t *a, atomic_int_t value)
{
	reg_t tmp, tmp2, tmp3;
        THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"add     %[tmp2], %[tmp], %[value]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
                , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
                : [atomic] "r" (a), [value] "r" (value)
		);

	return tmp;
}

#define HAS_CPU_ATOMIC_OR

ALWAYS_INLINE atomic_int_t
__cpu_atomic_or(atomic_int_t *a, atomic_int_t value)
{
	reg_t tmp, tmp2, tmp3;
        THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"orr     %[tmp2], %[tmp], %[value]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
                , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
                : [atomic] "r" (a), [value] "r" (value)
		);

	return tmp;
}

#define HAS_CPU_ATOMIC_XOR

ALWAYS_INLINE atomic_int_t
__cpu_atomic_xor(atomic_int_t *a, atomic_int_t value)
{
	reg_t tmp, tmp2, tmp3;
        THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"eor     %[tmp2], %[tmp], %[value]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
                , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
                : [atomic] "r" (a), [value] "r" (value)
		);

	return tmp;
}

#define HAS_CPU_ATOMIC_AND

ALWAYS_INLINE atomic_int_t
__cpu_atomic_and(atomic_int_t *a, atomic_int_t value)
{
	reg_t tmp, tmp2, tmp3;
        THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"and     %[tmp2], %[tmp], %[value]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
                , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
                : [atomic] "r" (a), [value] "r" (value)
		);

	return tmp;
}

#define HAS_CPU_ATOMIC_SWAP

ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
	reg_t tmp, tmp2;
        THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"strex   %[tmp2], %[value], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
                , [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
                : [atomic] "r" (a), [value] "r" (value)
		);

	return tmp;
}

#define HAS_CPU_ATOMIC_INC

ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
	reg_t tmp = 0, tmp2;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"add     %[tmp], %[tmp], #1   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [atomic] "r" (a)
		);

	return tmp != 0;
}

#define HAS_CPU_ATOMIC_DEC

ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
	reg_t tmp = 0, tmp2;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"sub     %[tmp], %[tmp], #1   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [atomic] "r" (a)
		);

	return tmp != 0;
}

#define HAS_CPU_ATOMIC_TESTSET

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2, tmp3;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"bne     2f           \n\t"
		"orr     %[tmp2], %[tmp], %[mask]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
		"2:                   \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)
		, [tmp3] "=&r"(tmp3), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [mask] "r" (mask), [atomic] "r" (a)
		);

	return (tmp & mask) != 0;
}

#define HAS_CPU_ATOMIC_WAITSET

ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp = 0, tmp2;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"bne     1b           \n\t"
		"orr     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %2, %[tmp], [%[atomic]]   \n\t"
		"tst     %2, #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [clobber] "=m" (*a)
		, [tmp2] "=&r" (tmp2)
		/*,*/ THUMB_OUT(,)
        : [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_TESTCLR

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2, tmp3;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"beq     2f           \n\t"
		"bic     %[tmp2], %[tmp], %[mask]   \n\t"
		"strex   %[tmp3], %[tmp2], [%[atomic]]   \n\t"
		"tst     %[tmp3], #1       \n\t"
		"bne     1b           \n\t"
		"2:                   \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		, [tmp3] "=&r"(tmp3)
		/*,*/ THUMB_OUT(,)
        : [mask] "r" (mask), [atomic] "r" (a)
		);

	return (tmp & mask) != 0;
}

#define HAS_CPU_ATOMIC_WAITCLR

ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp = 0, tmp2;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"tst     %[tmp], %[mask]       \n\t"
		"beq     1b           \n\t"
		"bic     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %2, %[tmp], [%[atomic]]   \n\t"
		"tst     %2, #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_SET

ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"orr     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_CLR

ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp, tmp2;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                   \n\t"
		"ldrex   %[tmp], [%[atomic]]       \n\t"
		"bic     %[tmp], %[tmp], %[mask]   \n\t"
		"strex   %[tmp2], %[tmp], [%[atomic]]   \n\t"
		"tst     %[tmp2], #1       \n\t"
		"bne     1b           \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2), [clobber] "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [mask] "r" (mask), [atomic] "r" (a)
		);
}

#define HAS_CPU_ATOMIC_COMPARE_AND_SWAP

ALWAYS_INLINE bool_t
__cpu_atomic_compare_and_swap(atomic_int_t *a, atomic_int_t old, atomic_int_t future)
{
	atomic_int_t tmp, loaded;
    THUMB_TMP_VAR;

	asm volatile(
        THUMB_TO_ARM
		"1:                                   \n\t"
		"ldrex   %[loaded], [%[atomic]]       \n\t"
		"cmp     %[loaded], %[old]            \n\t"
		"bne     2f                           \n\t"
		"strex   %[tmp], %[future], [%[atomic]]  \n\t"
		"tst     %[tmp], #1                   \n\t"
		"bne     1b                           \n\t"
        "2:                                   \n\t"
        ARM_TO_THUMB
		: [tmp] "=&r" (tmp), [loaded] "=&r" (loaded), "=m" (*a)
		/*,*/ THUMB_OUT(,)
        : [old] "r" (old), [future] "r" (future), [atomic] "r" (a)
		);

    return loaded == old;
}

#endif

