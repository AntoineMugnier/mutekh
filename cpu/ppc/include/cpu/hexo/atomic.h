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

#define CPU_ATOMIC_H_

typedef int32_t atomic_int_t;

typedef int32_t atomic_fast16_int_t;
#define HAS_CPU_ATOMIC_FAST16_ALIAS

typedef int32_t atomic_fast8_int_t;
#define HAS_CPU_ATOMIC_FAST8_ALIAS


#define HAS_CPU_ATOMIC_ADD

ALWAYS_INLINE atomic_int_t
__cpu_atomic_add(atomic_int_t *a, atomic_int_t value)
{
        reg_t result, tmp;

	asm volatile(
		"1:                              \n"
		"  lwarx   %[result], 0, %[atomic]  \n"
		"  add     %[tmp], %[result], %[value] \n"
		"  stwcx.  %[tmp], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [tmp] "=&r" (tmp), [result] "=&b" (result), "=m" (*a)
		: [atomic] "r" (a), [value] "r" (value)
        : "cr0"
		);

	return result;
}

#define HAS_CPU_ATOMIC_OR

ALWAYS_INLINE atomic_int_t
__cpu_atomic_or(atomic_int_t *a, atomic_int_t value)
{
        reg_t result, tmp;

	asm volatile(
		"1:                              \n"
		"  lwarx   %[result], 0, %[atomic]  \n"
		"  or      %[tmp], %[result], %[value] \n"
		"  stwcx.  %[tmp], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [tmp] "=&r" (tmp), [result] "=&b" (result), "=m" (*a)
		: [atomic] "r" (a), [value] "r" (value)
        : "cr0"
		);

	return result;
}

#define HAS_CPU_ATOMIC_XOR

ALWAYS_INLINE atomic_int_t
__cpu_atomic_xor(atomic_int_t *a, atomic_int_t value)
{
        reg_t result, tmp;

	asm volatile(
		"1:                              \n"
		"  lwarx   %[result], 0, %[atomic]  \n"
		"  xor     %[tmp], %[result], %[value] \n"
		"  stwcx.  %[tmp], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [tmp] "=&r" (tmp), [result] "=&b" (result), "=m" (*a)
		: [atomic] "r" (a), [value] "r" (value)
        : "cr0"
		);

	return result;
}

#define HAS_CPU_ATOMIC_AND

ALWAYS_INLINE atomic_int_t
__cpu_atomic_and(atomic_int_t *a, atomic_int_t value)
{
        reg_t result, tmp;

	asm volatile(
		"1:                              \n"
		"  lwarx   %[result], 0, %[atomic]  \n"
		"  and     %[tmp], %[result], %[value] \n"
		"  stwcx.  %[tmp], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [tmp] "=&r" (tmp), [result] "=&b" (result), "=m" (*a)
		: [atomic] "r" (a), [value] "r" (value)
        : "cr0"
		);

	return result;
}

#define HAS_CPU_ATOMIC_SWAP

ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
        reg_t result;

	asm volatile(
		"1:                              \n"
		"  lwarx   %[result], 0, %[atomic]  \n"
		"  stwcx.  %[value], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [result] "=&b" (result), "=m" (*a)
		: [atomic] "r" (a), [value] "r" (value)
        : "cr0"
		);

	return result;
}

#define HAS_CPU_ATOMIC_INC

ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
	reg_t tmp;
	
	asm volatile(
		"1:                              \n"
		"  lwarx   %[tmp], 0, %[atomic]  \n"
		"  addi    %[tmp], %[tmp], 1     \n"
		"  stwcx.  %[tmp], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [tmp] "=&b" (tmp), "=m" (*a)
		: [atomic] "r" (a)
        : "cr0"
		);
	
	return tmp != 0;
}

#define HAS_CPU_ATOMIC_DEC

ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
	reg_t tmp;
	
	asm volatile(
		"1:                              \n"
		"  lwarx   %[tmp], 0, %[atomic]  \n"
		"  addi    %[tmp], %[tmp], -1    \n"
		"  stwcx.  %[tmp], 0, %[atomic]  \n"
		"  bne-    1b                    \n"
		: [tmp] "=&b" (tmp), "=m" (*a)
		: [atomic] "r" (a)
        : "cr0"
		);
	
	return tmp != 0;
}

#define HAS_CPU_ATOMIC_TESTSET

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t result, tmp;

    asm volatile(
        "1:                                   \n"
        "  lwarx   %[tmp], 0, %[atomic]       \n"
        "  and.    %[result], %[tmp], %[mask] \n"
        "  bne     2f                         \n"
        "  or      %[tmp], %[tmp], %[mask]    \n"
        "  stwcx.  %[tmp], 0, %[atomic]       \n"
        "  bne-    1b                         \n"
        "2:                                   \n"
        : [tmp] "=&r" (tmp), [result] "=&r"(result), "=m" (*a)
        : [mask] "r"(mask), [atomic] "r" (a)
        : "cr0"
        );
    
	return result != 0;
}

#define HAS_CPU_ATOMIC_WAITSET

ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t result, tmp;

    asm volatile(
        "1:                                   \n"
        "  lwarx   %[tmp], 0, %[atomic]       \n"
        "  and.    %[result], %[tmp], %[mask] \n"
        "  bne     1b                         \n"
        "  or      %[tmp], %[tmp], %[mask]    \n"
        "  stwcx.  %[tmp], 0, %[atomic]       \n"
        "  bne-    1b                         \n"
        : [tmp] "=&r" (tmp), [result] "=&r"(result), "=m" (*a)
        : [mask] "r"(mask), [atomic] "r" (a)
        : "cr0"
        );
}

#define HAS_CPU_ATOMIC_TESTCLR

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t result, tmp;

    asm volatile(
        "1:                                   \n"
        "  lwarx   %[tmp], 0, %[atomic]       \n"
        "  and.    %[result], %[tmp], %[mask] \n"
        "  beq     2f                         \n"
        "  andc    %[tmp], %[tmp], %[mask]    \n"
        "  stwcx.  %[tmp], 0, %[atomic]       \n"
        "  bne-    1b                         \n"
        "2:                                   \n"
        : [tmp] "=&r" (tmp), [result] "=&r"(result), "=m" (*a)
        : [mask] "r"(mask), [atomic] "r" (a)
        : "cr0"
        );
    
	return result != 0;
}

#define HAS_CPU_ATOMIC_WAITCLR

ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t result, tmp;

    asm volatile(
        "1:                                   \n"
        "  lwarx   %[tmp], 0, %[atomic]       \n"
        "  and.    %[result], %[tmp], %[mask] \n"
        "  beq     1b                         \n"
        "  andc    %[tmp], %[tmp], %[mask]    \n"
        "  stwcx.  %[tmp], 0, %[atomic]       \n"
        "  bne-    1b                         \n"
        : [tmp] "=&r" (tmp), [result] "=&r"(result), "=m" (*a)
        : [mask] "r"(mask), [atomic] "r" (a)
        : "cr0"
        );
}

#define HAS_CPU_ATOMIC_SET

ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp;

    asm volatile(
        "1:                                 \n"
        "   lwarx   %[tmp], 0, %[atomic]    \n"
        "   or      %[tmp], %[tmp], %[mask] \n"
        "   stwcx.  %[tmp], 0, %[atomic]    \n"
        "   bne-    1b                      \n"
        : [tmp] "=&r" (tmp), "=m" (*a)
        : [mask] "r"(mask), [atomic] "r" (a)
        : "cr0"
        );
}

#define HAS_CPU_ATOMIC_CLR

ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
	reg_t mask = 1 << n;
	reg_t tmp;

    asm volatile(
        "1:                                 \n"
        "   lwarx   %[tmp], 0, %[atomic]    \n"
        "   andc    %[tmp], %[tmp], %[mask] \n"
        "   stwcx.  %[tmp], 0, %[atomic]    \n"
        "   bne-    1b                      \n"
        : [tmp] "=&r" (tmp), "=m" (*a)
        : [mask] "r"(mask), [atomic] "r" (a)
        : "cr0"
        );
}

#define HAS_CPU_ATOMIC_COMPARE_AND_SWAP

ALWAYS_INLINE bool_t
__cpu_atomic_compare_and_swap(atomic_int_t *a, atomic_int_t old, atomic_int_t future)
{
    reg_t loaded;

    asm volatile (
        "       sync                                 \n"
        "1:     lwarx   %[loaded], 0, %[atomic]      \n"
        "       cmpw    %[loaded], %[old]            \n"
        "       bne-    2f                           \n"
        "       stwcx.  %[future], 0, %[atomic]         \n"
        "       bne-    1b                           \n"
        "       isync                                \n"
        "2:                                          \n"
        : [loaded] "=&r" (loaded), "=m" (*a)
        : [old] "r" (old), [future] "r" (future), [atomic] "r" (a)
        : "cr0"
        );

    return loaded == old;
}

#endif
