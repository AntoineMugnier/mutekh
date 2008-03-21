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

#define CPU_ATOMIC_H_

#define HAS_CPU_ATOMIC_INC

static inline bool_t
cpu_atomic_inc(volatile atomic_int_t *a)
{
  reg_t tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%2         \n"
     "	addi	%0,%0,1		\n"
     "  stwcx.	%0,0,%2		\n"
     "	bne-	1b		\n"
     : "=&r" (tmp), "=m" (*a)
     : "r" (a), "m" (*a)
     );

  return tmp != 0;
}

#define HAS_CPU_ATOMIC_DEC

static inline bool_t
cpu_atomic_dec(volatile atomic_int_t *a)
{
  reg_t tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%2         \n"
     "	addi	%0,%0,-1		\n"
     "  stwcx.	%0,0,%2		\n"
     "	bne-	1b		\n"
     : "=&r" (tmp), "=m" (*a)
     : "r" (a), "m" (*a)
     );

  return tmp != 0;
}

#define HAS_CPU_ATOMIC_TESTSET

static inline bool_t
cpu_atomic_bit_testset(volatile atomic_int_t *a, uint_fast8_t n)
{
  reg_t mask = 1 << n;
  reg_t result, tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%3         \n"
     "	and.	%2,%0,%5		\n"
	 "  bne		2f				\n"
     "	or		%0,%0,%5		\n"
     "  stwcx.	%0,0,%3		\n"
     "	bne-	1b		\n"
	 "2:                \n"
     : "=&r" (tmp), "=m" (*a), "=&r"(result)
     : "r" (a), "m" (*a), "r"(mask)
     );

  return result != 0;
}

#define HAS_CPU_ATOMIC_WAITSET

static inline void
cpu_atomic_bit_waitset(volatile atomic_int_t *a, uint_fast8_t n)
{
  reg_t mask = 1 << n;
  reg_t result, tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%3         \n"
     "	and.	%2,%0,%5		\n"
	 "  bne		1b				\n"
     "	or		%0,%0,%5		\n"
     "  stwcx.	%0,0,%3		\n"
     "	bne-	1b		\n"
     : "=&r" (tmp), "=m" (*a), "=&r"(result)
     : "r" (a), "m" (*a), "r"(mask)
     );
}

#define HAS_CPU_ATOMIC_TESTCLR

static inline bool_t
cpu_atomic_bit_testclr(volatile atomic_int_t *a, uint_fast8_t n)
{
  reg_t mask = 1 << n;
  reg_t result, tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%3         \n"
     "	and.	%2,%0,%5		\n"
	 "  beq		2f				\n"
     "	xor		%0,%0,%5		\n"
     "  stwcx.	%0,0,%3		\n"
     "	bne-	1b		\n"
	 "2:                \n"
     : "=&r" (tmp), "=m" (*a), "=&r"(result)
     : "r" (a), "m" (*a), "r"(mask)
     );

  return result != 0;
}

#define HAS_CPU_ATOMIC_WAITCLR

static inline void
cpu_atomic_bit_waitclr(volatile atomic_int_t *a, uint_fast8_t n)
{
  reg_t mask = 1 << n;
  reg_t result, tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%3         \n"
     "	and.	%2,%0,%5		\n"
	 "  beq		1b				\n"
     "	xor		%0,%0,%5		\n"
     "  stwcx.	%0,0,%3		\n"
     "	bne-	1b		\n"
     : "=&r" (tmp), "=m" (*a), "=&r"(result)
     : "r" (a), "m" (*a), "r"(mask)
     );
}

#define HAS_CPU_ATOMIC_SET

static inline void
cpu_atomic_bit_set(volatile atomic_int_t *a, uint_fast8_t n)
{
  *a |= 1 << n;
}

#define HAS_CPU_ATOMIC_CLR

static inline void
cpu_atomic_bit_clr(volatile atomic_int_t *a, uint_fast8_t n)
{
  *a &= ~(1 << n);
}

#define HAS_CPU_ATOMIC_TEST

static inline bool_t
cpu_atomic_bit_test(volatile atomic_int_t *a, uint_fast8_t n)
{
  return (*a & (1 << n)) == 0;
}

#endif
