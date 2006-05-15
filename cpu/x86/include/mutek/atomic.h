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

static inline __bool_t
cpu_atomic_inc(__atomic_t *a)
{
  uint8_t		zero;

  asm volatile ("lock incl	%0	\n"
		"setnz		%1	\n"
		: "=m" (*a), "=r" (zero)
		);

  return zero;
}

#define HAS_CPU_ATOMIC_DEC

static inline __bool_t
cpu_atomic_dec(__atomic_t *a)
{
  uint8_t		zero;

  asm volatile ("lock decl	%0	\n"
		"setnz		%1	\n"
		: "=m" (*a), "=r" (zero)
		);

  return zero;
}

#define HAS_CPU_ATOMIC_TESTSET

static inline __bool_t
cpu_atomic_bit_testset(__atomic_t *a, uint_fast8_t n)
{
  uint8_t		isset;

  asm volatile ("lock bts	%2, %0	\n"
		"setc		%1	\n"
		: "=m,m" (*a), "=r,r" (isset)
		: "r,I" (n)
		);

  return isset;
}

#define HAS_CPU_ATOMIC_WAITSET

static inline void
cpu_atomic_bit_waitset(__atomic_t *a, uint_fast8_t n)
{
  asm volatile ("1:	lock bts	%1, %0	\n"
		"	jc		1b	\n"
		: "=m,m" (*a)
		: "r,I" (n)
		);
}

#define HAS_CPU_ATOMIC_TESTCLR

static inline __bool_t
cpu_atomic_bit_testclr(__atomic_t *a, uint_fast8_t n)
{
  uint8_t		isset;

  asm volatile ("lock btr	%2, %0	\n"
		"setc		%1	\n"
		: "=m,m" (*a), "=r,r" (isset)
		: "r,I" (n)
		);

  return isset;
}


#define HAS_CPU_ATOMIC_WAITCLR

static inline void
cpu_atomic_bit_waitclr(__atomic_t *a, uint_fast8_t n)
{
  asm volatile ("1:	lock btr	%1, %0	\n"
		"	jnc		1b	\n"
		: "=m,m" (*a)
		: "r,I" (n)
		);
}

#define HAS_CPU_ATOMIC_SET

static inline void
cpu_atomic_bit_set(__atomic_t *a, uint_fast8_t n)
{
  asm volatile ("lock bts	%1, %0	\n"
		: "=m,m" (*a)
		: "r,I" (n)
		);
}

#define HAS_CPU_ATOMIC_CLR

static inline void
cpu_atomic_bit_clr(__atomic_t *a, uint_fast8_t n)
{
  asm volatile ("lock btr	%1, %0	\n"
		: "=m,m" (*a)
		: "r,I" (n)
		);
}

#define HAS_CPU_ATOMIC_TEST

static inline __bool_t
cpu_atomic_bit_test(__atomic_t *a, uint_fast8_t n)
{
  uint8_t		isset;

  asm volatile ("bt		%2, %1	\n"
		"setc		%0	\n"
		: "=r,r" (isset), "=m,m" (*a)
		: "r,I" (n)
		);

  return isset;
}

#endif

