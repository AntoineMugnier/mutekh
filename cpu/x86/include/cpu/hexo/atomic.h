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


#define _SMPLOCK "lock\n"

#define HAS_CPU_ATOMIC_ADD

ALWAYS_INLINE atomic_int_t
__cpu_atomic_add(atomic_int_t *a, atomic_int_t value)
{
  asm volatile (_SMPLOCK
                "xadd	%1, %0\n"
		: "=m" (*a), "=r" (value)
                : "1" (value)
                : "cc"
                );

  return value;
}

#define HAS_CPU_ATOMIC_SWAP

ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
  asm volatile (_SMPLOCK
                "xchg	%1, %0\n"
		: "=m" (*a), "=r" (value)
                : "1" (value)
		);

  return value;
}

#define HAS_CPU_ATOMIC_INC

ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
  uint8_t		zero;

  asm volatile (_SMPLOCK
                "incl	%0	\n"
		"setnz		%1	\n"
		: "=m" (*a), "=q" (zero)
                :: "cc"
		);

  return zero;
}

#define HAS_CPU_ATOMIC_DEC

ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
  uint8_t		zero;

  asm volatile (_SMPLOCK
                "decl	%0	\n"
		"setnz		%1	\n"
		: "=m" (*a), "=q" (zero)
                :: "cc"
		);

  return zero;
}

#define HAS_CPU_ATOMIC_TESTSET

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
  uint8_t		isset;

  asm volatile (_SMPLOCK
                "btsl	%2, %0	\n"
		"setc		%1	\n"
		: "=m,m" (*a), "=q,q" (isset)
		: "r,I" (n)
                : "cc"
		);

  return isset;
}

#define HAS_CPU_ATOMIC_WAITSET

ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
  asm volatile ("1:	" _SMPLOCK
                "btsl	%1, %0          \n"
                "pause                  \n"
		"jc		1b	\n"
		: "=m,m" (*a)
		: "r,I" (n)
                : "cc"
		);
}

#define HAS_CPU_ATOMIC_TESTCLR

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
  uint8_t		isset;

  asm volatile (_SMPLOCK
                "btrl	%2, %0          \n"
		"setc		%1	\n"
		: "=m,m" (*a), "=q,q" (isset)
		: "r,I" (n)
                : "cc"
		);

  return isset;
}


#define HAS_CPU_ATOMIC_WAITCLR

ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
  asm volatile ("1:	" _SMPLOCK
                "btrl	%1, %0          \n"
                "pause                  \n"
		"jnc		1b	\n"
		: "=m,m" (*a)
		: "r,I" (n)
                : "cc"
		);
}

#define HAS_CPU_ATOMIC_SET

ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
  asm volatile (_SMPLOCK
                "btsl	%1, %0	\n"
		: "=m,m" (*a)
		: "r,I" (n)
                : "cc"
		);
}

#define HAS_CPU_ATOMIC_CLR

ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
  asm volatile (_SMPLOCK
                "btrl	%1, %0	\n"
		: "=m,m" (*a)
		: "r,I" (n)
                : "cc"
		);
}

#define HAS_CPU_ATOMIC_COMPARE_AND_SWAP

ALWAYS_INLINE bool_t
__cpu_atomic_compare_and_swap(atomic_int_t *a, atomic_int_t old, atomic_int_t future)
{
  uint8_t		done;

  asm volatile (_SMPLOCK
                "cmpxchgl	%3, %0	\n"
		"setz		%1	\n"
		: "=m" (*a), "=q" (done)
		: "a" (old), "r" (future)
                : "cc"
		);

  return done;
}

#undef _SMPLOCK

#endif

