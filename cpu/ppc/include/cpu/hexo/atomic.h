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

#if 0
static inline bool_t
cpu_atomic_inc(volatile atomic_int_t *a)
{
  reg_t tmp;

  asm volatile
    (
     "1:			\n"
     "	lwarx	%0,0,%2         \n"
     "	addic	%0,%0,1		\n"
     "  stwcx.	%0,0,%2		\n"
     "	bne	1b		\n"
     : "=&r" (t), "=m" (a)
     : "r" (&a), "m" (a)
     : "cc"
     );

  return zero;
}

#define HAS_CPU_ATOMIC_DEC

#define HAS_CPU_ATOMIC_TESTSET
#define HAS_CPU_ATOMIC_WAITSET
#define HAS_CPU_ATOMIC_TESTCLR
#define HAS_CPU_ATOMIC_WAITCLR
#define HAS_CPU_ATOMIC_SET
#define HAS_CPU_ATOMIC_CLR
#define HAS_CPU_ATOMIC_TEST

#endif

#endif

