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

#ifndef ATOMIC_H_
#define ATOMIC_H_

#include <hexo/decls.h>
#include <hexo/types.h>
#include <hexo/ordering.h>

/**
  @file
  @module {Core::Hardware abstraction layer}
  @short Atomic memory operations primitives

  When SMP is enabled, atomic operations are provided either by
  processor atomic instructions or by spin locked access to a regular
  integer value, depending on processor and platform support.

  In the same way, spin lock implementation can either rely on
  processor atomic operations or use an architecture specific locking
  mechanism.

  Atomic functions may include some memory barriers to ensure the
  consistency of memory accesses on weakly-ordered memory
  architectures.
*/

/** Atomic value type */
typedef struct arch_atomic_s atomic_t;

/** @this sets atomic integer value in memory */
ALWAYS_INLINE void atomic_set(atomic_t *a, atomic_int_t value);

/** @this gets atomic integer value in memory */
ALWAYS_INLINE atomic_int_t atomic_get(atomic_t *a);

/** @this atomicaly increments integer value in memory.
   @return 0 if new atomic value is 0. */
ALWAYS_INLINE bool_t atomic_inc(atomic_t *a);

/** @this atomicaly decrements integer value in memory
   @return 0 if new atomic value is 0. */
ALWAYS_INLINE bool_t atomic_dec(atomic_t *a);

/** @this atomicaly sets bit in integer value in memory */
ALWAYS_INLINE void atomic_bit_set(atomic_t *a, uint_fast8_t n);

/** @this atomicaly tests and sets bit in integer value in memory
   @return 0 if bit was cleared before. */
ALWAYS_INLINE bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n);

/** @this atomicaly clears bit in integer value in memory */
ALWAYS_INLINE void atomic_bit_clr(atomic_t *a, uint_fast8_t n);

/** @this atomicaly tests and clears bit in integer value in memory
   @return 0 if bit was cleared before. */
ALWAYS_INLINE bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n);

/** @this tests bit in integer value in memory
   @return 0 if bit is cleared. */
ALWAYS_INLINE bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n);

/** @this compares memory to old and replace with future if they are the same.
   @return true if exchanged */
ALWAYS_INLINE bool_t atomic_compare_and_swap(atomic_t *a, atomic_int_t old, atomic_int_t future);

#if 0
/** ALWAYS_INLINE atomic value initializer */
# define ATOMIC_INITIALIZER(n)	/* defined in implementation */
#endif


#if defined(CONFIG_CPU_SMP_CAPABLE) && defined(CONFIG_ARCH_SMP_CAPABLE)
/* 
   We have better using processor atomic operations when supported
   even for single processor systems because atomic_na implementation
   need to disable and restore interrupts for each access.
 */
# include <cpu/hexo/atomic.h>
#else
/*
  Atomic operations are performed by disabling interrupts in other
  single processor cases.
 */
# include <cpu/common/include/cpu/hexo/atomic_na.h>
#endif



#ifdef HAS_CPU_ATOMIC_COMPARE_AND_SWAP

/* Provide some default implementations based on compare and swap for
   other atomic functions unless a better implementation has been
   defined in processor support code. */

# ifndef HAS_CPU_ATOMIC_INC
#  define HAS_CPU_ATOMIC_INC
ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old + 1));

  return old + 1 != 0;
}
# endif

# ifndef HAS_CPU_ATOMIC_DEC
#  define HAS_CPU_ATOMIC_DEC
ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old - 1));

  return old - 1 != 0;
}
# endif

# ifndef HAS_CPU_ATOMIC_TESTSET
#  define HAS_CPU_ATOMIC_TESTSET
ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
  atomic_int_t mask = 1 << n;
  atomic_int_t old;
  bool_t res;

  do {
    order_smp_read();
    old = *a;
    res = (old & mask) != 0;
  } while (!res && !__cpu_atomic_compare_and_swap(a, old, old | mask));

  return res;
}
# endif

# ifndef HAS_CPU_ATOMIC_TESTCLR
#  define HAS_CPU_ATOMIC_TESTCLR
ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
  atomic_int_t mask = 1 << n;
  atomic_int_t old;
  bool_t res;

  do {
    order_smp_read();
    old = *a;
    res = (old & mask) != 0;
  } while (res && !__cpu_atomic_compare_and_swap(a, old, old & ~mask));

  return res;
}
# endif
#endif

#if defined(HAS_CPU_ATOMIC_TESTSET) && !defined(HAS_CPU_ATOMIC_WAITSET)
# define HAS_CPU_ATOMIC_WAITSET
ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
  while (__cpu_atomic_bit_testset(a, n))
    ;
}
#endif

#if defined(HAS_CPU_ATOMIC_TESTCLR) && !defined(HAS_CPU_ATOMIC_WAITCLR)
# define HAS_CPU_ATOMIC_WAITCLR
ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
  while (!__cpu_atomic_bit_testclr(a, n))
    ;
}
#endif

#if defined(HAS_CPU_ATOMIC_TESTSET) && !defined(HAS_CPU_ATOMIC_SET)
# define HAS_CPU_ATOMIC_SET
ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
  __cpu_atomic_bit_testset(a, n);
}
#endif

#if defined(HAS_CPU_ATOMIC_TESTCLR) && !defined(HAS_CPU_ATOMIC_CLR)
#define HAS_CPU_ATOMIC_CLR
ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
  __cpu_atomic_bit_testclr(a, n);
}
#endif

#include <arch/hexo/atomic.h>

#endif

