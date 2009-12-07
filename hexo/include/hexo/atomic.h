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

#ifndef ATOMIC_H_
#define ATOMIC_H_

#include "types.h"

/**
  @file
  @module{Hexo}
  @short Atomic memory operations primitives

  CPU atomic functions @tt cpu_atomic_* use standard integer values of type
  @ref atomic_int_t and provide atomic access when available.

  Atomicity is not garanted if system architecture does not handle
  atomic bus access. Please consider using arch @ref atomic_t and @tt atomic_*
  function instead in general case.

  Some CPU may have partial or missing atomic access capabilities,
  please check for @tt HAS_CPU_ATOMIC_* macro defined in @ref @hexo/atomic.h header.

  Arch atomic functions use architecture specific structures of type
  @ref atomic_t and provide locked access on atomic integer values. It may
  use cpu atomic operations or additional spin lock depending on
  system archicture hardware capabilities. Use it for general purpose
  atomic values access.
 */

#include "cpu/hexo/atomic.h"

/** @multiple @internal */
static bool_t cpu_atomic_inc(volatile atomic_int_t *a);
static bool_t cpu_atomic_dec(volatile atomic_int_t *a);
static bool_t cpu_atomic_bit_testset(volatile atomic_int_t *a, uint_fast8_t n);
static void cpu_atomic_bit_waitset(volatile atomic_int_t *a, uint_fast8_t n);
static bool_t cpu_atomic_bit_testclr(volatile atomic_int_t *a, uint_fast8_t n);
static void cpu_atomic_bit_waitclr(volatile atomic_int_t *a, uint_fast8_t n);
static void cpu_atomic_bit_set(volatile atomic_int_t *a, uint_fast8_t n);
static void cpu_atomic_bit_clr(volatile atomic_int_t *a, uint_fast8_t n);

/** Atomic value type */
typedef struct arch_atomic_s atomic_t;

/** @this sets atomic integer value. The @ref %value parameter is useless. */
static void atomic_set(atomic_t *a, atomic_int_t value);

/** @this gets atomic integer value */
static atomic_int_t atomic_get(atomic_t *a);

/** @this atomicaly increments integer value.
   @return 0 if new atomic value is 0. */
static bool_t atomic_inc(atomic_t *a);

/** @this atomicaly decrements integer value
   @return 0 if new atomic value is 0. */
static bool_t atomic_dec(atomic_t *a);

/**
   @this atomicaly adds an integer value.

   @param a Atomic to add into
   @param val Value to add
   @return the old value contained in a.
 */
static atomic_int_t atomic_add(atomic_t *a, atomic_int_t val);

/** @this atomicaly sets bit in intger value */
static void atomic_bit_set(atomic_t *a, uint_fast8_t n);

/** @this atomicaly tests and sets bit in integer value
   @return 0 if bit was cleared before. */
static bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n);

/** @this atomicaly clears bit in integer value */
static void atomic_bit_clr(atomic_t *a, uint_fast8_t n);

/** @this atomicaly tests and clears bit in integer value
   @return 0 if bit was cleared before. */
static bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n);

/** @this tests bit in integer value
   @return 0 if bit is cleared. */
static bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n);

/** @this compares memory to old and replace with new if they are the same
   @return true if exchanged */
static bool_t atomic_compare_and_swap(volatile atomic_int_t *a, atomic_int_t old, atomic_int_t new);

#if 0
/** Static atomic value initializer */
# define ATOMIC_INITIALIZER(n)	/* defined in implementation */
#endif

#include "arch/hexo/atomic.h"

#if __GNUC__ >= 4

static inline bool_t
atomic_compare_and_swap(volatile atomic_int_t *a, atomic_int_t old, atomic_int_t new)
{
# if defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP)
	return cpu_atomic_compare_and_swap(a, old, new);
# else
	return __sync_bool_compare_and_swap(a, old, new);
# endif
}

#endif // GNUC 4

#endif

