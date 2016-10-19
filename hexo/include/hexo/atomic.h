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

/** @this atomically add a signed value to an integer value in memory.
    @return the previous value. */
ALWAYS_INLINE atomic_int_t atomic_add(atomic_t *a, atomic_int_t value);

/** @this atomically add a signed value to an integer value in memory.
    @return the previous value. */
ALWAYS_INLINE atomic_int_t atomic_or(atomic_t *a, atomic_int_t value);

/** @this atomically add a signed value to an integer value in memory.
    @return the previous value. */
ALWAYS_INLINE atomic_int_t atomic_xor(atomic_t *a, atomic_int_t value);

/** @this atomically add a signed value to an integer value in memory.
    @return the previous value. */
ALWAYS_INLINE atomic_int_t atomic_and(atomic_t *a, atomic_int_t value);

/** @this atomically swap a value in memory.
    @return the previous value. */
ALWAYS_INLINE atomic_int_t atomic_swap(atomic_t *a, atomic_int_t value);

/** @this atomically increments integer value in memory.
    @return 0 if the new value is 0. */
ALWAYS_INLINE bool_t atomic_inc(atomic_t *a);

/** @this atomically decrements integer value in memory
    @return 0 if the new value is 0. */
ALWAYS_INLINE bool_t atomic_dec(atomic_t *a);

/** @this atomically sets bit in integer value in memory */
ALWAYS_INLINE void atomic_bit_set(atomic_t *a, uint_fast8_t n);

/** @this atomically tests and sets bit in integer value in memory
   @return 0 if bit was cleared before. */
ALWAYS_INLINE bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n);

/** @this atomically clears bit in integer value in memory */
ALWAYS_INLINE void atomic_bit_clr(atomic_t *a, uint_fast8_t n);

/** @this atomically tests and clears bit in integer value in memory
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

/* implement atomic operations defined above using either cpu atomic
   operations or a platform specific mechanism */
#include <arch/hexo/atomic.h>

#endif

