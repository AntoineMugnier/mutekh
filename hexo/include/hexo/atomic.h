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
typedef struct arch_atomic_fast8_s atomic_fast8_t;
typedef struct arch_atomic_fast16_s atomic_fast16_t;

#define __ATOMIC_PROTO(attr, type)                                      \
                                                                        \
/** @this sets atomic integer value in memory */                        \
attr void type##_set(type##_t *a, type##_int_t value);         \
                                                                        \
/** @this gets atomic integer value in memory */                        \
attr type##_int_t type##_get(type##_t *a);                     \
                                                                        \
/** @this atomically add a signed value to an integer value in memory.  \
    @return the previous value. */                                      \
attr type##_int_t type##_add(type##_t *a, type##_int_t value); \
                                                                        \
/** @this atomically add a signed value to an integer value in memory.  \
    @return the previous value. */                                      \
attr type##_int_t type##_or(type##_t *a, type##_int_t value);  \
                                                                        \
/** @this atomically add a signed value to an integer value in memory.  \
    @return the previous value. */                                      \
attr type##_int_t type##_xor(type##_t *a, type##_int_t value); \
                                                                        \
/** @this atomically add a signed value to an integer value in memory.  \
    @return the previous value. */                                      \
attr type##_int_t type##_and(type##_t *a, type##_int_t value); \
                                                                        \
/** @this atomically swap a value in memory.                            \
    @return the previous value. */                                      \
attr type##_int_t type##_swap(type##_t *a, type##_int_t value); \
                                                                        \
/** @this atomically increments integer value in memory.                \
    @return 0 if the new value is 0. */                                 \
attr bool_t type##_inc(type##_t *a);                           \
                                                                        \
/** @this atomically decrements integer value in memory                 \
    @return 0 if the new value is 0. */                                 \
attr bool_t type##_dec(type##_t *a);                           \
                                                                        \
/** @this atomically sets bit in integer value in memory */             \
attr void type##_bit_set(type##_t *a, uint_fast8_t n);         \
                                                                        \
/** @this atomically tests and sets bit in integer value in memory      \
   @return 0 if bit was cleared before. */                              \
attr bool_t type##_bit_testset(type##_t *a, uint_fast8_t n);   \
                                                                        \
/** @this atomically clears bit in integer value in memory */           \
attr void type##_bit_clr(type##_t *a, uint_fast8_t n);         \
                                                                        \
/** @this atomically tests and clears bit in integer value in memory    \
   @return 0 if bit was cleared before. */                              \
attr bool_t type##_bit_testclr(type##_t *a, uint_fast8_t n);   \
                                                                        \
/** @this tests bit in integer value in memory                          \
   @return 0 if bit is cleared. */                                      \
attr bool_t type##_bit_test(type##_t *a, uint_fast8_t n);      \
                                                                        \
/** @this compares memory to old and replace with future if they are    \
   the same. @return true if exchanged */                               \
attr bool_t type##_compare_and_swap(type##_t *a,               \
                       type##_int_t old, type##_int_t future);

#if 0
/** inline atomic value initializer */
# define ATOMIC_INITIALIZER(n)	/* defined in implementation */
# define ATOMIC_FAST8_INITIALIZER(n)	/* defined in implementation */
# define ATOMIC_FAST16_INITIALIZER(n)	/* defined in implementation */
#endif

/* implement atomic operations defined above using either cpu atomic
   operations or a platform specific mechanism */
#include <arch/hexo/atomic.h>

__ATOMIC_PROTO(inline, atomic);
__ATOMIC_PROTO(inline, atomic_fast8);
__ATOMIC_PROTO(inline, atomic_fast16);

#endif

