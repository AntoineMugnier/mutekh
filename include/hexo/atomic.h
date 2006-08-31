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




/*
  CPU atomic functions use standard integer values of type
  atomic_int_t and provide bus locked access.

  Atomicity is not garanted if system architecture does not handle
  atomic bus access. PLEASE CONSIDER USING arch atomic_t AND ATOMIC
  FUNCTION INSTEAD in general case.

  Some CPU may have partial or missing atomic access capabilities,
  please check for HAS_CPU_ATOMIC_* macro defined in
  "cpu/hexo/atomic.h".
 */

/**
   atomicaly increment value in memory.
   @return 0 if new atomic value is 0.
*/
static bool_t cpu_atomic_inc(volatile atomic_int_t *a);

/**
   atomicaly decrement value in memory
   @return 0 if new atomic value is 0.
*/
static bool_t cpu_atomic_dec(volatile atomic_int_t *a);

/** set bit in memory */
static void cpu_atomic_bit_set(volatile atomic_int_t *a, uint_fast8_t n);

/**
   atomicaly test and set bit in memory
   @return 0 if bit was cleared before.
*/
static bool_t cpu_atomic_bit_testset(volatile atomic_int_t *a, uint_fast8_t n);

/** atomicaly wait (spin) for bit clear and set it */
static void cpu_atomic_bit_waitset(volatile atomic_int_t *a, uint_fast8_t n);

/** clear bit in memory */
static void cpu_atomic_bit_clr(volatile atomic_int_t *a, uint_fast8_t n);

/**
   atomicaly test and clear bit in memory
   @return 0 if bit was cleared before.
 */
static bool_t cpu_atomic_bit_testclr(volatile atomic_int_t *a, uint_fast8_t n);

/** atomicaly wait for bit set and clear it */
static void cpu_atomic_bit_waitclr(volatile atomic_int_t *a, uint_fast8_t n);

/**
   test bit in memory
   @return 0 if bit is cleared.
*/
static bool_t cpu_atomic_bit_test(volatile atomic_int_t *a, uint_fast8_t n);

#include "cpu/hexo/atomic.h"




/*
  Arch atomic functions use architecture specific structures of type
  atomic_t and provide locked access on atomic integer values. It may
  use cpu atomic operations or additional spin lock depending on
  system archicture hardware capabilities. Use it for general purpose
  atomic values access.
 */

typedef struct arch_atomic_s atomic_t;

/** set atomic integer value */
static void atomic_set(atomic_t *a, atomic_int_t value);

/**
   get atomic integer value
   @return integer value
*/
static atomic_int_t atomic_get(atomic_t *a);

/**
   atomicaly increment integer value.
   @return 0 if new atomic value is 0.
*/
static bool_t atomic_inc(atomic_t *a);

/**
   atomicaly decrement integer value
   @return 0 if new atomic value is 0.
*/
static bool_t atomic_dec(atomic_t *a);

/** atomicaly set bit in intger value */
static void atomic_bit_set(atomic_t *a, uint_fast8_t n);

/**
   atomicaly test and set bit in integer value
   @return 0 if bit was cleared before.
*/
static bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n);

/** atomicaly clear bit in integer value */
static void atomic_bit_clr(atomic_t *a, uint_fast8_t n);

/**
   atomicaly test and clear bit in integer value
   @return 0 if bit was cleared before.
 */
static bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n);

/**
   test bit in integer value
   @return 0 if bit is cleared.
*/
static bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n);

/**
   static atomic value initializer

#define ATOMIC_INITIALIZER(n)
*/

#include "arch/hexo/atomic.h"

#endif

