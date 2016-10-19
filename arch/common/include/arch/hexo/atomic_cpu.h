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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#if !defined(ATOMIC_H_) || defined(ARCH_ATOMIC_H_)
#error This file can not be included directly
#else

#include <hexo/ordering.h>

/* implement underlying cpu atomic operations. */
#include <cpu/hexo/atomic.h>

/* implement missing underlying cpu atomic operation from available operations. */
#include <hexo/atomic_cpu_fallback.h>

/* implement missing underlying cpu atomic operation by disabling
   interrupts, this only work on single processor builds. */
#include <hexo/atomic_cpu_nosmp.h>

#define ATOMIC_INITIALIZER(n)		{ .value = (n) }

struct arch_atomic_s
{
  atomic_int_t	value;
};

ALWAYS_INLINE void atomic_set(atomic_t *a, atomic_int_t value)
{
  a->value = value;
  order_smp_write();
}

ALWAYS_INLINE atomic_int_t atomic_get(atomic_t *a)
{
  order_smp_read();
  return a->value;
}

ALWAYS_INLINE atomic_int_t atomic_add(atomic_t *a, atomic_int_t value)
{
  return __cpu_atomic_add(&a->value, value);
}

ALWAYS_INLINE atomic_int_t atomic_or(atomic_t *a, atomic_int_t value)
{
  return __cpu_atomic_or(&a->value, value);
}

ALWAYS_INLINE atomic_int_t atomic_xor(atomic_t *a, atomic_int_t value)
{
  return __cpu_atomic_xor(&a->value, value);
}

ALWAYS_INLINE atomic_int_t atomic_and(atomic_t *a, atomic_int_t value)
{
  return __cpu_atomic_and(&a->value, value);
}

ALWAYS_INLINE atomic_int_t atomic_swap(atomic_t *a, atomic_int_t value)
{
  return __cpu_atomic_swap(&a->value, value);
}

ALWAYS_INLINE bool_t atomic_inc(atomic_t *a)
{
  return __cpu_atomic_inc(&a->value);
}

ALWAYS_INLINE bool_t atomic_dec(atomic_t *a)
{
  return __cpu_atomic_dec(&a->value);
}

ALWAYS_INLINE void atomic_bit_set(atomic_t *a, uint_fast8_t n)
{
  __cpu_atomic_bit_set(&a->value, n);
}

ALWAYS_INLINE bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n)
{
  return __cpu_atomic_bit_testset(&a->value, n);
}

ALWAYS_INLINE void atomic_bit_clr(atomic_t *a, uint_fast8_t n)
{
  __cpu_atomic_bit_clr(&a->value, n);
}

ALWAYS_INLINE bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n)
{
  return __cpu_atomic_bit_testclr(&a->value, n);
}

ALWAYS_INLINE bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n)
{
  return (atomic_get(a) >> n) & 1;
}

ALWAYS_INLINE bool_t atomic_compare_and_swap(atomic_t *a, atomic_int_t old, atomic_int_t future)
{
  return __cpu_atomic_compare_and_swap(&a->value, old, future);
}

#define ARCH_ATOMIC_H_

#endif

