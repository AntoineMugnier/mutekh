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
#define ATOMIC_FAST8_INITIALIZER(n)	{ .value = (n) }
#define ATOMIC_FAST16_INITIALIZER(n)	{ .value = (n) }


#define __ATOMIC_CPU_PROTO(attr, type)                          \
attr type##_int_t                                               \
__cpu_##type##_add(type##_int_t *a, type##_int_t value);        \
                                                                \
attr type##_int_t                                               \
__cpu_##type##_or(type##_int_t *a, type##_int_t value);         \
                                                                \
attr type##_int_t                                               \
__cpu_##type##_xor(type##_int_t *a, type##_int_t value);        \
                                                                \
attr type##_int_t                                               \
__cpu_##type##_and(type##_int_t *a, type##_int_t value);        \
                                                                \
attr type##_int_t                                               \
__cpu_##type##_swap(type##_int_t *a, type##_int_t value);       \
                                                                \
attr bool_t                                                     \
__cpu_##type##_inc(type##_int_t *a);                            \
                                                                \
attr bool_t                                                     \
__cpu_##type##_dec(type##_int_t *a);                            \
                                                                \
attr bool_t                                                     \
__cpu_##type##_bit_testset(type##_int_t *a, uint_fast8_t n);    \
                                                                \
attr void                                                       \
__cpu_##type##_bit_waitset(type##_int_t *a, uint_fast8_t n);    \
                                                                \
attr bool_t                                                     \
__cpu_##type##_bit_testclr(type##_int_t *a, uint_fast8_t n);    \
                                                                \
attr void                                                       \
__cpu_##type##_bit_waitclr(type##_int_t *a, uint_fast8_t n);    \
                                                                \
attr void                                                       \
__cpu_##type##_bit_set(type##_int_t *a, uint_fast8_t n);        \
                                                                \
attr void                                                       \
__cpu_##type##_bit_clr(type##_int_t *a, uint_fast8_t n);        \
                                                                \
attr bool_t                                                     \
__cpu_##type##_compare_and_swap(type##_int_t *a,                \
                    type##_int_t old, type##_int_t future);

#define __ATOMIC_CPUBASED(type)                                         \
struct arch_##type##_s                                                  \
{                                                                       \
  type##_int_t	value;                                                  \
};                                                                      \
                                                                        \
ALWAYS_INLINE void type##_set(type##_t *a, type##_int_t value)          \
{                                                                       \
  a->value = value;                                                     \
  order_smp_write();                                                    \
}                                                                       \
                                                                        \
ALWAYS_INLINE type##_int_t type##_get(type##_t *a)                      \
{                                                                       \
  order_smp_read();                                                     \
  return a->value;                                                      \
}                                                                       \
                                                                        \
ALWAYS_INLINE type##_int_t type##_add(type##_t *a, type##_int_t value)  \
{                                                                       \
  return __cpu_##type##_add(&a->value, value);                          \
}                                                                       \
                                                                        \
ALWAYS_INLINE type##_int_t type##_or(type##_t *a, type##_int_t value)   \
{                                                                       \
  return __cpu_##type##_or(&a->value, value);                           \
}                                                                       \
                                                                        \
ALWAYS_INLINE type##_int_t type##_xor(type##_t *a, type##_int_t value)  \
{                                                                       \
  return __cpu_##type##_xor(&a->value, value);                          \
}                                                                       \
                                                                        \
ALWAYS_INLINE type##_int_t type##_and(type##_t *a, type##_int_t value)  \
{                                                                       \
  return __cpu_##type##_and(&a->value, value);                          \
}                                                                       \
                                                                        \
ALWAYS_INLINE type##_int_t type##_swap(type##_t *a, type##_int_t value) \
{                                                                       \
  return __cpu_##type##_swap(&a->value, value);                         \
}                                                                       \
                                                                        \
ALWAYS_INLINE bool_t type##_inc(type##_t *a)                            \
{                                                                       \
  return __cpu_##type##_inc(&a->value);                                 \
}                                                                       \
                                                                        \
ALWAYS_INLINE bool_t type##_dec(type##_t *a)                            \
{                                                                       \
  return __cpu_##type##_dec(&a->value);                                 \
}                                                                       \
                                                                        \
ALWAYS_INLINE void type##_bit_set(type##_t *a, uint_fast8_t n)          \
{                                                                       \
  __cpu_##type##_bit_set(&a->value, n);                                 \
}                                                                       \
                                                                        \
ALWAYS_INLINE bool_t type##_bit_testset(type##_t *a, uint_fast8_t n)    \
{                                                                       \
  return __cpu_##type##_bit_testset(&a->value, n);                      \
}                                                                       \
                                                                        \
ALWAYS_INLINE void type##_bit_clr(type##_t *a, uint_fast8_t n)          \
{                                                                       \
  __cpu_##type##_bit_clr(&a->value, n);                                 \
}                                                                       \
                                                                        \
ALWAYS_INLINE bool_t type##_bit_testclr(type##_t *a, uint_fast8_t n)    \
{                                                                       \
  return __cpu_##type##_bit_testclr(&a->value, n);                      \
}                                                                       \
                                                                        \
ALWAYS_INLINE bool_t type##_bit_test(type##_t *a, uint_fast8_t n)       \
{                                                                       \
  return (type##_get(a) >> n) & 1;                                      \
}                                                                       \
                                                                        \
ALWAYS_INLINE bool_t type##_compare_and_swap(type##_t *a,               \
                       type##_int_t old, type##_int_t future)           \
{                                                                       \
  return __cpu_##type##_compare_and_swap(&a->value, old, future);       \
}

__ATOMIC_CPUBASED(atomic);
__ATOMIC_CPUBASED(atomic_fast8);
__ATOMIC_CPUBASED(atomic_fast16);

#define ARCH_ATOMIC_H_

#endif

