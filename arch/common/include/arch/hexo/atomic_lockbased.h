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
#include <hexo/lock.h>

#define ATOMIC_INITIALIZER(n)		{ .value = (n) }
#define ATOMIC_FAST8_INITIALIZER(n)	{ .value = (n) }
#define ATOMIC_FAST16_INITIALIZER(n)	{ .value = (n) }

extern lock_t __atomic_arch_lock;

typedef int32_t atomic_int_t;
typedef int16_t atomic_fast16_int_t;
typedef int8_t  atomic_fast8_int_t;

#define __ATOMIC_LOCKBASED(type)                                        \
                                                                        \
struct arch_##type##_s                                                  \
{                                                                       \
  type##_int_t	value;                                                  \
};                                                                      \
                                                                        \
inline void type##_set(type##_t *a, type##_int_t value)                 \
{                                                                       \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  a->value = value;                                                     \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
}                                                                       \
                                                                        \
inline type##_int_t type##_get(type##_t *a)                             \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value;                                                       \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t type##_add(type##_t *a, type##_int_t value)         \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value;                                                       \
  a->value += value;                                                    \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t type##_or(type##_t *a, type##_int_t value)          \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value;                                                       \
  a->value |= value;                                                    \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t type##_xor(type##_t *a, type##_int_t value)         \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value;                                                       \
  a->value ^= value;                                                    \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t type##_and(type##_t *a, type##_int_t value)         \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value;                                                       \
  a->value &= value;                                                    \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t type##_swap(type##_t *a, type##_int_t value)        \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value;                                                       \
  a->value = value;                                                     \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline bool_t type##_inc(type##_t *a)                                   \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = ++a->value;                                                     \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline bool_t type##_dec(type##_t *a)                                   \
{                                                                       \
  type##_int_t	res;                                                    \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = --a->value;                                                     \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline void type##_bit_set(type##_t *a, uint_fast8_t n)                 \
{                                                                       \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  a->value |= 1 << n;                                                   \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
}                                                                       \
                                                                        \
inline bool_t type##_bit_testset(type##_t *a, uint_fast8_t n)           \
{                                                                       \
  bool_t		res;                                            \
  const type##_int_t	bit = 1 << n;                                   \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value & bit ? 1 : 0;                                         \
  a->value |= bit;                                                      \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline void type##_bit_clr(type##_t *a, uint_fast8_t n)                 \
{                                                                       \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  a->value &= ~(1 << n);                                                \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
}                                                                       \
                                                                        \
inline bool_t type##_bit_testclr(type##_t *a, uint_fast8_t n)           \
{                                                                       \
  bool_t		res;                                            \
  const type##_int_t	bit = 1 << n;                                   \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value & bit ? 1 : 0;                                         \
  a->value &= ~bit;                                                     \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline bool_t type##_bit_test(type##_t *a, uint_fast8_t n)              \
{                                                                       \
  bool_t		res;                                            \
  const type##_int_t	bit = 1 << n;                                   \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  res = a->value & bit ? 1 : 0;                                         \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}                                                                       \
                                                                        \
inline bool_t type##_compare_and_swap(type##_t *a,                      \
                       type##_int_t old, type##_int_t future)           \
{                                                                       \
  bool_t res = 0;                                                       \
                                                                        \
  LOCK_SPIN_IRQ(&__atomic_arch_lock);                                   \
  if (a->value == old)                                                  \
    {                                                                   \
      a->value = future;                                                \
      res = 1;                                                          \
    }                                                                   \
  LOCK_RELEASE_IRQ(&__atomic_arch_lock);                                \
                                                                        \
  return res;                                                           \
}

__ATOMIC_LOCKBASED(atomic);
__ATOMIC_LOCKBASED(atomic_fast8);
__ATOMIC_LOCKBASED(atomic_fast16);

#define ARCH_ATOMIC_H_

#endif

