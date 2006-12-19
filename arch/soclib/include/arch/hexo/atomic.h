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

#if !defined(ATOMIC_H_) || defined(ARCH_ATOMIC_H_)
#error This file can not be included directly
#else

#define ARCH_ATOMIC_H_

#define ATOMIC_INITIALIZER(n)		{ .value = (n) }

#include <hexo/lock.h>

extern lock_t __atomic_arch_lock;

struct arch_atomic_s
{
  volatile atomic_int_t	value;
};

static inline  void atomic_set(atomic_t *a, atomic_int_t value)
{
  lock_spin_irq(&__atomic_arch_lock);
  a->value = value;
  lock_release_irq(&__atomic_arch_lock);
}

static inline  atomic_int_t atomic_get(atomic_t *a)
{
  atomic_int_t	res;

  lock_spin_irq(&__atomic_arch_lock);
  res = a->value;
  lock_release_irq(&__atomic_arch_lock);

  return res;
}

static inline  bool_t atomic_inc(atomic_t *a)
{
  atomic_int_t	res;

  lock_spin_irq(&__atomic_arch_lock);
  res = ++a->value;
  lock_release_irq(&__atomic_arch_lock);

  return res;
}

static inline  bool_t atomic_dec(atomic_t *a)
{
  atomic_int_t	res;

  lock_spin_irq(&__atomic_arch_lock);
  res = --a->value;
  lock_release_irq(&__atomic_arch_lock);

  return res;
}

static inline  void atomic_bit_set(atomic_t *a, uint_fast8_t n)
{
  lock_spin_irq(&__atomic_arch_lock);
  a->value |= 1 << n;
  lock_release_irq(&__atomic_arch_lock);
}

static inline  bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n)
{
  bool_t		res;
  const atomic_int_t	bit = 1 << n;

  lock_spin_irq(&__atomic_arch_lock);
  res = a->value & bit ? 1 : 0;
  a->value |= bit;
  lock_release_irq(&__atomic_arch_lock);

  return res;
}

static inline  void atomic_bit_clr(atomic_t *a, uint_fast8_t n)
{
  lock_spin_irq(&__atomic_arch_lock);
  a->value &= ~(1 << n);
  lock_release_irq(&__atomic_arch_lock);
}

static inline  bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n)
{
  bool_t		res;
  const atomic_int_t	bit = 1 << n;

  lock_spin_irq(&__atomic_arch_lock);
  res = a->value & bit ? 1 : 0;
  a->value &= ~bit;
  lock_release_irq(&__atomic_arch_lock);

  return res;
}

static inline  bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n)
{
  bool_t		res;
  const atomic_int_t	bit = 1 << n;

  lock_spin_irq(&__atomic_arch_lock);
  res = a->value & bit ? 1 : 0;
  lock_release_irq(&__atomic_arch_lock);

  return res;
}

#endif

