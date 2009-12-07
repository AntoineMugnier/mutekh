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

#ifdef CONFIG_ARCH_SOCLIB_RAMLOCK

/************************************************************** 
	USE RAMLOCKS
 **************************************************************/

#define ATOMIC_INITIALIZER(n)		{ .value = (n) }

#include <hexo/lock.h>

#ifdef CONFIG_SMP
extern lock_t __atomic_arch_lock;
#endif

struct arch_atomic_s
{
  volatile atomic_int_t	value;
};

static inline  void atomic_set(atomic_t *a, atomic_int_t value)
{
#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  a->value = value;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif
}

static inline  atomic_int_t atomic_get(atomic_t *a)
{
  atomic_int_t	res;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = a->value;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

static inline  bool_t atomic_inc(atomic_t *a)
{
  atomic_int_t	res;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = ++a->value;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

static inline  atomic_int_t atomic_add(atomic_t *a, atomic_int_t val)
{
  atomic_int_t	res;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = a->value;
  a->value += val;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

static inline  bool_t atomic_dec(atomic_t *a)
{
  atomic_int_t	res;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = --a->value;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

static inline  void atomic_bit_set(atomic_t *a, uint_fast8_t n)
{
#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  a->value |= 1 << n;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif
}

static inline  bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n)
{
  bool_t		res;
  const atomic_int_t	bit = 1 << n;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = a->value & bit ? 1 : 0;
  a->value |= bit;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

static inline  void atomic_bit_clr(atomic_t *a, uint_fast8_t n)
{
#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  a->value &= ~(1 << n);
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif
}

static inline  bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n)
{
  bool_t		res;
  const atomic_int_t	bit = 1 << n;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = a->value & bit ? 1 : 0;
  a->value &= ~bit;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

static inline  bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n)
{
  bool_t		res;
  const atomic_int_t	bit = 1 << n;

#ifdef CONFIG_SMP
  lock_spin_irq(&__atomic_arch_lock);
#endif
  res = a->value & bit ? 1 : 0;
#ifdef CONFIG_SMP
  lock_release_irq(&__atomic_arch_lock);
#endif

  return res;
}

#else

/************************************************************** 
	USE CPU ATOMIC OPS
 **************************************************************/

#define ATOMIC_INITIALIZER(n)		{ .value = (n) }

struct arch_atomic_s
{
  volatile atomic_int_t	value;
};

static inline void atomic_set(atomic_t *a, atomic_int_t value)
{
  a->value = value;
}

static inline atomic_int_t atomic_get(atomic_t *a)
{
  return a->value;
}

static inline bool_t atomic_inc(atomic_t *a)
{
  return cpu_atomic_inc(&a->value);
}

static inline atomic_int_t atomic_add(atomic_t *a, atomic_int_t val)
{
    return cpu_atomic_add(&a->value, val);
}

static inline bool_t atomic_dec(atomic_t *a)
{
  return cpu_atomic_dec(&a->value);
}

static inline void atomic_bit_set(atomic_t *a, uint_fast8_t n)
{
  cpu_atomic_bit_set(&a->value, n);
}

static inline bool_t atomic_bit_testset(atomic_t *a, uint_fast8_t n)
{
  return cpu_atomic_bit_testset(&a->value, n);
}

static inline void atomic_bit_clr(atomic_t *a, uint_fast8_t n)
{
  cpu_atomic_bit_clr(&a->value, n);
}

static inline bool_t atomic_bit_testclr(atomic_t *a, uint_fast8_t n)
{
  return cpu_atomic_bit_testclr(&a->value, n);
}

static inline bool_t atomic_bit_test(atomic_t *a, uint_fast8_t n)
{
  return ((1 << n) & a->value) != 0;
}

#endif

#endif

