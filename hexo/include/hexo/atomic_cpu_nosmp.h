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

#ifndef CPU_ATOMIC_NA_H_
#define CPU_ATOMIC_NA_H_

#include <hexo/interrupt.h>

/*
  Atomic operations are performed by disabling interrupts.

  We dont need to add extra memory barrier for pseudo atomic
  operations here because interrupt enable and disable acts as a
  compiler memory barrier.
*/

#define CPU_ATOMIC_H_

#ifndef HAS_CPU_ATOMIC_ADD
# ifdef CONFIG_ARCH_SMP
#  error Missing add atomic operations for SMP build
# endif
ALWAYS_INLINE atomic_int_t
__cpu_atomic_add(atomic_int_t *a, atomic_int_t value)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  res = *a;
  *a += value;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_OR
# ifdef CONFIG_ARCH_SMP
#  error Missing or atomic operations for SMP build
# endif
ALWAYS_INLINE atomic_int_t
__cpu_atomic_or(atomic_int_t *a, atomic_int_t value)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  res = *a;
  *a |= value;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_XOR
# ifdef CONFIG_ARCH_SMP
#  error Missing xor atomic operations for SMP build
# endif
ALWAYS_INLINE atomic_int_t
__cpu_atomic_xor(atomic_int_t *a, atomic_int_t value)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  res = *a;
  *a ^= value;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_AND
# ifdef CONFIG_ARCH_SMP
#  error Missing and atomic operations for SMP build
# endif
ALWAYS_INLINE atomic_int_t
__cpu_atomic_and(atomic_int_t *a, atomic_int_t value)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  res = *a;
  *a &= value;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_SWAP
# ifdef CONFIG_ARCH_SMP
#  error Missing swap atomic operations for SMP build
# endif
ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  res = *a;
  *a = value;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_INC
# ifdef CONFIG_ARCH_SMP
#  error Missing inc atomic operations for SMP build
# endif
ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  (*a)++;
  res = *a ? 1 : 0;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_DEC
# ifdef CONFIG_ARCH_SMP
#  error Missing dec atomic operations for SMP build
# endif
ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
  bool_t res;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  (*a)--;
  res = *a ? 1 : 0;
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_TESTSET
# ifdef CONFIG_ARCH_SMP
#  error Missing bit test and set atomic operations for SMP build
# endif
ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
  bool_t res;
  atomic_int_t	old;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  old = *a;
  *a |= (1 << n);
  res = !!(old & (1 << n));
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_WAITSET
# ifdef CONFIG_ARCH_SMP
#  error Missing bit wait set atomic operations for SMP build
# endif
ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
  while (!__cpu_atomic_bit_testset(a, n))
    ;
}
#endif

#ifndef HAS_CPU_ATOMIC_TESTCLR
# ifdef CONFIG_ARCH_SMP
#  error Missing bit test and clear atomic operations for SMP build
# endif
ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
  bool_t res;
  atomic_int_t	old;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  old = *a;
  *a &= ~(1 << n);
  res = !!(old & (1 << n));
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}
#endif

#ifndef HAS_CPU_ATOMIC_WAITCLR
# ifdef CONFIG_ARCH_SMP
#  error Missing bit wait clear atomic operations for SMP build
# endif
ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
  while (__cpu_atomic_bit_testclr(a, n))
    ;
}
#endif

#ifndef HAS_CPU_ATOMIC_SET
# ifdef CONFIG_ARCH_SMP
#  error Missing bit set atomic operations for SMP build
# endif
ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  *a |= (1 << n);
  CPU_INTERRUPT_RESTORESTATE;
}
#endif

#ifndef HAS_CPU_ATOMIC_CLR
# ifdef CONFIG_ARCH_SMP
#  error Missing bit clear atomic operations for SMP build
# endif
ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  *a &= ~(1 << n);
  CPU_INTERRUPT_RESTORESTATE;
}
#endif

#ifndef HAS_CPU_ATOMIC_COMPARE_AND_SWAP
# ifdef CONFIG_ARCH_SMP
#  error Missing compare and swap atomic operations for SMP build
# endif
ALWAYS_INLINE bool_t
__cpu_atomic_compare_and_swap(atomic_int_t *a, atomic_int_t old, atomic_int_t future)
{
  bool_t res = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  if (*a == old)
    {
      *a = future;
      res = 1;
    }
  CPU_INTERRUPT_RESTORESTATE;  

  return res;
}
#endif

#endif

