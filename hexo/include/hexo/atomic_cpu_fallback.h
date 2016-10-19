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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2016

*/

#ifndef CPU_ATOMIC_FALLBACK_H_
#define CPU_ATOMIC_FALLBACK_H_

#ifdef HAS_CPU_ATOMIC_COMPARE_AND_SWAP

/* Provide some default implementations based on compare and swap for
   other atomic functions unless a better implementation has been
   defined in processor support code. */

# ifndef HAS_CPU_ATOMIC_ADD
#  define HAS_CPU_ATOMIC_ADD
ALWAYS_INLINE atomic_int_t
__cpu_atomic_add(atomic_int_t *a, atomic_int_t value)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old + value));

  return old;
}
# endif

# ifndef HAS_CPU_ATOMIC_OR
#  define HAS_CPU_ATOMIC_OR
ALWAYS_INLINE atomic_int_t
__cpu_atomic_or(atomic_int_t *a, atomic_int_t value)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old | value));

  return old;
}
# endif

# ifndef HAS_CPU_ATOMIC_XOR
#  define HAS_CPU_ATOMIC_XOR
ALWAYS_INLINE atomic_int_t
__cpu_atomic_xor(atomic_int_t *a, atomic_int_t value)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old ^ value));

  return old;
}
# endif

# ifndef HAS_CPU_ATOMIC_AND
#  define HAS_CPU_ATOMIC_AND
ALWAYS_INLINE atomic_int_t
__cpu_atomic_and(atomic_int_t *a, atomic_int_t value)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old & value));

  return old;
}
# endif

# ifndef HAS_CPU_ATOMIC_SWAP
#  define HAS_CPU_ATOMIC_SWAP
ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
  atomic_int_t old;
  do {
    order_smp_read();
    old = *a;
  } while (!__cpu_atomic_compare_and_swap(a, old, old));

  return old;
}
# endif

#endif /* HAS_CPU_ATOMIC_COMPARE_AND_SWAP */

#ifdef HAS_CPU_ATOMIC_ADD

# ifndef HAS_CPU_ATOMIC_INC
#  define HAS_CPU_ATOMIC_INC
ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
  return __cpu_atomic_add(a, 1) + 1 != 0;
}
# endif

# ifndef HAS_CPU_ATOMIC_DEC
#  define HAS_CPU_ATOMIC_DEC
ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
  return __cpu_atomic_add(a, -1) - 1 != 0;
}
# endif

#endif /* HAS_CPU_ATOMIC_ADD */

#if defined(HAS_CPU_ATOMIC_OR) && !defined(HAS_CPU_ATOMIC_TESTSET)
# define HAS_CPU_ATOMIC_TESTSET
ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
  atomic_int_t mask = 1 << n;
  return (__cpu_atomic_or(a, mask) & mask) != 0;
}
# endif

#if defined(HAS_CPU_ATOMIC_AND) && !defined(HAS_CPU_ATOMIC_TESTCLR)
#  define HAS_CPU_ATOMIC_TESTCLR
ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
  atomic_int_t mask = 1 << n;
  return (__cpu_atomic_and(a, ~mask) & mask) != 0;
}
# endif

#if defined(HAS_CPU_ATOMIC_TESTSET) && !defined(HAS_CPU_ATOMIC_WAITSET)
# define HAS_CPU_ATOMIC_WAITSET
ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
  while (__cpu_atomic_bit_testset(a, n))
    ;
}
#endif

#if defined(HAS_CPU_ATOMIC_TESTCLR) && !defined(HAS_CPU_ATOMIC_WAITCLR)
# define HAS_CPU_ATOMIC_WAITCLR
ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
  while (!__cpu_atomic_bit_testclr(a, n))
    ;
}
#endif

#if defined(HAS_CPU_ATOMIC_TESTSET) && !defined(HAS_CPU_ATOMIC_SET)
# define HAS_CPU_ATOMIC_SET
ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
  __cpu_atomic_bit_testset(a, n);
}
#endif

#if defined(HAS_CPU_ATOMIC_TESTCLR) && !defined(HAS_CPU_ATOMIC_CLR)
#define HAS_CPU_ATOMIC_CLR
ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
  __cpu_atomic_bit_testclr(a, n);
}
#endif

#endif
