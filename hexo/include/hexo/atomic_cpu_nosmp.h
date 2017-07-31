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

#define __CPU_ATOMIC_NOSMP_BINOP(name, type, op)        \
inline type##_int_t                              \
__cpu_##type##_##name(type##_int_t *a, type##_int_t value)\
{                                                       \
  bool_t res;                                           \
                                                        \
  CPU_INTERRUPT_SAVESTATE_DISABLE;                      \
  res = *a;                                             \
  *a = *a op value;                                     \
  CPU_INTERRUPT_RESTORESTATE;                           \
                                                        \
  return res;                                           \
}

#if !defined(HAS_CPU_ATOMIC_ADD)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(add, atomic, +);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST8_ADD)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(add, atomic_fast8, +);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST16_ADD)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(add, atomic_fast16, +);
#endif


#if !defined(HAS_CPU_ATOMIC_OR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(or, atomic, |);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST8_OR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(or, atomic_fast8, |);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST16_OR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(or, atomic_fast16, |);
#endif


#if !defined(HAS_CPU_ATOMIC_XOR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(xor, atomic, ^);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST8_XOR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(xor, atomic_fast8, ^);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST16_XOR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(xor, atomic_fast16, ^);
#endif


#if !defined(HAS_CPU_ATOMIC_AND)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(and, atomic, &);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST8_AND)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(and, atomic_fast8, &);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST16_AND)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BINOP(and, atomic_fast16, &);
#endif


#define __CPU_ATOMIC_NOSMP_SWAP(type)                           \
inline type##_int_t                                      \
__cpu_##type##_swap(type##_int_t *a, type##_int_t value)        \
{                                                               \
  bool_t res;                                                   \
                                                                \
  CPU_INTERRUPT_SAVESTATE_DISABLE;                              \
  res = *a;                                                     \
  *a = value;                                                   \
  CPU_INTERRUPT_RESTORESTATE;                                   \
                                                                \
  return res;                                                   \
}

#if !defined(HAS_CPU_ATOMIC_SWAP)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_SWAP(atomic);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST8_SWAP)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_SWAP(atomic_fast8);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST16_SWAP)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_SWAP(atomic_fast16);
#endif


#define __CPU_ATOMIC_NOSMP_INC(name, type, v)        \
inline bool_t                                    \
__cpu_##type##_##name(type##_int_t *a)                  \
{                                                       \
  bool_t res;                                           \
                                                        \
  CPU_INTERRUPT_SAVESTATE_DISABLE;                      \
  (*a) += v;                                            \
  res = !!*a;                                           \
  CPU_INTERRUPT_RESTORESTATE;                           \
                                                        \
  return res;                                           \
}

# if !defined(HAS_CPU_ATOMIC_INC)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_INC(inc, atomic, +1);
# endif

# if !defined(HAS_CPU_ATOMIC_FAST8_INC)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_INC(inc, atomic_fast8, +1);
# endif

# if !defined(HAS_CPU_ATOMIC_FAST16_INC)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_INC(inc, atomic_fast16, +1);
# endif


# if !defined(HAS_CPU_ATOMIC_DEC)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_INC(dec, atomic, -1);
# endif

# if !defined(HAS_CPU_ATOMIC_FAST8_DEC)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_INC(dec, atomic_fast8, -1);
# endif

# if !defined(HAS_CPU_ATOMIC_FAST16_DEC)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_INC(dec, atomic_fast16, -1);
# endif


#define __CPU_ATOMIC_NOSMP_BIT_TEST(name, type, expr)        \
inline bool_t                                            \
__cpu_##type##_bit_##name(type##_int_t *a, uint_fast8_t n)      \
{                                                               \
  bool_t res;                                                   \
  type##_int_t	old, mask = 1 << n;                             \
                                                                \
  CPU_INTERRUPT_SAVESTATE_DISABLE;                              \
  old = *a;                                                     \
  expr;                                                         \
  res = !!(old & mask);                                         \
  CPU_INTERRUPT_RESTORESTATE;                                   \
                                                                \
  return res;                                                   \
}

#if !defined(HAS_CPU_ATOMIC_TESTSET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_TEST(testset, atomic, *a |= mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST8_TESTSET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_TEST(testset, atomic_fast8, *a |= mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST16_TESTSET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_TEST(testset, atomic_fast16, *a |= mask);
# endif


#if !defined(HAS_CPU_ATOMIC_TESTCLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_TEST(testclr, atomic, *a &= ~mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST8_TESTCLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_TEST(testclr, atomic_fast8, *a &= ~mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST16_TESTCLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_TEST(testclr, atomic_fast16, *a &= ~mask);
# endif

#define __CPU_ATOMIC_NOSMP_BIT_WAIT(name, type)              \
inline void                                              \
__cpu_##type##_bit_wait##name(type##_int_t *a, uint_fast8_t n)  \
{                                                               \
  while (!__cpu_##type##_bit_test##name(a, n))                  \
    ;                                                           \
}

#if !defined(HAS_CPU_ATOMIC_WAITSET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_WAIT(set, atomic);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST8_WAITSET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_WAIT(set, atomic_fast8);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST16_WAITSET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_WAIT(set, atomic_fast16);
# endif

#if !defined(HAS_CPU_ATOMIC_WAITCLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_WAIT(clr, atomic);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST8_WAITCLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_WAIT(clr, atomic_fast8);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST16_WAITCLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_WAIT(clr, atomic_fast16);
# endif


#define __CPU_ATOMIC_NOSMP_BIT_UPDATE(name, type, expr)         \
inline void                                              \
__cpu_##type##_bit_##name(type##_int_t *a, uint_fast8_t n)      \
{                                                               \
  type##_int_t	mask = 1 << n;                                  \
  CPU_INTERRUPT_SAVESTATE_DISABLE;                              \
  expr;                                                         \
  CPU_INTERRUPT_RESTORESTATE;                                   \
}

#if !defined(HAS_CPU_ATOMIC_SET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_UPDATE(set, atomic, *a |= mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST8_SET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_UPDATE(set, atomic_fast8, *a |= mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST16_SET)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_UPDATE(set, atomic_fast16, *a |= mask);
# endif


#if !defined(HAS_CPU_ATOMIC_CLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_UPDATE(clr, atomic, *a &= ~mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST8_CLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_UPDATE(clr, atomic_fast8, *a &= ~mask);
# endif

#if !defined(HAS_CPU_ATOMIC_FAST16_CLR)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_UPDATE(clr, atomic_fast16, *a &= ~mask);
# endif

#define __CPU_ATOMIC_NOSMP_BIT_COMPARE_AND_SWAP(type)                   \
inline bool_t                                                    \
__cpu_##type##_compare_and_swap(type##_int_t *a, type##_int_t old, type##_int_t future) \
{                                                                       \
  bool_t res = 0;                                                       \
                                                                        \
  CPU_INTERRUPT_SAVESTATE_DISABLE;                                      \
  if (*a == old)                                                        \
    {                                                                   \
      *a = future;                                                      \
      res = 1;                                                          \
    }                                                                   \
  CPU_INTERRUPT_RESTORESTATE;                                           \
                                                                        \
  return res;                                                           \
}

#if !defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_COMPARE_AND_SWAP(atomic);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_COMPARE_AND_SWAP(atomic_fast8);
#endif

#if !defined(HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP)
#define __CPU_ATOMIC_NOSMP
__CPU_ATOMIC_NOSMP_BIT_COMPARE_AND_SWAP(atomic_fast16);
#endif

#if defined(__CPU_ATOMIC_NOSMP) && defined(CONFIG_ARCH_SMP)
# define __CPU_ATOMIC_NOSMP warn previous def
# error Missing cpu atomic operations for SMP build
#endif

#endif

