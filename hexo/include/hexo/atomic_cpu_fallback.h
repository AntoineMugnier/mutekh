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

/* Provide some default implementations based on compare and swap for
   other atomic functions unless a better implementation has been
   defined in processor support code. */

#define __CPU_ATOMIC_FALLBACK_BINOP(name, type, op)                     \
inline type##_int_t                                              \
__cpu_##type##_##name(type##_int_t *a, type##_int_t value)              \
{                                                                       \
  type##_int_t old;                                                     \
  do {                                                                  \
    order_smp_read();                                                   \
    old = *a;                                                           \
  } while (!__cpu_##type##_compare_and_swap(a, old, old op value));     \
                                                                        \
  return old;                                                           \
}

#if defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_ADD)
# define HAS_CPU_ATOMIC_ADD
__CPU_ATOMIC_FALLBACK_BINOP(add, atomic, +);
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST8_ADD)
# define HAS_CPU_ATOMIC_FAST8_ADD
__CPU_ATOMIC_FALLBACK_BINOP(add, atomic_fast8, +);
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST16_ADD)
# define HAS_CPU_ATOMIC_FAST16_ADD
__CPU_ATOMIC_FALLBACK_BINOP(add, atomic_fast16, +);
#endif


#if defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_OR)
# define HAS_CPU_ATOMIC_OR
__CPU_ATOMIC_FALLBACK_BINOP(or, atomic, |);
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST8_OR)
# define HAS_CPU_ATOMIC_FAST8_OR
__CPU_ATOMIC_FALLBACK_BINOP(or, atomic_fast8, |);
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST16_OR)
# define HAS_CPU_ATOMIC_FAST16_OR
__CPU_ATOMIC_FALLBACK_BINOP(or, atomic_fast16, |);
#endif


#if defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_XOR)
# define HAS_CPU_ATOMIC_XOR
__CPU_ATOMIC_FALLBACK_BINOP(xor, atomic, ^);
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST8_XOR)
# define HAS_CPU_ATOMIC_FAST8_XOR
__CPU_ATOMIC_FALLBACK_BINOP(xor, atomic_fast8, ^);
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST16_XOR)
# define HAS_CPU_ATOMIC_FAST16_XOR
__CPU_ATOMIC_FALLBACK_BINOP(xor, atomic_fast16, ^);
#endif


#if defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_AND)
# define HAS_CPU_ATOMIC_AND
__CPU_ATOMIC_FALLBACK_BINOP(and, atomic, &);
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST8_AND)
# define HAS_CPU_ATOMIC_FAST8_AND
__CPU_ATOMIC_FALLBACK_BINOP(and, atomic_fast8, &);
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST16_AND)
# define HAS_CPU_ATOMIC_FAST16_AND
__CPU_ATOMIC_FALLBACK_BINOP(and, atomic_fast16, &);
#endif


#define __CPU_ATOMIC_FALLBACK_SWAP(type)                        \
inline type##_int_t                                      \
__cpu_##type##_swap(type##_int_t *a, type##_int_t value)        \
{                                                               \
  type##_int_t old;                                             \
  do {                                                          \
    order_smp_read();                                           \
    old = *a;                                                   \
  } while (!__cpu_##type##_compare_and_swap(a, old, value));    \
                                                                \
  return old;                                                   \
}

#if defined(HAS_CPU_ATOMIC_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_SWAP)
# define HAS_CPU_ATOMIC_SWAP
__CPU_ATOMIC_FALLBACK_SWAP(atomic);
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST8_SWAP)
# define HAS_CPU_ATOMIC_FAST8_SWAP
__CPU_ATOMIC_FALLBACK_SWAP(atomic_fast8);
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP) && !defined(HAS_CPU_ATOMIC_FAST16_SWAP)
# define HAS_CPU_ATOMIC_FAST16_SWAP
__CPU_ATOMIC_FALLBACK_SWAP(atomic_fast16);
#endif


#define __CPU_ATOMIC_FALLBACK_INC(name, type, v)        \
inline bool_t                                           \
__cpu_##type##_##name(type##_int_t *a)                  \
{                                                       \
  return __cpu_##type##_add(a, v) + v != 0;             \
}

# if defined(HAS_CPU_ATOMIC_ADD) && !defined(HAS_CPU_ATOMIC_INC)
#  define HAS_CPU_ATOMIC_INC
__CPU_ATOMIC_FALLBACK_INC(inc, atomic, +1);
# endif

# if defined(HAS_CPU_ATOMIC_FAST8_ADD) && !defined(HAS_CPU_ATOMIC_FAST8_INC)
#  define HAS_CPU_ATOMIC_FAST8_INC
__CPU_ATOMIC_FALLBACK_INC(inc, atomic_fast8, +1);
# endif

# if defined(HAS_CPU_ATOMIC_FAST16_ADD) && !defined(HAS_CPU_ATOMIC_FAST16_INC)
#  define HAS_CPU_ATOMIC_FAST16_INC
__CPU_ATOMIC_FALLBACK_INC(inc, atomic_fast16, +1);
# endif


# if defined(HAS_CPU_ATOMIC_ADD) && !defined(HAS_CPU_ATOMIC_DEC)
#  define HAS_CPU_ATOMIC_DEC
__CPU_ATOMIC_FALLBACK_INC(dec, atomic, -1);
# endif

# if defined(HAS_CPU_ATOMIC_FAST8_ADD) && !defined(HAS_CPU_ATOMIC_FAST8_DEC)
#  define HAS_CPU_ATOMIC_FAST8_DEC
__CPU_ATOMIC_FALLBACK_INC(dec, atomic_fast8, -1);
# endif

# if defined(HAS_CPU_ATOMIC_FAST16_ADD) && !defined(HAS_CPU_ATOMIC_FAST16_DEC)
#  define HAS_CPU_ATOMIC_FAST16_DEC
__CPU_ATOMIC_FALLBACK_INC(dec, atomic_fast16, -1);
# endif


#define __CPU_ATOMIC_FALLBACK_BIT_TEST(name, type, expr)        \
inline bool_t                                            \
__cpu_##type##_bit_##name(type##_int_t *a, uint_fast8_t n)      \
{                                                               \
  type##_int_t mask = 1 << n;                                   \
  return (expr) != 0;                                           \
}

#if defined(HAS_CPU_ATOMIC_OR) && !defined(HAS_CPU_ATOMIC_TESTSET)
# define HAS_CPU_ATOMIC_TESTSET
__CPU_ATOMIC_FALLBACK_BIT_TEST(testset, atomic, __cpu_atomic_or(a, mask) & mask);
# endif

#if defined(HAS_CPU_ATOMIC_FAST8_OR) && !defined(HAS_CPU_ATOMIC_FAST8_TESTSET)
# define HAS_CPU_ATOMIC_FAST8_TESTSET
__CPU_ATOMIC_FALLBACK_BIT_TEST(testset, atomic_fast8, __cpu_atomic_fast8_or(a, mask) & mask);
# endif

#if defined(HAS_CPU_ATOMIC_FAST16_OR) && !defined(HAS_CPU_ATOMIC_FAST16_TESTSET)
# define HAS_CPU_ATOMIC_FAST16_TESTSET
__CPU_ATOMIC_FALLBACK_BIT_TEST(testset, atomic_fast16, __cpu_atomic_fast16_or(a, mask) & mask);
# endif


#if defined(HAS_CPU_ATOMIC_AND) && !defined(HAS_CPU_ATOMIC_TESTCLR)
# define HAS_CPU_ATOMIC_TESTCLR
__CPU_ATOMIC_FALLBACK_BIT_TEST(testclr, atomic, __cpu_atomic_and(a, ~mask) & mask);
# endif

#if defined(HAS_CPU_ATOMIC_FAST8_AND) && !defined(HAS_CPU_ATOMIC_FAST8_TESTCLR)
# define HAS_CPU_ATOMIC_FAST8_TESTCLR
__CPU_ATOMIC_FALLBACK_BIT_TEST(testclr, atomic_fast8, __cpu_atomic_fast8_and(a, ~mask) & mask);
# endif

#if defined(HAS_CPU_ATOMIC_FAST16_AND) && !defined(HAS_CPU_ATOMIC_FAST16_TESTCLR)
# define HAS_CPU_ATOMIC_FAST16_TESTCLR
__CPU_ATOMIC_FALLBACK_BIT_TEST(testclr, atomic_fast16, __cpu_atomic_fast16_and(a, ~mask) & mask);
# endif


#define __CPU_ATOMIC_FALLBACK_BIT_WAIT(name, type, expr)        \
inline void                                              \
__cpu_##type##_bit_##name(type##_int_t *a, uint_fast8_t n)      \
{                                                               \
  while (expr)                                                  \
    ;                                                           \
}

#if defined(HAS_CPU_ATOMIC_TESTSET) && !defined(HAS_CPU_ATOMIC_WAITSET)
# define HAS_CPU_ATOMIC_WAITSET
__CPU_ATOMIC_FALLBACK_BIT_WAIT(waitset, atomic, __cpu_atomic_bit_testset(a, n))
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_TESTSET) && !defined(HAS_CPU_ATOMIC_FAST8_WAITSET)
# define HAS_CPU_ATOMIC_FAST8_WAITSET
__CPU_ATOMIC_FALLBACK_BIT_WAIT(waitset, atomic_fast8, __cpu_atomic_fast8_bit_testset(a, n))
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_TESTSET) && !defined(HAS_CPU_ATOMIC_FAST16_WAITSET)
# define HAS_CPU_ATOMIC_FAST16_WAITSET
__CPU_ATOMIC_FALLBACK_BIT_WAIT(waitset, atomic_fast16, __cpu_atomic_fast16_bit_testset(a, n))
#endif


#if defined(HAS_CPU_ATOMIC_TESTCLR) && !defined(HAS_CPU_ATOMIC_WAITCLR)
# define HAS_CPU_ATOMIC_WAITCLR
__CPU_ATOMIC_FALLBACK_BIT_WAIT(waitclr, atomic, !__cpu_atomic_bit_testclr(a, n))
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_TESTCLR) && !defined(HAS_CPU_ATOMIC_FAST8_WAITCLR)
# define HAS_CPU_ATOMIC_FAST8_WAITCLR
__CPU_ATOMIC_FALLBACK_BIT_WAIT(waitclr, atomic_fast8, !__cpu_atomic_fast8_bit_testclr(a, n))
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_TESTCLR) && !defined(HAS_CPU_ATOMIC_FAST16_WAITCLR)
# define HAS_CPU_ATOMIC_FAST16_WAITCLR
__CPU_ATOMIC_FALLBACK_BIT_WAIT(waitclr, atomic_fast16, !__cpu_atomic_fast16_bit_testclr(a, n))
#endif


#define __CPU_ATOMIC_FALLBACK_BIT_UPDATE(name, type)            \
inline void                                                     \
__cpu_##type##_bit_##name(type##_int_t *a, uint_fast8_t n)      \
{                                                               \
  __cpu_atomic_bit_test##name(a, n);                            \
}

#if defined(HAS_CPU_ATOMIC_TESTSET) && !defined(HAS_CPU_ATOMIC_SET)
# define HAS_CPU_ATOMIC_SET
__CPU_ATOMIC_FALLBACK_BIT_UPDATE(set, atomic)
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_TESTSET) && !defined(HAS_CPU_ATOMIC_FAST8_SET)
# define HAS_CPU_ATOMIC_FAST8_SET
__CPU_ATOMIC_FALLBACK_BIT_UPDATE(set, atomic_fast8)
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_TESTSET) && !defined(HAS_CPU_ATOMIC_FAST16_SET)
# define HAS_CPU_ATOMIC_FAST16_SET
__CPU_ATOMIC_FALLBACK_BIT_UPDATE(set, atomic_fast16)
#endif

#if defined(HAS_CPU_ATOMIC_TESTCLR) && !defined(HAS_CPU_ATOMIC_CLR)
# define HAS_CPU_ATOMIC_CLR
__CPU_ATOMIC_FALLBACK_BIT_UPDATE(clr, atomic)
#endif

#if defined(HAS_CPU_ATOMIC_FAST8_TESTCLR) && !defined(HAS_CPU_ATOMIC_FAST8_CLR)
# define HAS_CPU_ATOMIC_FAST8_CLR
__CPU_ATOMIC_FALLBACK_BIT_UPDATE(clr, atomic_fast8)
#endif

#if defined(HAS_CPU_ATOMIC_FAST16_TESTCLR) && !defined(HAS_CPU_ATOMIC_FAST16_CLR)
# define HAS_CPU_ATOMIC_FAST16_CLR
__CPU_ATOMIC_FALLBACK_BIT_UPDATE(clr, atomic_fast16)
#endif


#ifdef HAS_CPU_ATOMIC_FAST16_ALIAS
# ifdef HAS_CPU_ATOMIC_ADD
#  define HAS_CPU_ATOMIC_FAST16_ADD
#  define __cpu_atomic_fast16_add              __cpu_atomic_add
# endif
# ifdef HAS_CPU_ATOMIC_OR
#  define HAS_CPU_ATOMIC_FAST16_OR
#  define __cpu_atomic_fast16_or               __cpu_atomic_or
# endif
# ifdef HAS_CPU_ATOMIC_XOR
#  define HAS_CPU_ATOMIC_FAST16_XOR
#  define __cpu_atomic_fast16_xor              __cpu_atomic_xor
# endif
# ifdef HAS_CPU_ATOMIC_AND
#  define HAS_CPU_ATOMIC_FAST16_AND
#  define __cpu_atomic_fast16_and              __cpu_atomic_and
# endif
# ifdef HAS_CPU_ATOMIC_SWAP
#  define HAS_CPU_ATOMIC_FAST16_SWAP
#  define __cpu_atomic_fast16_swap             __cpu_atomic_swap
# endif
# ifdef HAS_CPU_ATOMIC_INC
#  define HAS_CPU_ATOMIC_FAST16_INC
#  define __cpu_atomic_fast16_inc              __cpu_atomic_inc
# endif
# ifdef HAS_CPU_ATOMIC_DEC
#  define HAS_CPU_ATOMIC_FAST16_DEC
#  define __cpu_atomic_fast16_dec              __cpu_atomic_dec
# endif
# ifdef HAS_CPU_ATOMIC_SET
#  define HAS_CPU_ATOMIC_FAST16_SET
#  define __cpu_atomic_fast16_bit_set          __cpu_atomic_bit_set
# endif
# ifdef HAS_CPU_ATOMIC_TESTSET
#  define HAS_CPU_ATOMIC_FAST16_TESTSET
#  define __cpu_atomic_fast16_bit_testset      __cpu_atomic_bit_testset
# endif
# ifdef HAS_CPU_ATOMIC_WAITSET
#  define HAS_CPU_ATOMIC_FAST16_WAITSET
#  define __cpu_atomic_fast16_bit_waitset      __cpu_atomic_bit_waitset
# endif
# ifdef HAS_CPU_ATOMIC_CLR
#  define HAS_CPU_ATOMIC_FAST16_CLR
#  define __cpu_atomic_fast16_bit_clr          __cpu_atomic_bit_clr
# endif
# ifdef HAS_CPU_ATOMIC_TESTCLR
#  define HAS_CPU_ATOMIC_FAST16_TESTCLR
#  define __cpu_atomic_fast16_bit_testclr      __cpu_atomic_bit_testclr
# endif
# ifdef HAS_CPU_ATOMIC_WAITCLR
#  define HAS_CPU_ATOMIC_FAST16_WAITCLR
#  define __cpu_atomic_fast16_bit_waitclr      __cpu_atomic_bit_waitclr
# endif
# ifdef HAS_CPU_ATOMIC_COMPARE_AND_SWAP
#  define HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP
#  define __cpu_atomic_fast16_compare_and_swap __cpu_atomic_compare_and_swap
# endif
#endif

#ifdef HAS_CPU_ATOMIC_FAST8_ALIAS
# ifdef HAS_CPU_ATOMIC_ADD
#  define HAS_CPU_ATOMIC_FAST8_ADD
#  define __cpu_atomic_fast8_add              __cpu_atomic_add
# endif
# ifdef HAS_CPU_ATOMIC_OR
#  define HAS_CPU_ATOMIC_FAST8_OR
#  define __cpu_atomic_fast8_or               __cpu_atomic_or
# endif
# ifdef HAS_CPU_ATOMIC_XOR
#  define HAS_CPU_ATOMIC_FAST8_XOR
#  define __cpu_atomic_fast8_xor              __cpu_atomic_xor
# endif
# ifdef HAS_CPU_ATOMIC_AND
#  define HAS_CPU_ATOMIC_FAST8_AND
#  define __cpu_atomic_fast8_and              __cpu_atomic_and
# endif
# ifdef HAS_CPU_ATOMIC_SWAP
#  define HAS_CPU_ATOMIC_FAST8_SWAP
#  define __cpu_atomic_fast8_swap             __cpu_atomic_swap
# endif
# ifdef HAS_CPU_ATOMIC_INC
#  define HAS_CPU_ATOMIC_FAST8_INC
#  define __cpu_atomic_fast8_inc              __cpu_atomic_inc
# endif
# ifdef HAS_CPU_ATOMIC_DEC
#  define HAS_CPU_ATOMIC_FAST8_DEC
#  define __cpu_atomic_fast8_dec              __cpu_atomic_dec
# endif
# ifdef HAS_CPU_ATOMIC_SET
#  define HAS_CPU_ATOMIC_FAST8_SET
#  define __cpu_atomic_fast8_bit_set          __cpu_atomic_bit_set
# endif
# ifdef HAS_CPU_ATOMIC_TESTSET
#  define HAS_CPU_ATOMIC_FAST8_TESTSET
#  define __cpu_atomic_fast8_bit_testset      __cpu_atomic_bit_testset
# endif
# ifdef HAS_CPU_ATOMIC_WAITSET
#  define HAS_CPU_ATOMIC_FAST8_WAITSET
#  define __cpu_atomic_fast8_bit_waitset      __cpu_atomic_bit_waitset
# endif
# ifdef HAS_CPU_ATOMIC_CLR
#  define HAS_CPU_ATOMIC_FAST8_CLR
#  define __cpu_atomic_fast8_bit_clr          __cpu_atomic_bit_clr
# endif
# ifdef HAS_CPU_ATOMIC_TESTCLR
#  define HAS_CPU_ATOMIC_FAST8_TESTCLR
#  define __cpu_atomic_fast8_bit_testclr      __cpu_atomic_bit_testclr
# endif
# ifdef HAS_CPU_ATOMIC_WAITCLR
#  define HAS_CPU_ATOMIC_FAST8_WAITCLR
#  define __cpu_atomic_fast8_bit_waitclr      __cpu_atomic_bit_waitclr
# endif
# ifdef HAS_CPU_ATOMIC_COMPARE_AND_SWAP
#  define HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP
#  define __cpu_atomic_fast8_compare_and_swap __cpu_atomic_compare_and_swap
# endif
#endif

#endif
