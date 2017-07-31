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

/**
   @file CPU atomic operations
*/

#if !defined(ATOMIC_H_) || defined(CPU_ATOMIC_H_)
#error This file can not be included directly
#else

#include "asm.h"

#define CPU_ATOMIC_H_

#define __ATOMIC_ARM32(type, width)                                     \
                                                                        \
inline type##_int_t                                                     \
__cpu_##type##_add(type##_int_t *a, type##_int_t value)                 \
{                                                                       \
  reg_t tmp, tmp2, tmp3;                                                \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "add     %[tmp2], %[tmp], %[value]   \n\t"               \
               "strex" width "   %[tmp3], %[tmp2], [%[atomic]]   \n\t"  \
               "tst     %[tmp3], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)                 \
                 , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)             \
                 /*,*/ THUMB_OUT(,)                                     \
               : [atomic] "r" (a), [value] "r" (value)                  \
               );                                                       \
                                                                        \
  return tmp;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t                                                     \
__cpu_##type##_or(type##_int_t *a, type##_int_t value)                  \
{                                                                       \
  reg_t tmp, tmp2, tmp3;                                                \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "orr     %[tmp2], %[tmp], %[value]   \n\t"               \
               "strex" width "   %[tmp3], %[tmp2], [%[atomic]]   \n\t"  \
               "tst     %[tmp3], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)                 \
                 , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)             \
                 /*,*/ THUMB_OUT(,)                                     \
               : [atomic] "r" (a), [value] "r" (value)                  \
               );                                                       \
                                                                        \
  return tmp;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t                                                     \
__cpu_##type##_xor(type##_int_t *a, type##_int_t value)                 \
{                                                                       \
  reg_t tmp, tmp2, tmp3;                                                \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "eor     %[tmp2], %[tmp], %[value]   \n\t"               \
               "strex" width "   %[tmp3], %[tmp2], [%[atomic]]   \n\t"  \
               "tst     %[tmp3], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)                 \
                 , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)             \
                 /*,*/ THUMB_OUT(,)                                     \
               : [atomic] "r" (a), [value] "r" (value)                  \
               );                                                       \
                                                                        \
  return tmp;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t                                                     \
__cpu_##type##_and(type##_int_t *a, type##_int_t value)                 \
{                                                                       \
  reg_t tmp, tmp2, tmp3;                                                \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "and     %[tmp2], %[tmp], %[value]   \n\t"               \
               "strex" width "   %[tmp3], %[tmp2], [%[atomic]]   \n\t"  \
               "tst     %[tmp3], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)                 \
                 , [tmp3] "=&r" (tmp3), [clobber] "=m" (*a)             \
                 /*,*/ THUMB_OUT(,)                                     \
               : [atomic] "r" (a), [value] "r" (value)                  \
               );                                                       \
                                                                        \
  return tmp;                                                           \
}                                                                       \
                                                                        \
inline type##_int_t                                                     \
__cpu_##type##_swap(type##_int_t *a, type##_int_t value)                \
{                                                                       \
  reg_t tmp, tmp2;                                                      \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "strex" width "   %[tmp2], %[value], [%[atomic]]   \n\t" \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)                 \
                 , [clobber] "=m" (*a)                                  \
                 /*,*/ THUMB_OUT(,)                                     \
               : [atomic] "r" (a), [value] "r" (value)                  \
               );                                                       \
                                                                        \
  return tmp;                                                           \
}                                                                       \
                                                                        \
inline bool_t                                                           \
__cpu_##type##_inc(type##_int_t *a)                                     \
{                                                                       \
  reg_t tmp = 0, tmp2;                                                  \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "add     %[tmp], %[tmp], #1   \n\t"                      \
               "strex" width "   %[tmp2], %[tmp], [%[atomic]]   \n\t"   \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2),                \
                 [clobber] "=m" (*a) /*,*/ THUMB_OUT(,)                 \
               : [atomic] "r" (a)                                       \
               );                                                       \
                                                                        \
  return tmp != 0;                                                      \
}                                                                       \
                                                                        \
inline bool_t                                                           \
__cpu_##type##_dec(type##_int_t *a)                                     \
{                                                                       \
  reg_t tmp = 0, tmp2;                                                  \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "sub     %[tmp], %[tmp], #1   \n\t"                      \
               "strex" width "   %[tmp2], %[tmp], [%[atomic]]   \n\t"   \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2),                \
                 [clobber] "=m" (*a) /*,*/ THUMB_OUT(,)                 \
               : [atomic] "r" (a)                                       \
               );                                                       \
                                                                        \
  return tmp != 0;                                                      \
}                                                                       \
                                                                        \
inline bool_t                                                           \
__cpu_##type##_bit_testset(type##_int_t *a, uint_fast8_t n)             \
{                                                                       \
  reg_t mask = 1 << n;                                                  \
  reg_t tmp, tmp2, tmp3;                                                \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "tst     %[tmp], %[mask]       \n\t"                     \
               "bne     2f           \n\t"                              \
               "orr     %[tmp2], %[tmp], %[mask]   \n\t"                \
               "strex" width "   %[tmp3], %[tmp2], [%[atomic]]   \n\t"  \
               "tst     %[tmp3], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               "2:                   \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2)                 \
                 , [tmp3] "=&r"(tmp3), [clobber] "=m" (*a)              \
                 /*,*/ THUMB_OUT(,)                                     \
               : [mask] "r" (mask), [atomic] "r" (a)                    \
               );                                                       \
                                                                        \
  return (tmp & mask) != 0;                                             \
}                                                                       \
                                                                        \
inline void                                                             \
__cpu_##type##_bit_waitset(type##_int_t *a, uint_fast8_t n)             \
{                                                                       \
  reg_t mask = 1 << n;                                                  \
  reg_t tmp = 0, tmp2;                                                  \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "tst     %[tmp], %[mask]       \n\t"                     \
               "bne     1b           \n\t"                              \
               "orr     %[tmp], %[tmp], %[mask]   \n\t"                 \
               "strex" width "   %[tmp2], %[tmp], [%[atomic]]   \n\t"   \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [clobber] "=m" (*a)                 \
                 , [tmp2] "=&r" (tmp2)                                  \
                 /*,*/ THUMB_OUT(,)                                     \
               : [mask] "r" (mask), [atomic] "r" (a)                    \
               );                                                       \
}                                                                       \
                                                                        \
inline bool_t                                                           \
__cpu_##type##_bit_testclr(type##_int_t *a, uint_fast8_t n)             \
{                                                                       \
  reg_t mask = 1 << n;                                                  \
  reg_t tmp, tmp2, tmp3;                                                \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "tst     %[tmp], %[mask]       \n\t"                     \
               "beq     2f           \n\t"                              \
               "bic     %[tmp2], %[tmp], %[mask]   \n\t"                \
               "strex" width "   %[tmp3], %[tmp2], [%[atomic]]   \n\t"  \
               "tst     %[tmp3], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               "2:                   \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2),                \
                 [clobber] "=m" (*a), [tmp3] "=&r"(tmp3)                \
                 /*,*/ THUMB_OUT(,)                                     \
               : [mask] "r" (mask), [atomic] "r" (a)                    \
               );                                                       \
                                                                        \
  return (tmp & mask) != 0;                                             \
}                                                                       \
                                                                        \
inline void                                                             \
__cpu_##type##_bit_waitclr(type##_int_t *a, uint_fast8_t n)             \
{                                                                       \
  reg_t mask = 1 << n;                                                  \
  reg_t tmp = 0, tmp2;                                                  \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "tst     %[tmp], %[mask]       \n\t"                     \
               "beq     1b           \n\t"                              \
               "bic     %[tmp], %[tmp], %[mask]   \n\t"                 \
               "strex" width "   %[tmp2], %[tmp], [%[atomic]]   \n\t"   \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2),                \
                 [clobber] "=m" (*a) /*,*/ THUMB_OUT(,)                 \
               : [mask] "r" (mask), [atomic] "r" (a)                    \
               );                                                       \
}                                                                       \
                                                                        \
inline void                                                             \
__cpu_##type##_bit_set(type##_int_t *a, uint_fast8_t n)                 \
{                                                                       \
  reg_t mask = 1 << n;                                                  \
  reg_t tmp, tmp2;                                                      \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "orr     %[tmp], %[tmp], %[mask]   \n\t"                 \
               "strex" width "   %[tmp2], %[tmp], [%[atomic]]   \n\t"   \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2),                \
                 [clobber] "=m" (*a) /*,*/ THUMB_OUT(,)                 \
               : [mask] "r" (mask), [atomic] "r" (a)                    \
               );                                                       \
}                                                                       \
                                                                        \
inline void                                                             \
__cpu_##type##_bit_clr(type##_int_t *a, uint_fast8_t n)                 \
{                                                                       \
  reg_t mask = 1 << n;                                                  \
  reg_t tmp, tmp2;                                                      \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                   \n\t"                              \
               "ldrex" width "   %[tmp], [%[atomic]]       \n\t"        \
               "bic     %[tmp], %[tmp], %[mask]   \n\t"                 \
               "strex" width "   %[tmp2], %[tmp], [%[atomic]]   \n\t"   \
               "tst     %[tmp2], #1       \n\t"                         \
               "bne     1b           \n\t"                              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [tmp2] "=&r" (tmp2),                \
                 [clobber] "=m" (*a) /*,*/ THUMB_OUT(,)                 \
               : [mask] "r" (mask), [atomic] "r" (a)                    \
               );                                                       \
}                                                                       \
                                                                        \
inline bool_t                                                           \
__cpu_##type##_compare_and_swap(type##_int_t *a, type##_int_t old,      \
                                type##_int_t future)                    \
{                                                                       \
  reg_t tmp, loaded;                                                    \
  THUMB_TMP_VAR;                                                        \
                                                                        \
  asm volatile(                                                         \
               THUMB_TO_ARM                                             \
               "1:                                   \n\t"              \
               "ldrex" width "   %[loaded], [%[atomic]]       \n\t"     \
               "cmp     %[loaded], %[old]            \n\t"              \
               "bne     2f                           \n\t"              \
               "strex" width "   %[tmp], %[future], [%[atomic]]  \n\t"  \
               "tst     %[tmp], #1                   \n\t"              \
               "bne     1b                           \n\t"              \
               "2:                                   \n\t"              \
               ARM_TO_THUMB                                             \
               : [tmp] "=&r" (tmp), [loaded] "=&r" (loaded), "=m" (*a)  \
                 /*,*/ THUMB_OUT(,)                                     \
               : [old] "r" (old), [future] "r" (future), [atomic] "r" (a) \
               );                                                       \
                                                                        \
  return loaded == old;                                                 \
}

typedef int32_t atomic_int_t;

#if CONFIG_CPU_ARM32_ARCH_VERSION >= 6

# define HAS_CPU_ATOMIC_ADD
# define HAS_CPU_ATOMIC_OR
# define HAS_CPU_ATOMIC_XOR
# define HAS_CPU_ATOMIC_AND
# define HAS_CPU_ATOMIC_SWAP
# define HAS_CPU_ATOMIC_INC
# define HAS_CPU_ATOMIC_DEC
# define HAS_CPU_ATOMIC_TESTSET
# define HAS_CPU_ATOMIC_WAITSET
# define HAS_CPU_ATOMIC_TESTCLR
# define HAS_CPU_ATOMIC_WAITCLR
# define HAS_CPU_ATOMIC_SET
# define HAS_CPU_ATOMIC_CLR
# define HAS_CPU_ATOMIC_COMPARE_AND_SWAP

__ATOMIC_ARM32(atomic, "");

# if CONFIG_CPU_ARM32_ARCH_VERSION >= 7

typedef int16_t atomic_fast16_int_t;

#  define HAS_CPU_ATOMIC_FAST16_ADD
#  define HAS_CPU_ATOMIC_FAST16_OR
#  define HAS_CPU_ATOMIC_FAST16_XOR
#  define HAS_CPU_ATOMIC_FAST16_AND
#  define HAS_CPU_ATOMIC_FAST16_SWAP
#  define HAS_CPU_ATOMIC_FAST16_INC
#  define HAS_CPU_ATOMIC_FAST16_DEC
#  define HAS_CPU_ATOMIC_FAST16_TESTSET
#  define HAS_CPU_ATOMIC_FAST16_WAITSET
#  define HAS_CPU_ATOMIC_FAST16_TESTCLR
#  define HAS_CPU_ATOMIC_FAST16_WAITCLR
#  define HAS_CPU_ATOMIC_FAST16_SET
#  define HAS_CPU_ATOMIC_FAST16_CLR
#  define HAS_CPU_ATOMIC_FAST16_COMPARE_AND_SWAP

__ATOMIC_ARM32(atomic_fast16, "h");

typedef int8_t atomic_fast8_int_t;

#  define HAS_CPU_ATOMIC_FAST8_ADD
#  define HAS_CPU_ATOMIC_FAST8_OR
#  define HAS_CPU_ATOMIC_FAST8_XOR
#  define HAS_CPU_ATOMIC_FAST8_AND
#  define HAS_CPU_ATOMIC_FAST8_SWAP
#  define HAS_CPU_ATOMIC_FAST8_INC
#  define HAS_CPU_ATOMIC_FAST8_DEC
#  define HAS_CPU_ATOMIC_FAST8_TESTSET
#  define HAS_CPU_ATOMIC_FAST8_WAITSET
#  define HAS_CPU_ATOMIC_FAST8_TESTCLR
#  define HAS_CPU_ATOMIC_FAST8_WAITCLR
#  define HAS_CPU_ATOMIC_FAST8_SET
#  define HAS_CPU_ATOMIC_FAST8_CLR
#  define HAS_CPU_ATOMIC_FAST8_COMPARE_AND_SWAP

__ATOMIC_ARM32(atomic_fast8, "b");

# else  /* v6, use 32 bits for all atomic types */

typedef int32_t atomic_fast16_int_t;
#define HAS_CPU_ATOMIC_FAST16_ALIAS

typedef int32_t atomic_fast8_int_t;
#define HAS_CPU_ATOMIC_FAST8_ALIAS

# endif

#else  /* <= v5, rely on hexo/atomic_cpu_nosmp.h */

typedef int16_t atomic_fast16_int_t;
typedef int8_t atomic_fast8_int_t;

#endif

#endif

