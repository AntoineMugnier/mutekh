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

#include <hexo/ordering.h>

#define CPU_ATOMIC_H_

typedef int32_t atomic_int_t;

typedef int32_t atomic_fast16_int_t;
#define HAS_CPU_ATOMIC_FAST16_ALIAS

typedef int32_t atomic_fast8_int_t;
#define HAS_CPU_ATOMIC_FAST8_ALIAS


#define HAS_CPU_ATOMIC_ADD

ALWAYS_INLINE atomic_int_t
__cpu_atomic_add(atomic_int_t *a, atomic_int_t value)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile(
        "1:     ll      %[result], %[atomic]      \n"
        "       addu    %[tmp], %[result], %[value] \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        : [value] "r" (value)
        );

    return result;
}

#define HAS_CPU_ATOMIC_OR

ALWAYS_INLINE atomic_int_t
__cpu_atomic_or(atomic_int_t *a, atomic_int_t value)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile(
        "1:     ll      %[result], %[atomic]      \n"
        "       or      %[tmp], %[result], %[value] \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        : [value] "r" (value)
        );

    return result;
}

#define HAS_CPU_ATOMIC_XOR

ALWAYS_INLINE atomic_int_t
__cpu_atomic_xor(atomic_int_t *a, atomic_int_t value)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile(
        "1:     ll      %[result], %[atomic]      \n"
        "       xor     %[tmp], %[result], %[value] \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        : [value] "r" (value)
        );

    return result;
}

#define HAS_CPU_ATOMIC_AND

ALWAYS_INLINE atomic_int_t
__cpu_atomic_and(atomic_int_t *a, atomic_int_t value)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile(
        "1:     ll      %[result], %[atomic]      \n"
        "       and     %[tmp], %[result], %[value] \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        : [value] "r" (value)
        );

    return result;
}

#define HAS_CPU_ATOMIC_SWAP

ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile(
        "1:     ll      %[result], %[atomic]      \n"
        "       move    %[tmp], %[value]          \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        : [value] "r" (value)
        );

    return result;
}

#define HAS_CPU_ATOMIC_INC

ALWAYS_INLINE bool_t
__cpu_atomic_inc(atomic_int_t *a)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile(
        "1:     ll      %[result], %[atomic]      \n"
        "       addiu   %[tmp], %[result], 1      \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        );

    return result + 1 != 0;
}

#define HAS_CPU_ATOMIC_DEC

ALWAYS_INLINE bool_t
__cpu_atomic_dec(atomic_int_t *a)
{
    reg_t  result, tmp;

    order_smp_mem();

    asm volatile (
        "1:     ll      %[result], %[atomic]      \n"
        "       addiu   %[tmp], %[result], -1     \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "+m" (*a)
        );

    return result - 1 != 0;
}

#define HAS_CPU_ATOMIC_TESTSET

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testset(atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t result, tmp, loaded;

    order_smp_mem();

    asm volatile (
        ".set push                                   \n"
        ".set noreorder                              \n"
        "1:     ll      %[loaded], %[atomic]         \n"
        "       and     %[result], %[loaded], %[mask]\n"
        "       bnez    %[result], 2f                \n"
        "       or      %[tmp], %[loaded], %[mask]   \n"
        "       sc      %[tmp], %[atomic]            \n"
        ".set pop                                    \n"
        "       beqz    %[tmp], 1b                   \n"
        "2:                                          \n"
        : [tmp] "=&r" (tmp), [loaded] "=&r" (loaded)
        , [atomic] "+m" (*a), [result] "=&r" (result)
        : [mask] "r" (mask)
        );

    return result != 0;
}

#define HAS_CPU_ATOMIC_WAITSET

ALWAYS_INLINE void
__cpu_atomic_bit_waitset(atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t tmp, loaded;

    order_smp_mem();

    asm volatile(
        ".set push                                  \n"
        ".set noreorder                             \n"
        "1:     ll      %[loaded], %[atomic]        \n"
        "       and     %[tmp], %[loaded], %[mask]  \n"
        "       bnez    %[tmp], 1b                  \n"
        "       or      %[tmp], %[loaded], %[mask]  \n"
        "       sc      %[tmp], %[atomic]           \n"
        ".set pop                                   \n"
        "       beqz    %[tmp], 1b                  \n"
        "2:                                         \n"
        : [tmp] "=&r" (tmp), [loaded] "=&r" (loaded)
        , [atomic] "+m" (*a)
        : [mask] "r" (mask)
        );
}

#define HAS_CPU_ATOMIC_TESTCLR

ALWAYS_INLINE bool_t
__cpu_atomic_bit_testclr(atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t result, tmp, loaded;

    order_smp_mem();

    asm volatile(
        ".set push                                    \n"
        ".set noreorder                               \n"
        "1:     ll      %[loaded], %[atomic]          \n"
        "       and     %[result], %[loaded], %[mask] \n"
        "       beqz    %[result], 2f                 \n"
        "       xor     %[tmp], %[loaded], %[mask]    \n"
        "       sc      %[tmp], %[atomic]             \n"
        ".set pop                                     \n"
        "       beqz    %[tmp], 1b                    \n"
        "2:                                           \n"
        : [tmp] "=&r" (tmp), [loaded] "=&r" (loaded)
        , [atomic] "+m" (*a), [result] "=&r" (result)
        : [mask] "r" (mask)
        );

    return result != 0;
}

#define HAS_CPU_ATOMIC_WAITCLR

ALWAYS_INLINE void
__cpu_atomic_bit_waitclr(atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t tmp, loaded;

    order_smp_mem();

    asm volatile(
        ".set push                                 \n"
        ".set noreorder                            \n"
        "1:     ll      %[loaded], %[atomic]       \n"
        "       and     %[tmp], %[loaded], %[mask] \n"
        "       beqz    %[tmp], 1b                 \n"
        "       xor     %[tmp], %[loaded], %[mask] \n"
        "       sc      %[tmp], %[atomic]          \n"
        ".set pop                                  \n"
        "       beqz    %[tmp], 1b                 \n"
        "2:                                        \n"
        : [tmp] "=&r" (tmp), [loaded] "=&r" (loaded)
        , [atomic] "+m" (*a)
        : [mask] "r" (mask)
        );
}

#define HAS_CPU_ATOMIC_SET

ALWAYS_INLINE void
__cpu_atomic_bit_set(atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t tmp;

    order_smp_mem();

    asm volatile(
        ".set push                              \n"
        ".set noreorder                         \n"
        "1:     ll      %[tmp], %[atomic]       \n"
        "       or      %[tmp], %[tmp], %[mask] \n"
        "       sc      %[tmp], %[atomic]       \n"
        ".set pop                               \n"
        "       beqz    %[tmp], 1b              \n"
        : [tmp] "=&r" (tmp), [atomic] "+m" (*a)
        : [mask] "r" (mask)
        );
}

#define HAS_CPU_ATOMIC_CLR

ALWAYS_INLINE void
__cpu_atomic_bit_clr(atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = ~(1 << n);
    reg_t tmp;

    order_smp_mem();

    asm volatile(
        ".set push                              \n"
        ".set noreorder                         \n"
        "1:     ll      %[tmp], %[atomic]       \n"
        "       and     %[tmp], %[tmp], %[mask] \n"
        "       sc      %[tmp], %[atomic]       \n"
        ".set pop                               \n"
        "       beqz    %[tmp], 1b              \n"
        : [tmp] "=&r" (tmp), [atomic] "+m" (*a)
        : [mask] "r" (mask)
        );
}

#define HAS_CPU_ATOMIC_COMPARE_AND_SWAP

ALWAYS_INLINE bool_t
__cpu_atomic_compare_and_swap(atomic_int_t *a, atomic_int_t old, atomic_int_t future)
{
    reg_t tmp, loaded;

    order_smp_mem();

    asm volatile (
        ".set push                                   \n"
        ".set noreorder                              \n"
        "1:     ll      %[loaded], %[atomic]         \n"
        "       bne     %[loaded], %[old], 2f        \n"
        "       move    %[tmp], %[future]               \n"
        "       sc      %[tmp], %[atomic]            \n"
        "       beqz    %[tmp], 1b                   \n"
        "       nop                                  \n"
        "2:                                          \n"
        ".set pop                                    \n"
        : [tmp] "=&r" (tmp), [loaded] "=&r" (loaded), [atomic] "+m" (*a)
        : [old] "r" (old), [future] "r" (future)
        : "memory"
        );

    return loaded == old;
}

#endif

