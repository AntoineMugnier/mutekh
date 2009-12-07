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

/**
   @file CPU atomic operations
*/

#if !defined(ATOMIC_H_) || defined(CPU_ATOMIC_H_)
#error This file can not be included directly
#else

#define CPU_ATOMIC_H_

#define HAS_CPU_ATOMIC_INC

static inline bool_t
cpu_atomic_inc(volatile atomic_int_t *a)
{
    reg_t  result, tmp;

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

static inline bool_t
cpu_atomic_dec(volatile atomic_int_t *a)
{
    reg_t  result, tmp;

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

#define HAS_CPU_ATOMIC_ADD

static inline atomic_int_t
cpu_atomic_add(volatile atomic_int_t *a, atomic_int_t val)
{
    reg_t  result, tmp;

    asm volatile (
        "1:     ll      %[result], %[atomic]      \n"
        "       addu    %[tmp], %[result], %[val] \n"
        "       sc      %[tmp], %[atomic]         \n"
        "       beqz    %[tmp], 1b                \n"
        : [tmp] "=&r" (tmp), [result] "=&r" (result)
        , [atomic] "=m" (*a)
        : [val] "r" (val)
        , [atomic_in] "m" (*a)
        );

    return result;
}

#define HAS_CPU_ATOMIC_TESTSET

static inline bool_t
cpu_atomic_bit_testset(volatile atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t result, tmp, loaded;

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

static inline void
cpu_atomic_bit_waitset(volatile atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t tmp, loaded;

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

static inline bool_t
cpu_atomic_bit_testclr(volatile atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t result, tmp, loaded;

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

static inline void
cpu_atomic_bit_waitclr(volatile atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t tmp, loaded;

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

static inline void
cpu_atomic_bit_set(volatile atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = 1 << n;
    reg_t tmp;

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

static inline void
cpu_atomic_bit_clr(volatile atomic_int_t *a, uint_fast8_t n)
{
    reg_t mask = ~(1 << n);
    reg_t tmp;

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

#endif

