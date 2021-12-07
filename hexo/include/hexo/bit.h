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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2016 Nicolas Pouillon <nipo@ssji.net> 
    Copyright (c) 2017 Alexandre Becoulet <alexandre.becoulet@free.fr>
*/

/**
   @file
   @module {Core::Hardware abstraction layer}
   @short Bit operations
 */

#ifndef __BIT_H_
#define __BIT_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include "types.h"

#include "cpu/hexo/bit.h"

/** @this returns true if value is a power of 2 */
#define is_pow2(x) (!((x) & ((x) - 1)))

/** @this returns true if value is multiple of b, which must be a
    power of 2 */
#define is_pow2_multiple(x, b) (!((x) & ((b) - 1)))

/** @this aligns value on the next power of two boundary  */
#define align_pow2_up(x, b) ((((x) - 1) | ((b) - 1)) + 1)

/** @this aligns value on the previous power of two boundary */
#define align_pow2_down(x, b) ((x) & ~((b) - 1))

#define __pow2_m1_up1(x)  ((x) | ((x) >> 1))
#define __pow2_m1_up2(x)  (__pow2_m1_up1(x) | (__pow2_m1_up1(x) >> 2))
#define __pow2_m1_up4(x)  (__pow2_m1_up2(x) | (__pow2_m1_up2(x) >> 4))
#define __pow2_m1_up8(x)  (__pow2_m1_up4(x) | (__pow2_m1_up4(x) >> (8 % (sizeof(x)*8))))
#define __pow2_m1_up16(x) (__pow2_m1_up8(x) | (__pow2_m1_up8(x) >> (16 % (sizeof(x)*8))))

/** @this returns a power of 2 minus one where the pow2 is greater
    than the specified value */
#define pow2_m1_up(x)    (__pow2_m1_up16(x) | (__pow2_m1_up16(x) >> (32 % (sizeof(x)*8))))

/** @this returns a power of 2 greater or equal to the specified value */
#define pow2_up(x) (pow2_m1_up(x - 1) + 1)

/** @this selects bit @tt n */
#define bit(n) (1ull << (n))

/** @this makes a bit mask of @tt count bits width from bit @tt index */
#define bit_mask(index, count) ((bit(count) - 1) << (index))

/** @this makes a bit mask from @tt first bit to @tt last bit (included) */
#define bit_range(first, last) (bit((last) + 1) - bit(first))

/** @this extracts bit at specified index */
#define bit_get(value, index) (typeof(value))(((value) >> (index)) & 1)

/** @this extracts @tt count bits from specified @tt index in @tt
    value */
#define bit_get_mask(value, index, count)  \
  (typeof(value))(((value) >> (index)) & bit_mask(0, count))

/** @this extracts bits between specified @tt first and @tt last index
    (included) in @tt value */
#define bit_get_range(value, first, last) \
  (typeof(value))bit_get_mask(value, first, (last) - (first) + 1)



/** @this returns whether an address is aligned */
#define address_is_aligned(x, b) is_pow2_multiple((uintptr_t)(x), b)

/** @this aligns address on the next power of two boundary  */
#define address_align_up(x, b) ((void *)align_pow2_up((uintptr_t)(x), (b)))

/** @this aligns address on the next power of two boundary  */
#define address_align_down(x, b) ((void*)align_pow2_down((uintptr_t)(x), (b)))


/** @this packs 4 bytes and returns a 32 bits word */
#define byte_pack32(msb, a, b, lsb) \
  ((((msb) & 0xff) << 24) | (((a) & 0xff ) << 16) | (((b) & 0xff) << 8) | ((lsb) & 0xff))

/** @this packs 2 bytes and returns a 16 bits word */
#define byte_pack16(msb, lsb) \
  ((((msb) & 0xff) << 8) | ((lsb) & 0xff))


/** @this sets bit @tt bit in @tt value */
#define BIT_SET(value, b) ((value) |= bit(b))

/** @this clears bit @tt bit in @tt value */
#define BIT_CLEAR(value, b) ((value) &= ~bit(b))

/** @this inserts @tt count bits value @tt ins at specified @tt index
    in @tt value */
#define BIT_INSERT(value, ins, index, count)            \
  (((value) & ~bit_mask(index, count))                 \
   |(((ins) << (index)) & bit_mask(index, count)))

#define __BIT_FUNC_GEN(n, l)                                          \
/** @internal */                                                      \
inline uint_fast8_t __bit_ctz##n(uint##n##_t x)                       \
{                                                                     \
  x &= -x;                                                            \
  uint##n##_t c = (x & (uint##n##_t)0x5555555555555555ULL) - 1;       \
  c = (c >> 1) ^ ((x & (uint##n##_t)0x3333333333333333ULL) - 1);      \
  c = (c >> 1) ^ ((x & (uint##n##_t)0x0f0f0f0f0f0f0f0fULL) - 1);      \
  if (n > 8)                                                          \
    c = (c >> 1) ^ ((x & (uint##n##_t)0x00ff00ff00ff00ffULL) - 1);    \
  if (n > 16)                                                         \
    c = (c >> 1) ^ ((x & (uint##n##_t)0x0000ffff0000ffffULL) - 1);    \
  if (n > 32)                                                         \
    c = (c >> 1) ^ ((x & (uint##n##_t)0x00000000ffffffffULL) - 1);    \
  return (c >> (n - l)) ^ (c >> (n - l + 1));                         \
}                                                                     \
                                                                      \
/** @internal */                                                      \
inline uint_fast8_t __bit_clz##n(uint##n##_t x)                       \
{                                                                     \
  uint##n##_t a0, a1, a2, a3, a4, j = 0;                              \
  a0 = x  | (( x & (uint##n##_t)0xaaaaaaaaaaaaaaaaULL) >> 1);         \
  a1 = a0 | ((a0 & (uint##n##_t)0xccccccccccccccccULL) >> 2);         \
  a2 = a1 | ((a1 & (uint##n##_t)0xf0f0f0f0f0f0f0f0ULL) >> 4);         \
  a3 = a2 | ((a2 & (uint##n##_t)0xff00ff00ff00ff00ULL) >> 8);         \
  a4 = a3 | ((a3 & (uint##n##_t)0xffff0000ffff0000ULL) >> 16);        \
  if (n > 32)                                                         \
    j |= (a4 >> (j + 32-5)) & 32;                                     \
  if (n > 16)                                                         \
    j |= (a3 >> (j + 16-4)) & 16;                                     \
  if (n > 8)                                                          \
    j |= (a2 >> (j + 8-3))  & 8;                                      \
  j |= (a1 >> (j + 4-2))  & 4;                                        \
  j |= (a0 >> (j + 2-1))  & 2;                                        \
  j |= (x  >> (j + 1-0))  & 1;                                        \
  return j ^ (n - 1);                                                 \
}                                                                     \
                                                                      \
/** @internal */                                                      \
inline uint_fast8_t __bit_popc##n(uint##n##_t x)                      \
{                                                                     \
  x = (x & (uint##n##_t)0x5555555555555555ULL) +                      \
    ((x >> 1) & (uint##n##_t)0x5555555555555555ULL);                  \
  x = (x & (uint##n##_t)0x3333333333333333ULL) +                      \
    ((x >> 2) & (uint##n##_t)0x3333333333333333ULL);                  \
  x = (x & (uint##n##_t)0x0f0f0f0f0f0f0f0fULL) +                      \
    ((x >> 4) & (uint##n##_t)0x0f0f0f0f0f0f0f0fULL);                  \
  if (n > 8)                                                          \
    x = (x & (uint##n##_t)0x00ff00ff00ff00ffULL) +                    \
      ((x >> 8) & (uint##n##_t)0x00ff00ff00ff00ffULL);                \
  if (n > 16)                                                         \
    x = (x & (uint##n##_t)0x0000ffff0000ffffULL) +                    \
      ((x >> 16) & (uint##n##_t)0x0000ffff0000ffffULL);               \
  if (n > 32)                                                         \
    x = (x & 0x00000000ffffffffULL) +                                 \
      (((uint64_t)x >> 32) & 0x00000000ffffffffULL);                  \
  return x;                                                           \
}

__BIT_FUNC_GEN(8, 3);
__BIT_FUNC_GEN(16, 4);
__BIT_FUNC_GEN(32, 5);
__BIT_FUNC_GEN(64, 6);

#ifndef HAS_CPU_BIT_CLZ64
# if defined(HAS_CPU_BIT_CLZ32)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_clz64(uint64_t _x)
{
  uint32_t _h = _x >> 32;
  return _h ? cpu_bit_clz32(_h) : cpu_bit_clz32(_x) + 32;
}
# else
/** @internal */
#  define cpu_bit_clz64(x) __bit_clz64(x)
# endif
#endif

#ifndef HAS_CPU_BIT_CLZ32
# if defined(HAS_CPU_BIT_CLZ64)
/** @internal */
#  define cpu_bit_clz32(x) (cpu_bit_clz64((uint32_t)x) - 32)
# else
/** @internal */
#  define cpu_bit_clz32(x) __bit_clz32(x)
# endif
#endif

#ifndef HAS_CPU_BIT_CLZ16
# if defined(HAS_CPU_BIT_CLZ32)
/** @internal */
#  define cpu_bit_clz16(x) (cpu_bit_clz32((uint16_t)x) - 16)
# elif defined(HAS_CPU_BIT_CLZ64)
/** @internal */
#  define cpu_bit_clz16(x) (cpu_bit_clz64((uint16_t)x) - 48)
# else
/** @internal */
#  define cpu_bit_clz16(x) __bit_clz16(x)
# endif
#endif

#ifndef HAS_CPU_BIT_CLZ8
# if defined(HAS_CPU_BIT_CLZ16)
/** @internal */
#  define cpu_bit_clz8(x) (cpu_bit_clz16((uint8_t)x) - 8)
# elif defined(HAS_CPU_BIT_CLZ32)
/** @internal */
#  define cpu_bit_clz8(x) (cpu_bit_clz32((uint8_t)x) - 24)
# elif defined(HAS_CPU_BIT_CLZ64)
/** @internal */
#  define cpu_bit_clz8(x) (cpu_bit_clz64((uint8_t)x) - 56)
# else
/** @internal */
#  define cpu_bit_clz8(x) __bit_clz8(x)
# endif
#endif

/** @This returns the number of leading zero bits in a 8 bits
    value. The result is undefined if the input is 0. @csee #bit_clz_unsafe */
#define bit_clz8(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                          __builtin_clzll((uint8_t)x) - 56, cpu_bit_clz8(x))

/** @This returns the number of leading zero bits in a 16 bits
    value. The result is undefined if the input is 0. @csee #bit_clz_unsafe */
#define bit_clz16(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                           __builtin_clzll((uint16_t)x) - 48, cpu_bit_clz16(x))

/** @This returns the number of leading zero bits in a 32 bits
    value. The result is undefined if the input is 0. @csee #bit_clz_unsafe */
#define bit_clz32(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                           __builtin_clzll((uint32_t)x) - 32, cpu_bit_clz32(x))

/** @This returns the number of leading zero bits in a 64 bits
    value. The result is undefined if the input is 0. @csee #bit_clz_unsafe */
#define bit_clz64(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_clzll(x), cpu_bit_clz64(x))

/** @This returns the index of the most significant set bit in an
    integer value. The index of the rightmost bit is 0.

    This is the truncated log2 of an integer value. The result is
    undefined if the input is 0.  @see #bit_clz_unsafe @see
    #bit_lsb_index */
#define bit_msb_index(x) __builtin_choose_expr(sizeof(x) == 1, 7 - bit_clz8(x),   \
                    __builtin_choose_expr(sizeof(x) == 2, 15 - bit_clz16(x), \
                    __builtin_choose_expr(sizeof(x) == 4, 31 - bit_clz32(x), \
                                                          63 - bit_clz64(x))))

/** @This returns the number of leading zero bits in an integer
    value. The result is undefined if the input is 0.

    Because the results depends on the width of the integer type, it
    is less error prone to rely on @ref #bit_msb_index when possible.
*/
#define bit_clz_unsafe(x) __builtin_choose_expr(sizeof(x) == 1, bit_clz8(x),   \
                    __builtin_choose_expr(sizeof(x) == 2, bit_clz16(x), \
                    __builtin_choose_expr(sizeof(x) == 4, bit_clz32(x), \
                                                          bit_clz64(x))))

#ifndef HAS_CPU_BIT_CTZ64
# if defined(HAS_CPU_BIT_CLZ64) || defined(HAS_CPU_BIT_CLZ32)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_ctz64(uint64_t _x)
{
  return 63 - cpu_bit_clz64(_x & ~(_x - 1));
}
# elif defined(HAS_CPU_BIT_CTZ32)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_ctz64(uint64_t _x)
{
  return (uint32_t)_x
    ? cpu_bit_ctz32(_x)
    : cpu_bit_ctz32(_x >> 32) + 32;
}
# else
/** @internal */
#  define cpu_bit_ctz64(x) __bit_ctz64(x)
# endif
#endif

#ifndef HAS_CPU_BIT_CTZ32
# if defined(HAS_CPU_BIT_CTZ64)
/** @internal */
#  define cpu_bit_ctz32(x) cpu_bit_ctz64(x)
# elif defined(HAS_CPU_BIT_CLZ32) || defined(HAS_CPU_BIT_CLZ64)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_ctz32(uint32_t _x)
{
  return 31 - cpu_bit_clz32(_x & ~(_x - 1));
}
# else
/** @internal */
#  define cpu_bit_ctz32(x) __bit_ctz32(x)
# endif
#endif

#ifndef HAS_CPU_BIT_CTZ16
# if defined(HAS_CPU_BIT_CTZ32)
/** @internal */
#  define cpu_bit_ctz16(x) cpu_bit_ctz32(x)
# elif defined(HAS_CPU_BIT_CTZ64)
/** @internal */
#  define cpu_bit_ctz16(x) cpu_bit_ctz64(x)
# elif defined(HAS_CPU_BIT_CLZ16) || defined(HAS_CPU_BIT_CLZ32) || \
  defined(HAS_CPU_BIT_CLZ64)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_ctz16(uint16_t _x)
{
  return 15 - cpu_bit_clz16(_x & ~(_x - 1));
}
# else
/** @internal */
#  define cpu_bit_ctz16(x) __bit_ctz16(x)
# endif
#endif

#ifndef HAS_CPU_BIT_CTZ8
# if defined(HAS_CPU_BIT_CTZ16)
/** @internal */
#  define cpu_bit_ctz8(x) cpu_bit_ctz16(x)
# elif defined(HAS_CPU_BIT_CTZ32)
/** @internal */
#  define cpu_bit_ctz8(x) cpu_bit_ctz32(x)
# elif defined(HAS_CPU_BIT_CTZ64)
/** @internal */
#  define cpu_bit_ctz8(x) cpu_bit_ctz64(x)
# elif defined(HAS_CPU_BIT_CLZ8) || defined(HAS_CPU_BIT_CLZ16) || \
  defined(HAS_CPU_BIT_CLZ32) || defined(HAS_CPU_BIT_CLZ64)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_ctz8(uint8_t _x)
{
  return 7 - cpu_bit_clz8(_x & ~(_x - 1));
}
# else
/** @internal */
#  define cpu_bit_ctz8(x) __bit_ctz8(x)
# endif
#endif

/** @This returns the number of trailing zero bits in a 8 bits
    value. The result is undefined if the input is 0. */
#define bit_ctz8(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                            __builtin_ctzll(x), cpu_bit_ctz8(x))

/** @This returns the number of trailing zero bits in a 16 bits
    value. The result is undefined if the input is 0. */
#define bit_ctz16(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_ctzll(x), cpu_bit_ctz16(x))

/** @This returns the number of trailing zero bits in a 32 bits
    value. The result is undefined if the input is 0. */
#define bit_ctz32(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                            __builtin_ctzll(x), cpu_bit_ctz32(x))

/** @This returns the number of trailing zero bits in a 64 bits
    value. The result is undefined if the input is 0. */
#define bit_ctz64(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_ctzll(x), cpu_bit_ctz64(x))

/** @This returns the number of trailing zero bits in an integer
    value. The result is undefined if the input is 0.
    @see #bit_lsb_index */
#define bit_ctz(x) __builtin_choose_expr(sizeof(x) == 1, bit_ctz8(x),   \
                    __builtin_choose_expr(sizeof(x) == 2, bit_ctz16(x), \
                    __builtin_choose_expr(sizeof(x) == 4, bit_ctz32(x), \
                                                          bit_ctz64(x))))

/** @This returns the index of the least significant set bit in an
    intger value. The index of the rightmost bit is 0. The result is
    undefined if the input is 0.

    This is equivalent to @ref #bit_ctz */
#define bit_lsb_index(x) bit_ctz(x)

/** @internal */
ALWAYS_INLINE uint_fast8_t __bit_ffs8(uint8_t x)
{
  return x ? 1 + cpu_bit_ctz8(x) : 0;
}

/** @internal */
ALWAYS_INLINE uint_fast8_t __bit_ffs16(uint16_t x)
{
  return x ? 1 + cpu_bit_ctz16(x) : 0;
}

/** @internal */
ALWAYS_INLINE uint_fast8_t __bit_ffs32(uint32_t x)
{
  return x ? 1 + cpu_bit_ctz32(x) : 0;
}

/** @internal */
ALWAYS_INLINE uint_fast8_t __bit_ffs64(uint64_t x)
{
  return x ? 1 + cpu_bit_ctz64(x) : 0;
}

/** @This returns the index of the least significant set bit in a
    8 bits value. The index of the rightmost bit is 1. The result is
    0 if the input is 0. */
#define bit_ffs8(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                            __builtin_ffsll(x), __bit_ffs8(x))

/** @This returns the index of the least significant set bit in a
    16 bits value. The index of the rightmost bit is 1. The result is
    0 if the input is 0. */
#define bit_ffs16(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_ffsll(x), __bit_ffs16(x))

/** @This returns the index of the least significant set bit in a
    32 bits value. The index of the rightmost bit is 1. The result is
    0 if the input is 0. */
#define bit_ffs32(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                            __builtin_ffsll(x), __bit_ffs32(x))

/** @This returns the index of the least significant set bit in a
    64 bits value. The index of the rightmost bit is 1. The result is
    0 if the input is 0. */
#define bit_ffs64(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_ffsll(x), __bit_ffs64(x))

/** @This returns the index of the least significant set bit in an
    integer value. The index of the rightmost bit is 1. The result is
    0 if the input is 0. */
#define bit_ffs(x)  __builtin_choose_expr(sizeof(x) == 1, bit_ffs8(x),   \
                    __builtin_choose_expr(sizeof(x) == 2, bit_ffs16(x), \
                    __builtin_choose_expr(sizeof(x) == 4, bit_ffs32(x), \
                                                          bit_ffs64(x))))

#ifndef HAS_CPU_BIT_POPC64
# if defined(HAS_CPU_BIT_POPC32)
/** @internal */
ALWAYS_INLINE uint_fast8_t cpu_bit_popc64(uint64_t _x)
{
  return cpu_bit_popc32(_x) + cpu_bit_popc32(_x >> 32);
}
# else
/** @internal */
#  define cpu_bit_popc64(x) __bit_popc64(x)
# endif
#endif

#ifndef HAS_CPU_BIT_POPC32
# if defined(HAS_CPU_BIT_POPC64)
/** @internal */
#  define cpu_bit_popc32(x) cpu_bit_popc64(x)
# else
/** @internal */
#  define cpu_bit_popc32(x) __bit_popc32(x)
# endif
#endif

#ifndef HAS_CPU_BIT_POPC16
# if defined(HAS_CPU_BIT_POPC32)
/** @internal */
#  define cpu_bit_popc16(x) cpu_bit_popc32(x)
# elif defined(HAS_CPU_BIT_POPC64)
/** @internal */
#  define cpu_bit_popc16(x) cpu_bit_popc64(x)
# else
/** @internal */
#  define cpu_bit_popc16(x) __bit_popc16(x)
# endif
#endif

#ifndef HAS_CPU_BIT_POPC8
# if defined(HAS_CPU_BIT_POPC16)
/** @internal */
#  define cpu_bit_popc8(x) cpu_bit_popc16(x)
# elif defined(HAS_CPU_BIT_POPC32)
/** @internal */
#  define cpu_bit_popc8(x) cpu_bit_popc32(x)
# elif defined(HAS_CPU_BIT_POPC64)
/** @internal */
#  define cpu_bit_popc8(x) cpu_bit_popc64(x)
# else
/** @internal */
#  define cpu_bit_popc8(x) __bit_popc8(x)
# endif
#endif

/** @This returns the number of bits set in a 8 bits value. */
#define bit_popc8(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                           __builtin_popcountll((uint8_t)x), cpu_bit_popc8(x))

/** @This returns the number of bits set in a 16 bits value. */
#define bit_popc16(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_popcountll((uint16_t)x), cpu_bit_popc16(x))

/** @This returns the number of bits set in a 32 bits value. */
#define bit_popc32(x) __builtin_choose_expr(__builtin_constant_p((x)), \
                                            __builtin_popcountll((uint32_t)x), cpu_bit_popc32(x))

/** @This returns the number of bits set in a 64 bits value. */
#define bit_popc64(x) __builtin_choose_expr(__builtin_constant_p((x)),  \
                                            __builtin_popcountll((uint64_t)x), cpu_bit_popc64(x))

/** @This returns the number of bits set in an integer value. */
#define bit_popc(x) __builtin_choose_expr(sizeof(x) == 1, bit_popc8(x),   \
                    __builtin_choose_expr(sizeof(x) == 2, bit_popc16(x), \
                    __builtin_choose_expr(sizeof(x) == 4, bit_popc32(x), \
                                                          bit_popc64(x))))

/**
   @this sets a given bit index in a bit string. There is no
   boundary check in the bit string.
 */
ALWAYS_INLINE
void bitstring_set(uint8_t *bitstring, reg_t index)
{
  bitstring[index >> 3] |= (1 << (index & 7));
}

/**
   @this clears a given bit index in a bit string. There is no
   boundary check in the bit string.
 */

ALWAYS_INLINE
void bitstring_clear(uint8_t *bitstring, reg_t index)
{
  bitstring[index >> 3] &= ~(1 << (index & 7));
}

/**
   @this clears or set a given bit index in a bit string depending on
   argument. There is no boundary check in the bit string.
 */

ALWAYS_INLINE
void bitstring_set_value(uint8_t *bitstring, reg_t index, bool_t value)
{
  if (value)
    bitstring_set(bitstring, index);
  else
    bitstring_clear(bitstring, index);
}

/**
   @this retrieves a bit at a given bit index in a bit string.
 */
ALWAYS_INLINE
bool_t bitstring_get(const uint8_t *bitstring, reg_t index)
{
  return bit_get(bitstring[index >> 3], index & 7);
}

/**
   @this sets at most 32 unaligned bits in a bit string. There is no
   boundary check in the bit string.
 */
void bitstring_set32(uint8_t *bitstring, reg_t index, reg_t width, uint32_t value);

/**
   @this retrieves at most 32 unaligned bits in a bit string. There is no
   boundary check in the bit string.
 */
uint32_t bitstring_get32(const uint8_t *bitstring, reg_t index, reg_t width);

C_HEADER_END

#endif
