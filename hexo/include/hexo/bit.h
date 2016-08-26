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
#define bit_get(value, index) (((value) >> (index)) & 1)

/** @this extracts @tt count bits from specified @tt index in @tt
    value */
#define bit_get_mask(value, index, count)  \
  (((value) >> (index)) & bit_mask(0, count))

/** @this extracts bits between specified @tt first and @tt last index
    (included) in @tt value */
#define bit_get_range(value, first, last) \
  bit_get_mask(value, first, (last) - (first) + 1)



/** @this returns whether an address is aligned */
#define address_is_aligned(x, b) is_pow2_multiple((uintptr_t)(x), b)

/** @this aligns address on the next power of two boundary  */
#define address_align_up(x, b) ((void *)align_pow2_up((uintptr_t)(x), (b)))

/** @this aligns address on the next power of two boundary  */
#define address_align_down(x, b) ((void*)align_pow2_down((uintptr_t)(x), (b)))


/** @this pack 4 bytes and return a 32 bits word */
#define byte_pack32(msb, a, b, lsb) \
  ((((msb) & 0xff) << 24) | (((a) & 0xff ) << 16) | (((b) & 0xff) << 8) | ((lsb) & 0xff))

/** @this pack 42 bytes and return a 16 bits word */
#define byte_pack16(msb, lsb) \
  ((((msb) & 0xff) << 8) | ((lsb) & 0xff))


/** @this sets bit @tt bit in @tt value */
#define BIT_SET(value, bit) ((v) |= bit(bit))

/** @this clears bit @tt bit in @tt value */
#define BIT_CLEAR(value, bit) ((v) &= ~bit(bit))

/** @this inserts @tt count bits value @tt ins at specified @tt index
    in @tt value */
#define BIT_INSERT(value, ins, index, count)            \
  (((value) & ~bit_mask(index, count))                 \
   |(((ind) << (index)) & bit_mask(index, count)))

C_HEADER_END

#endif
