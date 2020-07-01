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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2020

*/

#ifndef MUTEK_LUT_H_
#define MUTEK_LUT_H_

/** @file
    @module {Core::Kernel services}
    @short Inline lookup tables
    @index {Inline lookup tables} {Kernel services}

    This header file provides a clean way to define and use small
    compile-time lookup tables. The LUTs are stored in a
    single integer and accessed using bitwise operations.

    The lookup table can be defined either in order or with a
    designated initialization syntax:

@code
  // a 64 bits LUT of at most 16 entries of 4 bits each
  // with implicit indices
  lut_16_4_t fibo = XLUT_16_4_DEF(0, 1, 1, 2, 3, 5, 8, 13);

  // a 32 bits LUT of at most 8 entries of 4 bits each
  // with explicit indices
  lut_8_4_t square = DLUT_8_4_DEF(0,  1,
				  1,  1,
				  2,  4,
				  3,  9);
@end code

  Access to the LUT is performed by a specific macro:

@code
  int_fast8_t sq3 = LUT_8_4_GET(3, square);
@end code

  Alternatively, LUT definition and access can be performed at the same time:
@code
  bool_t x_is_prime = XLUT_32_1(x, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0);
@end code
*/

#include <stdint.h>

/** @internal */
typedef char _LUT_out_of_range;

/** @internal */
#define _LUT_RANGE(x, b) (x + sizeof(_LUT_out_of_range[-(x > b)]))


typedef uint32_t lut_2_16_t;

/** @internal */
#define DLUT_2_16_DEF_(i0, v0, i1, v1, ...) (\
    (lut_2_16_t)_LUT_RANGE(v0, 65535) << (_LUT_RANGE(i0, 1) * 16) | \
    (lut_2_16_t)_LUT_RANGE(v1, 65535) << (_LUT_RANGE(i1, 1) * 16))

/** @internal */
#define XLUT_2_16_DEF_(v0, v1, ...) (\
    (lut_2_16_t)_LUT_RANGE(v0, 65535) << 0 | \
    (lut_2_16_t)_LUT_RANGE(v1, 65535) << 16)

/** @This computes an integer constant usable as a shift based lookup
    table of 2 entries of 16 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_2_16_DEF(...) DLUT_2_16_DEF_(__VA_ARGS__, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 2 entries of 16 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_2_16_DEF(...) XLUT_2_16_DEF_(__VA_ARGS__, 0)

/** @This accesses a shift based lookup table of 2 entries of 16 bits each. */
#define LUT_2_16_GET(x, lut) (((lut) >> ((x) * 16)) & 65535)

/** @This defines a shift based lookup table of 2
    entries of 16 bits each and accesses it.
    @csee #DLUT_2_16_DEF @csee #DLUT_2_16_GET */
#define DLUT_2_16(x, ...) LUT_2_16_GET(x, DLUT_2_16_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 2
    entries of 16 bits each and accesses it.
    @csee #XLUT_2_16_DEF @csee #XLUT_2_16_GET */
#define XLUT_2_16(x, ...) LUT_2_16_GET(x, XLUT_2_16_DEF(__VA_ARGS__))

typedef uint32_t lut_4_8_t;

/** @internal */
#define DLUT_4_8_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, ...) (\
    (lut_4_8_t)_LUT_RANGE(v0, 255) << (_LUT_RANGE(i0, 3) * 8) | \
    (lut_4_8_t)_LUT_RANGE(v1, 255) << (_LUT_RANGE(i1, 3) * 8) | \
    (lut_4_8_t)_LUT_RANGE(v2, 255) << (_LUT_RANGE(i2, 3) * 8) | \
    (lut_4_8_t)_LUT_RANGE(v3, 255) << (_LUT_RANGE(i3, 3) * 8))

/** @internal */
#define XLUT_4_8_DEF_(v0, v1, v2, v3, ...) (\
    (lut_4_8_t)_LUT_RANGE(v0, 255) << 0 | \
    (lut_4_8_t)_LUT_RANGE(v1, 255) << 8 | \
    (lut_4_8_t)_LUT_RANGE(v2, 255) << 16 | \
    (lut_4_8_t)_LUT_RANGE(v3, 255) << 24)

/** @This computes an integer constant usable as a shift based lookup
    table of 4 entries of 8 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_4_8_DEF(...) DLUT_4_8_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 4 entries of 8 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_4_8_DEF(...) XLUT_4_8_DEF_(__VA_ARGS__, 0, 0, 0)

/** @This accesses a shift based lookup table of 4 entries of 8 bits each. */
#define LUT_4_8_GET(x, lut) (((lut) >> ((x) * 8)) & 255)

/** @This defines a shift based lookup table of 4
    entries of 8 bits each and accesses it.
    @csee #DLUT_4_8_DEF @csee #DLUT_4_8_GET */
#define DLUT_4_8(x, ...) LUT_4_8_GET(x, DLUT_4_8_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 4
    entries of 8 bits each and accesses it.
    @csee #XLUT_4_8_DEF @csee #XLUT_4_8_GET */
#define XLUT_4_8(x, ...) LUT_4_8_GET(x, XLUT_4_8_DEF(__VA_ARGS__))

typedef uint32_t lut_8_4_t;

/** @internal */
#define DLUT_8_4_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, ...) (\
    (lut_8_4_t)_LUT_RANGE(v0, 15) << (_LUT_RANGE(i0, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v1, 15) << (_LUT_RANGE(i1, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v2, 15) << (_LUT_RANGE(i2, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v3, 15) << (_LUT_RANGE(i3, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v4, 15) << (_LUT_RANGE(i4, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v5, 15) << (_LUT_RANGE(i5, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v6, 15) << (_LUT_RANGE(i6, 7) * 4) | \
    (lut_8_4_t)_LUT_RANGE(v7, 15) << (_LUT_RANGE(i7, 7) * 4))

/** @internal */
#define XLUT_8_4_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, ...) (\
    (lut_8_4_t)_LUT_RANGE(v0, 15) << 0 | \
    (lut_8_4_t)_LUT_RANGE(v1, 15) << 4 | \
    (lut_8_4_t)_LUT_RANGE(v2, 15) << 8 | \
    (lut_8_4_t)_LUT_RANGE(v3, 15) << 12 | \
    (lut_8_4_t)_LUT_RANGE(v4, 15) << 16 | \
    (lut_8_4_t)_LUT_RANGE(v5, 15) << 20 | \
    (lut_8_4_t)_LUT_RANGE(v6, 15) << 24 | \
    (lut_8_4_t)_LUT_RANGE(v7, 15) << 28)

/** @This computes an integer constant usable as a shift based lookup
    table of 8 entries of 4 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_8_4_DEF(...) DLUT_8_4_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 8 entries of 4 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_8_4_DEF(...) XLUT_8_4_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 8 entries of 4 bits each. */
#define LUT_8_4_GET(x, lut) (((lut) >> ((x) * 4)) & 15)

/** @This defines a shift based lookup table of 8
    entries of 4 bits each and accesses it.
    @csee #DLUT_8_4_DEF @csee #DLUT_8_4_GET */
#define DLUT_8_4(x, ...) LUT_8_4_GET(x, DLUT_8_4_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 8
    entries of 4 bits each and accesses it.
    @csee #XLUT_8_4_DEF @csee #XLUT_8_4_GET */
#define XLUT_8_4(x, ...) LUT_8_4_GET(x, XLUT_8_4_DEF(__VA_ARGS__))

typedef uint32_t lut_16_2_t;

/** @internal */
#define DLUT_16_2_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, i8, v8, i9, v9, i10, v10, i11, v11, i12, v12, i13, v13, i14, v14, i15, v15, ...) (\
    (lut_16_2_t)_LUT_RANGE(v0, 3) << (_LUT_RANGE(i0, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v1, 3) << (_LUT_RANGE(i1, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v2, 3) << (_LUT_RANGE(i2, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v3, 3) << (_LUT_RANGE(i3, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v4, 3) << (_LUT_RANGE(i4, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v5, 3) << (_LUT_RANGE(i5, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v6, 3) << (_LUT_RANGE(i6, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v7, 3) << (_LUT_RANGE(i7, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v8, 3) << (_LUT_RANGE(i8, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v9, 3) << (_LUT_RANGE(i9, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v10, 3) << (_LUT_RANGE(i10, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v11, 3) << (_LUT_RANGE(i11, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v12, 3) << (_LUT_RANGE(i12, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v13, 3) << (_LUT_RANGE(i13, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v14, 3) << (_LUT_RANGE(i14, 15) * 2) | \
    (lut_16_2_t)_LUT_RANGE(v15, 3) << (_LUT_RANGE(i15, 15) * 2))

/** @internal */
#define XLUT_16_2_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, ...) (\
    (lut_16_2_t)_LUT_RANGE(v0, 3) << 0 | \
    (lut_16_2_t)_LUT_RANGE(v1, 3) << 2 | \
    (lut_16_2_t)_LUT_RANGE(v2, 3) << 4 | \
    (lut_16_2_t)_LUT_RANGE(v3, 3) << 6 | \
    (lut_16_2_t)_LUT_RANGE(v4, 3) << 8 | \
    (lut_16_2_t)_LUT_RANGE(v5, 3) << 10 | \
    (lut_16_2_t)_LUT_RANGE(v6, 3) << 12 | \
    (lut_16_2_t)_LUT_RANGE(v7, 3) << 14 | \
    (lut_16_2_t)_LUT_RANGE(v8, 3) << 16 | \
    (lut_16_2_t)_LUT_RANGE(v9, 3) << 18 | \
    (lut_16_2_t)_LUT_RANGE(v10, 3) << 20 | \
    (lut_16_2_t)_LUT_RANGE(v11, 3) << 22 | \
    (lut_16_2_t)_LUT_RANGE(v12, 3) << 24 | \
    (lut_16_2_t)_LUT_RANGE(v13, 3) << 26 | \
    (lut_16_2_t)_LUT_RANGE(v14, 3) << 28 | \
    (lut_16_2_t)_LUT_RANGE(v15, 3) << 30)

/** @This computes an integer constant usable as a shift based lookup
    table of 16 entries of 2 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_16_2_DEF(...) DLUT_16_2_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 16 entries of 2 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_16_2_DEF(...) XLUT_16_2_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 16 entries of 2 bits each. */
#define LUT_16_2_GET(x, lut) (((lut) >> ((x) * 2)) & 3)

/** @This defines a shift based lookup table of 16
    entries of 2 bits each and accesses it.
    @csee #DLUT_16_2_DEF @csee #DLUT_16_2_GET */
#define DLUT_16_2(x, ...) LUT_16_2_GET(x, DLUT_16_2_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 16
    entries of 2 bits each and accesses it.
    @csee #XLUT_16_2_DEF @csee #XLUT_16_2_GET */
#define XLUT_16_2(x, ...) LUT_16_2_GET(x, XLUT_16_2_DEF(__VA_ARGS__))

typedef uint32_t lut_32_1_t;

/** @internal */
#define DLUT_32_1_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, i8, v8, i9, v9, i10, v10, i11, v11, i12, v12, i13, v13, i14, v14, i15, v15, i16, v16, i17, v17, i18, v18, i19, v19, i20, v20, i21, v21, i22, v22, i23, v23, i24, v24, i25, v25, i26, v26, i27, v27, i28, v28, i29, v29, i30, v30, i31, v31, ...) (\
    (lut_32_1_t)_LUT_RANGE(v0, 1) << (_LUT_RANGE(i0, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v1, 1) << (_LUT_RANGE(i1, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v2, 1) << (_LUT_RANGE(i2, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v3, 1) << (_LUT_RANGE(i3, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v4, 1) << (_LUT_RANGE(i4, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v5, 1) << (_LUT_RANGE(i5, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v6, 1) << (_LUT_RANGE(i6, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v7, 1) << (_LUT_RANGE(i7, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v8, 1) << (_LUT_RANGE(i8, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v9, 1) << (_LUT_RANGE(i9, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v10, 1) << (_LUT_RANGE(i10, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v11, 1) << (_LUT_RANGE(i11, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v12, 1) << (_LUT_RANGE(i12, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v13, 1) << (_LUT_RANGE(i13, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v14, 1) << (_LUT_RANGE(i14, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v15, 1) << (_LUT_RANGE(i15, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v16, 1) << (_LUT_RANGE(i16, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v17, 1) << (_LUT_RANGE(i17, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v18, 1) << (_LUT_RANGE(i18, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v19, 1) << (_LUT_RANGE(i19, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v20, 1) << (_LUT_RANGE(i20, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v21, 1) << (_LUT_RANGE(i21, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v22, 1) << (_LUT_RANGE(i22, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v23, 1) << (_LUT_RANGE(i23, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v24, 1) << (_LUT_RANGE(i24, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v25, 1) << (_LUT_RANGE(i25, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v26, 1) << (_LUT_RANGE(i26, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v27, 1) << (_LUT_RANGE(i27, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v28, 1) << (_LUT_RANGE(i28, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v29, 1) << (_LUT_RANGE(i29, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v30, 1) << (_LUT_RANGE(i30, 31) * 1) | \
    (lut_32_1_t)_LUT_RANGE(v31, 1) << (_LUT_RANGE(i31, 31) * 1))

/** @internal */
#define XLUT_32_1_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, ...) (\
    (lut_32_1_t)_LUT_RANGE(v0, 1) << 0 | \
    (lut_32_1_t)_LUT_RANGE(v1, 1) << 1 | \
    (lut_32_1_t)_LUT_RANGE(v2, 1) << 2 | \
    (lut_32_1_t)_LUT_RANGE(v3, 1) << 3 | \
    (lut_32_1_t)_LUT_RANGE(v4, 1) << 4 | \
    (lut_32_1_t)_LUT_RANGE(v5, 1) << 5 | \
    (lut_32_1_t)_LUT_RANGE(v6, 1) << 6 | \
    (lut_32_1_t)_LUT_RANGE(v7, 1) << 7 | \
    (lut_32_1_t)_LUT_RANGE(v8, 1) << 8 | \
    (lut_32_1_t)_LUT_RANGE(v9, 1) << 9 | \
    (lut_32_1_t)_LUT_RANGE(v10, 1) << 10 | \
    (lut_32_1_t)_LUT_RANGE(v11, 1) << 11 | \
    (lut_32_1_t)_LUT_RANGE(v12, 1) << 12 | \
    (lut_32_1_t)_LUT_RANGE(v13, 1) << 13 | \
    (lut_32_1_t)_LUT_RANGE(v14, 1) << 14 | \
    (lut_32_1_t)_LUT_RANGE(v15, 1) << 15 | \
    (lut_32_1_t)_LUT_RANGE(v16, 1) << 16 | \
    (lut_32_1_t)_LUT_RANGE(v17, 1) << 17 | \
    (lut_32_1_t)_LUT_RANGE(v18, 1) << 18 | \
    (lut_32_1_t)_LUT_RANGE(v19, 1) << 19 | \
    (lut_32_1_t)_LUT_RANGE(v20, 1) << 20 | \
    (lut_32_1_t)_LUT_RANGE(v21, 1) << 21 | \
    (lut_32_1_t)_LUT_RANGE(v22, 1) << 22 | \
    (lut_32_1_t)_LUT_RANGE(v23, 1) << 23 | \
    (lut_32_1_t)_LUT_RANGE(v24, 1) << 24 | \
    (lut_32_1_t)_LUT_RANGE(v25, 1) << 25 | \
    (lut_32_1_t)_LUT_RANGE(v26, 1) << 26 | \
    (lut_32_1_t)_LUT_RANGE(v27, 1) << 27 | \
    (lut_32_1_t)_LUT_RANGE(v28, 1) << 28 | \
    (lut_32_1_t)_LUT_RANGE(v29, 1) << 29 | \
    (lut_32_1_t)_LUT_RANGE(v30, 1) << 30 | \
    (lut_32_1_t)_LUT_RANGE(v31, 1) << 31)

/** @This computes an integer constant usable as a shift based lookup
    table of 32 entries of 1 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_32_1_DEF(...) DLUT_32_1_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 32 entries of 1 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_32_1_DEF(...) XLUT_32_1_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 32 entries of 1 bits each. */
#define LUT_32_1_GET(x, lut) (((lut) >> ((x) * 1)) & 1)

/** @This defines a shift based lookup table of 32
    entries of 1 bits each and accesses it.
    @csee #DLUT_32_1_DEF @csee #DLUT_32_1_GET */
#define DLUT_32_1(x, ...) LUT_32_1_GET(x, DLUT_32_1_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 32
    entries of 1 bits each and accesses it.
    @csee #XLUT_32_1_DEF @csee #XLUT_32_1_GET */
#define XLUT_32_1(x, ...) LUT_32_1_GET(x, XLUT_32_1_DEF(__VA_ARGS__))

typedef uint64_t lut_2_32_t;

/** @internal */
#define DLUT_2_32_DEF_(i0, v0, i1, v1, ...) (\
    (lut_2_32_t)_LUT_RANGE(v0, 4294967295) << (_LUT_RANGE(i0, 1) * 32) | \
    (lut_2_32_t)_LUT_RANGE(v1, 4294967295) << (_LUT_RANGE(i1, 1) * 32))

/** @internal */
#define XLUT_2_32_DEF_(v0, v1, ...) (\
    (lut_2_32_t)_LUT_RANGE(v0, 4294967295) << 0 | \
    (lut_2_32_t)_LUT_RANGE(v1, 4294967295) << 32)

/** @This computes an integer constant usable as a shift based lookup
    table of 2 entries of 32 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_2_32_DEF(...) DLUT_2_32_DEF_(__VA_ARGS__, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 2 entries of 32 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_2_32_DEF(...) XLUT_2_32_DEF_(__VA_ARGS__, 0)

/** @This accesses a shift based lookup table of 2 entries of 32 bits each. */
#define LUT_2_32_GET(x, lut) (((lut) >> ((x) * 32)) & 4294967295)

/** @This defines a shift based lookup table of 2
    entries of 32 bits each and accesses it.
    @csee #DLUT_2_32_DEF @csee #DLUT_2_32_GET */
#define DLUT_2_32(x, ...) LUT_2_32_GET(x, DLUT_2_32_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 2
    entries of 32 bits each and accesses it.
    @csee #XLUT_2_32_DEF @csee #XLUT_2_32_GET */
#define XLUT_2_32(x, ...) LUT_2_32_GET(x, XLUT_2_32_DEF(__VA_ARGS__))

typedef uint64_t lut_4_16_t;

/** @internal */
#define DLUT_4_16_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, ...) (\
    (lut_4_16_t)_LUT_RANGE(v0, 65535) << (_LUT_RANGE(i0, 3) * 16) | \
    (lut_4_16_t)_LUT_RANGE(v1, 65535) << (_LUT_RANGE(i1, 3) * 16) | \
    (lut_4_16_t)_LUT_RANGE(v2, 65535) << (_LUT_RANGE(i2, 3) * 16) | \
    (lut_4_16_t)_LUT_RANGE(v3, 65535) << (_LUT_RANGE(i3, 3) * 16))

/** @internal */
#define XLUT_4_16_DEF_(v0, v1, v2, v3, ...) (\
    (lut_4_16_t)_LUT_RANGE(v0, 65535) << 0 | \
    (lut_4_16_t)_LUT_RANGE(v1, 65535) << 16 | \
    (lut_4_16_t)_LUT_RANGE(v2, 65535) << 32 | \
    (lut_4_16_t)_LUT_RANGE(v3, 65535) << 48)

/** @This computes an integer constant usable as a shift based lookup
    table of 4 entries of 16 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_4_16_DEF(...) DLUT_4_16_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 4 entries of 16 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_4_16_DEF(...) XLUT_4_16_DEF_(__VA_ARGS__, 0, 0, 0)

/** @This accesses a shift based lookup table of 4 entries of 16 bits each. */
#define LUT_4_16_GET(x, lut) (((lut) >> ((x) * 16)) & 65535)

/** @This defines a shift based lookup table of 4
    entries of 16 bits each and accesses it.
    @csee #DLUT_4_16_DEF @csee #DLUT_4_16_GET */
#define DLUT_4_16(x, ...) LUT_4_16_GET(x, DLUT_4_16_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 4
    entries of 16 bits each and accesses it.
    @csee #XLUT_4_16_DEF @csee #XLUT_4_16_GET */
#define XLUT_4_16(x, ...) LUT_4_16_GET(x, XLUT_4_16_DEF(__VA_ARGS__))

typedef uint64_t lut_8_8_t;

/** @internal */
#define DLUT_8_8_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, ...) (\
    (lut_8_8_t)_LUT_RANGE(v0, 255) << (_LUT_RANGE(i0, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v1, 255) << (_LUT_RANGE(i1, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v2, 255) << (_LUT_RANGE(i2, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v3, 255) << (_LUT_RANGE(i3, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v4, 255) << (_LUT_RANGE(i4, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v5, 255) << (_LUT_RANGE(i5, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v6, 255) << (_LUT_RANGE(i6, 7) * 8) | \
    (lut_8_8_t)_LUT_RANGE(v7, 255) << (_LUT_RANGE(i7, 7) * 8))

/** @internal */
#define XLUT_8_8_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, ...) (\
    (lut_8_8_t)_LUT_RANGE(v0, 255) << 0 | \
    (lut_8_8_t)_LUT_RANGE(v1, 255) << 8 | \
    (lut_8_8_t)_LUT_RANGE(v2, 255) << 16 | \
    (lut_8_8_t)_LUT_RANGE(v3, 255) << 24 | \
    (lut_8_8_t)_LUT_RANGE(v4, 255) << 32 | \
    (lut_8_8_t)_LUT_RANGE(v5, 255) << 40 | \
    (lut_8_8_t)_LUT_RANGE(v6, 255) << 48 | \
    (lut_8_8_t)_LUT_RANGE(v7, 255) << 56)

/** @This computes an integer constant usable as a shift based lookup
    table of 8 entries of 8 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_8_8_DEF(...) DLUT_8_8_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 8 entries of 8 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_8_8_DEF(...) XLUT_8_8_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 8 entries of 8 bits each. */
#define LUT_8_8_GET(x, lut) (((lut) >> ((x) * 8)) & 255)

/** @This defines a shift based lookup table of 8
    entries of 8 bits each and accesses it.
    @csee #DLUT_8_8_DEF @csee #DLUT_8_8_GET */
#define DLUT_8_8(x, ...) LUT_8_8_GET(x, DLUT_8_8_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 8
    entries of 8 bits each and accesses it.
    @csee #XLUT_8_8_DEF @csee #XLUT_8_8_GET */
#define XLUT_8_8(x, ...) LUT_8_8_GET(x, XLUT_8_8_DEF(__VA_ARGS__))

typedef uint64_t lut_16_4_t;

/** @internal */
#define DLUT_16_4_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, i8, v8, i9, v9, i10, v10, i11, v11, i12, v12, i13, v13, i14, v14, i15, v15, ...) (\
    (lut_16_4_t)_LUT_RANGE(v0, 15) << (_LUT_RANGE(i0, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v1, 15) << (_LUT_RANGE(i1, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v2, 15) << (_LUT_RANGE(i2, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v3, 15) << (_LUT_RANGE(i3, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v4, 15) << (_LUT_RANGE(i4, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v5, 15) << (_LUT_RANGE(i5, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v6, 15) << (_LUT_RANGE(i6, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v7, 15) << (_LUT_RANGE(i7, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v8, 15) << (_LUT_RANGE(i8, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v9, 15) << (_LUT_RANGE(i9, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v10, 15) << (_LUT_RANGE(i10, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v11, 15) << (_LUT_RANGE(i11, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v12, 15) << (_LUT_RANGE(i12, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v13, 15) << (_LUT_RANGE(i13, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v14, 15) << (_LUT_RANGE(i14, 15) * 4) | \
    (lut_16_4_t)_LUT_RANGE(v15, 15) << (_LUT_RANGE(i15, 15) * 4))

/** @internal */
#define XLUT_16_4_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, ...) (\
    (lut_16_4_t)_LUT_RANGE(v0, 15) << 0 | \
    (lut_16_4_t)_LUT_RANGE(v1, 15) << 4 | \
    (lut_16_4_t)_LUT_RANGE(v2, 15) << 8 | \
    (lut_16_4_t)_LUT_RANGE(v3, 15) << 12 | \
    (lut_16_4_t)_LUT_RANGE(v4, 15) << 16 | \
    (lut_16_4_t)_LUT_RANGE(v5, 15) << 20 | \
    (lut_16_4_t)_LUT_RANGE(v6, 15) << 24 | \
    (lut_16_4_t)_LUT_RANGE(v7, 15) << 28 | \
    (lut_16_4_t)_LUT_RANGE(v8, 15) << 32 | \
    (lut_16_4_t)_LUT_RANGE(v9, 15) << 36 | \
    (lut_16_4_t)_LUT_RANGE(v10, 15) << 40 | \
    (lut_16_4_t)_LUT_RANGE(v11, 15) << 44 | \
    (lut_16_4_t)_LUT_RANGE(v12, 15) << 48 | \
    (lut_16_4_t)_LUT_RANGE(v13, 15) << 52 | \
    (lut_16_4_t)_LUT_RANGE(v14, 15) << 56 | \
    (lut_16_4_t)_LUT_RANGE(v15, 15) << 60)

/** @This computes an integer constant usable as a shift based lookup
    table of 16 entries of 4 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_16_4_DEF(...) DLUT_16_4_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 16 entries of 4 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_16_4_DEF(...) XLUT_16_4_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 16 entries of 4 bits each. */
#define LUT_16_4_GET(x, lut) (((lut) >> ((x) * 4)) & 15)

/** @This defines a shift based lookup table of 16
    entries of 4 bits each and accesses it.
    @csee #DLUT_16_4_DEF @csee #DLUT_16_4_GET */
#define DLUT_16_4(x, ...) LUT_16_4_GET(x, DLUT_16_4_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 16
    entries of 4 bits each and accesses it.
    @csee #XLUT_16_4_DEF @csee #XLUT_16_4_GET */
#define XLUT_16_4(x, ...) LUT_16_4_GET(x, XLUT_16_4_DEF(__VA_ARGS__))

typedef uint64_t lut_32_2_t;

/** @internal */
#define DLUT_32_2_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, i8, v8, i9, v9, i10, v10, i11, v11, i12, v12, i13, v13, i14, v14, i15, v15, i16, v16, i17, v17, i18, v18, i19, v19, i20, v20, i21, v21, i22, v22, i23, v23, i24, v24, i25, v25, i26, v26, i27, v27, i28, v28, i29, v29, i30, v30, i31, v31, ...) (\
    (lut_32_2_t)_LUT_RANGE(v0, 3) << (_LUT_RANGE(i0, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v1, 3) << (_LUT_RANGE(i1, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v2, 3) << (_LUT_RANGE(i2, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v3, 3) << (_LUT_RANGE(i3, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v4, 3) << (_LUT_RANGE(i4, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v5, 3) << (_LUT_RANGE(i5, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v6, 3) << (_LUT_RANGE(i6, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v7, 3) << (_LUT_RANGE(i7, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v8, 3) << (_LUT_RANGE(i8, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v9, 3) << (_LUT_RANGE(i9, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v10, 3) << (_LUT_RANGE(i10, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v11, 3) << (_LUT_RANGE(i11, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v12, 3) << (_LUT_RANGE(i12, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v13, 3) << (_LUT_RANGE(i13, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v14, 3) << (_LUT_RANGE(i14, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v15, 3) << (_LUT_RANGE(i15, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v16, 3) << (_LUT_RANGE(i16, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v17, 3) << (_LUT_RANGE(i17, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v18, 3) << (_LUT_RANGE(i18, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v19, 3) << (_LUT_RANGE(i19, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v20, 3) << (_LUT_RANGE(i20, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v21, 3) << (_LUT_RANGE(i21, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v22, 3) << (_LUT_RANGE(i22, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v23, 3) << (_LUT_RANGE(i23, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v24, 3) << (_LUT_RANGE(i24, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v25, 3) << (_LUT_RANGE(i25, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v26, 3) << (_LUT_RANGE(i26, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v27, 3) << (_LUT_RANGE(i27, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v28, 3) << (_LUT_RANGE(i28, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v29, 3) << (_LUT_RANGE(i29, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v30, 3) << (_LUT_RANGE(i30, 31) * 2) | \
    (lut_32_2_t)_LUT_RANGE(v31, 3) << (_LUT_RANGE(i31, 31) * 2))

/** @internal */
#define XLUT_32_2_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, ...) (\
    (lut_32_2_t)_LUT_RANGE(v0, 3) << 0 | \
    (lut_32_2_t)_LUT_RANGE(v1, 3) << 2 | \
    (lut_32_2_t)_LUT_RANGE(v2, 3) << 4 | \
    (lut_32_2_t)_LUT_RANGE(v3, 3) << 6 | \
    (lut_32_2_t)_LUT_RANGE(v4, 3) << 8 | \
    (lut_32_2_t)_LUT_RANGE(v5, 3) << 10 | \
    (lut_32_2_t)_LUT_RANGE(v6, 3) << 12 | \
    (lut_32_2_t)_LUT_RANGE(v7, 3) << 14 | \
    (lut_32_2_t)_LUT_RANGE(v8, 3) << 16 | \
    (lut_32_2_t)_LUT_RANGE(v9, 3) << 18 | \
    (lut_32_2_t)_LUT_RANGE(v10, 3) << 20 | \
    (lut_32_2_t)_LUT_RANGE(v11, 3) << 22 | \
    (lut_32_2_t)_LUT_RANGE(v12, 3) << 24 | \
    (lut_32_2_t)_LUT_RANGE(v13, 3) << 26 | \
    (lut_32_2_t)_LUT_RANGE(v14, 3) << 28 | \
    (lut_32_2_t)_LUT_RANGE(v15, 3) << 30 | \
    (lut_32_2_t)_LUT_RANGE(v16, 3) << 32 | \
    (lut_32_2_t)_LUT_RANGE(v17, 3) << 34 | \
    (lut_32_2_t)_LUT_RANGE(v18, 3) << 36 | \
    (lut_32_2_t)_LUT_RANGE(v19, 3) << 38 | \
    (lut_32_2_t)_LUT_RANGE(v20, 3) << 40 | \
    (lut_32_2_t)_LUT_RANGE(v21, 3) << 42 | \
    (lut_32_2_t)_LUT_RANGE(v22, 3) << 44 | \
    (lut_32_2_t)_LUT_RANGE(v23, 3) << 46 | \
    (lut_32_2_t)_LUT_RANGE(v24, 3) << 48 | \
    (lut_32_2_t)_LUT_RANGE(v25, 3) << 50 | \
    (lut_32_2_t)_LUT_RANGE(v26, 3) << 52 | \
    (lut_32_2_t)_LUT_RANGE(v27, 3) << 54 | \
    (lut_32_2_t)_LUT_RANGE(v28, 3) << 56 | \
    (lut_32_2_t)_LUT_RANGE(v29, 3) << 58 | \
    (lut_32_2_t)_LUT_RANGE(v30, 3) << 60 | \
    (lut_32_2_t)_LUT_RANGE(v31, 3) << 62)

/** @This computes an integer constant usable as a shift based lookup
    table of 32 entries of 2 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_32_2_DEF(...) DLUT_32_2_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 32 entries of 2 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_32_2_DEF(...) XLUT_32_2_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 32 entries of 2 bits each. */
#define LUT_32_2_GET(x, lut) (((lut) >> ((x) * 2)) & 3)

/** @This defines a shift based lookup table of 32
    entries of 2 bits each and accesses it.
    @csee #DLUT_32_2_DEF @csee #DLUT_32_2_GET */
#define DLUT_32_2(x, ...) LUT_32_2_GET(x, DLUT_32_2_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 32
    entries of 2 bits each and accesses it.
    @csee #XLUT_32_2_DEF @csee #XLUT_32_2_GET */
#define XLUT_32_2(x, ...) LUT_32_2_GET(x, XLUT_32_2_DEF(__VA_ARGS__))

typedef uint64_t lut_64_1_t;

/** @internal */
#define DLUT_64_1_DEF_(i0, v0, i1, v1, i2, v2, i3, v3, i4, v4, i5, v5, i6, v6, i7, v7, i8, v8, i9, v9, i10, v10, i11, v11, i12, v12, i13, v13, i14, v14, i15, v15, i16, v16, i17, v17, i18, v18, i19, v19, i20, v20, i21, v21, i22, v22, i23, v23, i24, v24, i25, v25, i26, v26, i27, v27, i28, v28, i29, v29, i30, v30, i31, v31, i32, v32, i33, v33, i34, v34, i35, v35, i36, v36, i37, v37, i38, v38, i39, v39, i40, v40, i41, v41, i42, v42, i43, v43, i44, v44, i45, v45, i46, v46, i47, v47, i48, v48, i49, v49, i50, v50, i51, v51, i52, v52, i53, v53, i54, v54, i55, v55, i56, v56, i57, v57, i58, v58, i59, v59, i60, v60, i61, v61, i62, v62, i63, v63, ...) (\
    (lut_64_1_t)_LUT_RANGE(v0, 1) << (_LUT_RANGE(i0, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v1, 1) << (_LUT_RANGE(i1, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v2, 1) << (_LUT_RANGE(i2, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v3, 1) << (_LUT_RANGE(i3, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v4, 1) << (_LUT_RANGE(i4, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v5, 1) << (_LUT_RANGE(i5, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v6, 1) << (_LUT_RANGE(i6, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v7, 1) << (_LUT_RANGE(i7, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v8, 1) << (_LUT_RANGE(i8, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v9, 1) << (_LUT_RANGE(i9, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v10, 1) << (_LUT_RANGE(i10, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v11, 1) << (_LUT_RANGE(i11, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v12, 1) << (_LUT_RANGE(i12, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v13, 1) << (_LUT_RANGE(i13, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v14, 1) << (_LUT_RANGE(i14, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v15, 1) << (_LUT_RANGE(i15, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v16, 1) << (_LUT_RANGE(i16, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v17, 1) << (_LUT_RANGE(i17, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v18, 1) << (_LUT_RANGE(i18, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v19, 1) << (_LUT_RANGE(i19, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v20, 1) << (_LUT_RANGE(i20, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v21, 1) << (_LUT_RANGE(i21, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v22, 1) << (_LUT_RANGE(i22, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v23, 1) << (_LUT_RANGE(i23, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v24, 1) << (_LUT_RANGE(i24, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v25, 1) << (_LUT_RANGE(i25, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v26, 1) << (_LUT_RANGE(i26, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v27, 1) << (_LUT_RANGE(i27, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v28, 1) << (_LUT_RANGE(i28, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v29, 1) << (_LUT_RANGE(i29, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v30, 1) << (_LUT_RANGE(i30, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v31, 1) << (_LUT_RANGE(i31, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v32, 1) << (_LUT_RANGE(i32, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v33, 1) << (_LUT_RANGE(i33, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v34, 1) << (_LUT_RANGE(i34, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v35, 1) << (_LUT_RANGE(i35, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v36, 1) << (_LUT_RANGE(i36, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v37, 1) << (_LUT_RANGE(i37, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v38, 1) << (_LUT_RANGE(i38, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v39, 1) << (_LUT_RANGE(i39, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v40, 1) << (_LUT_RANGE(i40, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v41, 1) << (_LUT_RANGE(i41, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v42, 1) << (_LUT_RANGE(i42, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v43, 1) << (_LUT_RANGE(i43, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v44, 1) << (_LUT_RANGE(i44, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v45, 1) << (_LUT_RANGE(i45, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v46, 1) << (_LUT_RANGE(i46, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v47, 1) << (_LUT_RANGE(i47, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v48, 1) << (_LUT_RANGE(i48, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v49, 1) << (_LUT_RANGE(i49, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v50, 1) << (_LUT_RANGE(i50, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v51, 1) << (_LUT_RANGE(i51, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v52, 1) << (_LUT_RANGE(i52, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v53, 1) << (_LUT_RANGE(i53, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v54, 1) << (_LUT_RANGE(i54, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v55, 1) << (_LUT_RANGE(i55, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v56, 1) << (_LUT_RANGE(i56, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v57, 1) << (_LUT_RANGE(i57, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v58, 1) << (_LUT_RANGE(i58, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v59, 1) << (_LUT_RANGE(i59, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v60, 1) << (_LUT_RANGE(i60, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v61, 1) << (_LUT_RANGE(i61, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v62, 1) << (_LUT_RANGE(i62, 63) * 1) | \
    (lut_64_1_t)_LUT_RANGE(v63, 1) << (_LUT_RANGE(i63, 63) * 1))

/** @internal */
#define XLUT_64_1_DEF_(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61, v62, v63, ...) (\
    (lut_64_1_t)_LUT_RANGE(v0, 1) << 0 | \
    (lut_64_1_t)_LUT_RANGE(v1, 1) << 1 | \
    (lut_64_1_t)_LUT_RANGE(v2, 1) << 2 | \
    (lut_64_1_t)_LUT_RANGE(v3, 1) << 3 | \
    (lut_64_1_t)_LUT_RANGE(v4, 1) << 4 | \
    (lut_64_1_t)_LUT_RANGE(v5, 1) << 5 | \
    (lut_64_1_t)_LUT_RANGE(v6, 1) << 6 | \
    (lut_64_1_t)_LUT_RANGE(v7, 1) << 7 | \
    (lut_64_1_t)_LUT_RANGE(v8, 1) << 8 | \
    (lut_64_1_t)_LUT_RANGE(v9, 1) << 9 | \
    (lut_64_1_t)_LUT_RANGE(v10, 1) << 10 | \
    (lut_64_1_t)_LUT_RANGE(v11, 1) << 11 | \
    (lut_64_1_t)_LUT_RANGE(v12, 1) << 12 | \
    (lut_64_1_t)_LUT_RANGE(v13, 1) << 13 | \
    (lut_64_1_t)_LUT_RANGE(v14, 1) << 14 | \
    (lut_64_1_t)_LUT_RANGE(v15, 1) << 15 | \
    (lut_64_1_t)_LUT_RANGE(v16, 1) << 16 | \
    (lut_64_1_t)_LUT_RANGE(v17, 1) << 17 | \
    (lut_64_1_t)_LUT_RANGE(v18, 1) << 18 | \
    (lut_64_1_t)_LUT_RANGE(v19, 1) << 19 | \
    (lut_64_1_t)_LUT_RANGE(v20, 1) << 20 | \
    (lut_64_1_t)_LUT_RANGE(v21, 1) << 21 | \
    (lut_64_1_t)_LUT_RANGE(v22, 1) << 22 | \
    (lut_64_1_t)_LUT_RANGE(v23, 1) << 23 | \
    (lut_64_1_t)_LUT_RANGE(v24, 1) << 24 | \
    (lut_64_1_t)_LUT_RANGE(v25, 1) << 25 | \
    (lut_64_1_t)_LUT_RANGE(v26, 1) << 26 | \
    (lut_64_1_t)_LUT_RANGE(v27, 1) << 27 | \
    (lut_64_1_t)_LUT_RANGE(v28, 1) << 28 | \
    (lut_64_1_t)_LUT_RANGE(v29, 1) << 29 | \
    (lut_64_1_t)_LUT_RANGE(v30, 1) << 30 | \
    (lut_64_1_t)_LUT_RANGE(v31, 1) << 31 | \
    (lut_64_1_t)_LUT_RANGE(v32, 1) << 32 | \
    (lut_64_1_t)_LUT_RANGE(v33, 1) << 33 | \
    (lut_64_1_t)_LUT_RANGE(v34, 1) << 34 | \
    (lut_64_1_t)_LUT_RANGE(v35, 1) << 35 | \
    (lut_64_1_t)_LUT_RANGE(v36, 1) << 36 | \
    (lut_64_1_t)_LUT_RANGE(v37, 1) << 37 | \
    (lut_64_1_t)_LUT_RANGE(v38, 1) << 38 | \
    (lut_64_1_t)_LUT_RANGE(v39, 1) << 39 | \
    (lut_64_1_t)_LUT_RANGE(v40, 1) << 40 | \
    (lut_64_1_t)_LUT_RANGE(v41, 1) << 41 | \
    (lut_64_1_t)_LUT_RANGE(v42, 1) << 42 | \
    (lut_64_1_t)_LUT_RANGE(v43, 1) << 43 | \
    (lut_64_1_t)_LUT_RANGE(v44, 1) << 44 | \
    (lut_64_1_t)_LUT_RANGE(v45, 1) << 45 | \
    (lut_64_1_t)_LUT_RANGE(v46, 1) << 46 | \
    (lut_64_1_t)_LUT_RANGE(v47, 1) << 47 | \
    (lut_64_1_t)_LUT_RANGE(v48, 1) << 48 | \
    (lut_64_1_t)_LUT_RANGE(v49, 1) << 49 | \
    (lut_64_1_t)_LUT_RANGE(v50, 1) << 50 | \
    (lut_64_1_t)_LUT_RANGE(v51, 1) << 51 | \
    (lut_64_1_t)_LUT_RANGE(v52, 1) << 52 | \
    (lut_64_1_t)_LUT_RANGE(v53, 1) << 53 | \
    (lut_64_1_t)_LUT_RANGE(v54, 1) << 54 | \
    (lut_64_1_t)_LUT_RANGE(v55, 1) << 55 | \
    (lut_64_1_t)_LUT_RANGE(v56, 1) << 56 | \
    (lut_64_1_t)_LUT_RANGE(v57, 1) << 57 | \
    (lut_64_1_t)_LUT_RANGE(v58, 1) << 58 | \
    (lut_64_1_t)_LUT_RANGE(v59, 1) << 59 | \
    (lut_64_1_t)_LUT_RANGE(v60, 1) << 60 | \
    (lut_64_1_t)_LUT_RANGE(v61, 1) << 61 | \
    (lut_64_1_t)_LUT_RANGE(v62, 1) << 62 | \
    (lut_64_1_t)_LUT_RANGE(v63, 1) << 63)

/** @This computes an integer constant usable as a shift based lookup
    table of 64 entries of 1 bits each. The macro takes a list of
    index and value pairs as parameters. */
#define DLUT_64_1_DEF(...) DLUT_64_1_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This computes an integer constant usable as a shift based lookup
    table of 64 entries of 1 bits each. The macro takes a list of
    values as parameters. */
#define XLUT_64_1_DEF(...) XLUT_64_1_DEF_(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/** @This accesses a shift based lookup table of 64 entries of 1 bits each. */
#define LUT_64_1_GET(x, lut) (((lut) >> ((x) * 1)) & 1)

/** @This defines a shift based lookup table of 64
    entries of 1 bits each and accesses it.
    @csee #DLUT_64_1_DEF @csee #DLUT_64_1_GET */
#define DLUT_64_1(x, ...) LUT_64_1_GET(x, DLUT_64_1_DEF(__VA_ARGS__))

/** @This defines a shift based lookup table of 64
    entries of 1 bits each and accesses it.
    @csee #XLUT_64_1_DEF @csee #XLUT_64_1_GET */
#define XLUT_64_1(x, ...) LUT_64_1_GET(x, XLUT_64_1_DEF(__VA_ARGS__))

#endif
