/*  -*- c -*-

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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2017

*/

#ifndef _GFX_BYTECODE_H_
#define _GFX_BYTECODE_H_

#include <gfx/gfx.h>

struct bc_context_s;

typedef uint32_t gfx_bc_surface_desc_t;
typedef uint32_t gfx_bc_tilemap_desc_t;
typedef uint32_t gfx_2dvector_t;

/** @This packs 11 bits integer coordinates as a 32 bits value.
@code r
    in:                         XXX XXXXXXXX
                                YYY YYYYYYYY
    out: xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
@end code
*/
#define GFX_XY(x, y) ( ((x) & 0x700) << 11 | ((y) & 0x7ff) << 8 | ((x) & 0xff) )

/** @This packs Q11.5 fixed point coordinates as a 32 bits value
for use in a bytecode programm.
@code r
    in:                     XXXXXXXXXXXxxxxx
                            YYYYYYYYYYYyyyyy
    out: xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
@end code
*/
#define GFX_XYF(x, y) (((x) & 0x1f) << 27 | ((x) & 0x1fe0) >> 5 | ((x) & 0xe000) << 6 \
                     | ((y) & 0x1f) << 22 | ((y) & 0xffe0) << 3)

/** @This packs surface attributes as a 32 bits value for use in a
    bytecode programm. */
#define GFX_SURFACE(width, height, fmt) \
	((width) << 20 | (height) << 8 | (fmt))

/** @This packs tilemap attributes as a 32 bits value for use in a
    bytecode programm. */
#define GFX_TILEMAP(width, height, offset)              \
	((width) << 20 | (height) << 8 | (offset))

/** @internal
@code r
    in:  xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
    out: YYYYYYYY YYYyyyyy XXXXXXXX XXXxxxxx
@end code
*/
ALWAYS_INLINE gfx_2dvector_t gfx_vector_p2xy(gfx_2dvector_t p)
{
  return ((p & 0x7ff00) << 13)
       | ((p & 0xff) << 5)
       | ((p >> 6) & 0x1fe000)
       | ((p >> 27) & 0x1f);
}

/** @internal
@code r
    in:  YYYYYYYY YYYyyyyy XXXXXXXX XXXxxxxx
    out: xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
@end code
*/
ALWAYS_INLINE gfx_2dvector_t gfx_vector_xy2p(gfx_2dvector_t p)
{
  return ((p >> 13) & 0x7ff00)
       | ((p >> 5) & 0xff)
       | ((p & 0x1fe000) << 6)
       | ((p & 0x1f) << 27);
}

/** @internal
@code r
    out: xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
@end code
*/
ALWAYS_INLINE gfx_2dvector_t gfx_vector_xy_2p(gfx_pos_t x, gfx_pos_t y)
{
  return (x & 0x1f) << 27 | (x & 0x1fe0) >> 5 | (x & 0xe000) << 6
    |    (y & 0x1f) << 22 | (y & 0xffe0) << 3;
}

/** @internal
@code r
    in:  xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
    out:                        XXX XXXXXXXX
@end code
*/
ALWAYS_INLINE uint_fast16_t gfx_vector_xint(gfx_2dvector_t p)
{
  return ((p >> 11) & 0x700) | (p & 0xff);
}

/** @internal
@code r
    in:  xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
    out:                  XXX XXXXXXXX xxxxx
@end code
*/
ALWAYS_INLINE uint_fast16_t gfx_vector_x(gfx_2dvector_t p)
{
  return ((p >> 6) & 0xe000) | ((p & 0xff) << 5) | ((p >> 27) & 0x1f);
}

/** @internal
@code r
    in:                         XXX XXXXXXXX
    out: xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
@end code
*/
ALWAYS_INLINE gfx_2dvector_t gfx_vector_xint2p(uint_fast16_t x)
{
  return ((x & 0x700) << 11) | (x & 0xff);
}

/** @internal
@code r
    in:  xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
    out:                        YYY YYYYYYYY
@end code
*/
ALWAYS_INLINE uint_fast16_t gfx_vector_yint(gfx_2dvector_t p)
{
  return (p >> 8) & 0x7ff;
}

/** @internal
@code r
    in:  xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
    out:                  YYY YYYYYYYY yyyyy
@end code
*/
ALWAYS_INLINE uint_fast16_t gfx_vector_y(gfx_2dvector_t p)
{
  return ((p >> 3) & 0xffe0) | ((p >> 22) & 0x1f);
}

/** @internal
@code r
    in:                         YYY YYYYYYYY
    out: xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
@end code
*/
ALWAYS_INLINE gfx_2dvector_t gfx_vector_yint2p(uint_fast16_t y)
{
  return (y & 0x7ff) << 8;
}

/** @This contains the fours surfaces and the single tilemap available
    to the bytecode program. */
struct gfx_bc_context_s
{
  /** surface pool */
  struct gfx_surface_s s[4];

  /** currently selected tilemap */
  struct gfx_tilemap_s tilemap;

  /** current drawing attribute */
  gfx_pixel_t attr;
};

/** @This initializes a gfx bytecode context. */
void gfx_bc_init(struct gfx_bc_context_s *ctx);

/** @This executes a single gfx bytecode operation. */
error_t gfx_bc_run(struct bc_context_s *vm,
		   struct gfx_bc_context_s *ctx,
		   uint16_t op);

/** @This tests if the return status of @ref bc_run is gfx opcode. */
#define GFX_BC_IS_GFX_OP(op) (((op) & 0xf000) > 0x8000)

#endif

