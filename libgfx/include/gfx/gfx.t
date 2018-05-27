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

#ifndef _GFX_H_
#define _GFX_H_

#include <stdint.h>

#ifndef __MUTEK__		/* mkdoc:skip */
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#define align_pow2_up(x, b) ((((x) - 1) | ((b) - 1)) + 1)
#define align_pow2_down(x, b) ((x) & ~((b) - 1))
#define bit_msb_index(x) (sizeof(int) * 8 - 1 - __builtin_clz(x))
# define ALWAYS_INLINE __attribute__((always_inline)) inline
#define GFX_ASSERT(...)
typedef int8_t error_t;
typedef uint8_t bool_t;

/* specifies log2 of default surface bits per pixel */
#define CONFIG_GFX_DEFAULT_L2BPP 0

/* enable support for 1 bit per pixel surfaces */
#define CONFIG_GFX_BPP0
/* enable support for 2 bit per pixel surfaces */
#define CONFIG_GFX_BPP1
/* enable support for 4 bit per pixel surfaces */
#define CONFIG_GFX_BPP2
/* enable support for 8 bit per pixel surfaces */
#define CONFIG_GFX_BPP3
/* enable support for 16 bit per pixel surfaces */
#define CONFIG_GFX_BPP4
/* enable support for 32 bit per pixel surfaces */
#define CONFIG_GFX_BPP5

#define CONFIG_GFX_LOG2_WORD_WIDTH 2

//#define CONFIG_GFX_LOG2_SURFACE_SIZE
#define CONFIG_GFX_UNROLL

#else                            /* MutekH */
#define GFX_ASSERT(...) assert(__VA_ARGS__)
#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/bit.h>
#endif

/** An integer type large enough to hold a pixel value in any format. */
typedef uint32_t gfx_pixel_t;
/** An signed integer type large enough to hold a pixel coordinate. */
typedef int32_t  gfx_pos_t;
/** An integer type large enough to address a word in the data storage of a surface. */
typedef uint32_t gfx_addr_t;

#include <gfx/math.h>

/** @This specifies the various implemented pixel formats */
enum gfx_surface_format
{
  /** see CONFIG_GFX_DEFAULT_L2BPP */
  GFX_FMT_DEFAULT,
  GFX_FMT_1BIT,
  GFX_FMT_2BIT,
  GFX_FMT_4BIT,
  GFX_FMT_8BIT,
  GFX_FMT_16BIT,
  GFX_FMT_32BIT,
};

/** @This specifies the various implemented surface compression formats */
enum gfx_surface_compress
{
  GFX_CMP_NONE,
  GFX_CMP_RLE,
  GFX_CMP_LZO,
};

/** @This specifies directions. */
enum gfx_direction_e
{
  GFX_DIR_LEFT  = 0,
  GFX_DIR_RIGHT = 1,
  GFX_DIR_UP    = 2,
  GFX_DIR_DOWN  = 3,
};

/** @internalmembers
    @This is a surface descriptor.

    A surface object has an associated user provided buffer used to
    store the image data. The actual data format depends on the pixel
    format and library configuration.

    The image data is stored as an array of @ref gfx_word_t. Native
    endianess is used. The word width used is defined by the @ref
    #CONFIG_GFX_LOG2_WORD_WIDTH token.

    @see gfx_surface_init
*/
struct gfx_surface_s
{
  /** pointer to surface data */
  void *ptr;
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  /** ptr index mask */
  gfx_addr_t mask;
  /** log2 of row size in words */
  uint8_t l2bw;
  /** log2 of width in pixels */
  uint8_t l2w;
  /** log2 of height in pixels */
  uint8_t l2h;
#else
  /** size in words */
  gfx_addr_t bsize;
  /** row size in words */
  uint16_t bw;
  /** width in pixels */
  uint16_t w;
  /** height in pixels */
  uint16_t h;
#endif
  /** pixel format */
  enum gfx_surface_format fmt;
};

/** @internalmembers
    @This contains a @ref gfx_surface_s object along with information
    about the set of tiles it contains. This allows extracting a single
    tile for drawing on an other surface.

    This is typically used to implement bitmap fonts.
*/
struct gfx_tilemap_s
{
  struct gfx_surface_s s;
  /* log2 tiles per row */
  uint8_t l2tpr;
  /* tile width */
  uint8_t tw;
  /* tile height */
  uint8_t th;
  /* index of first tile in strings */
  uint8_t offset;
};

/** @This initializes a tilemap object from a surface containing the
    tile set. You need to specifies the resolution of a single tile.

    The number of tiles on a single row of the tile set image is the
    largest power of 2 that fits. They may be padding on the right if
    you have some surface resolution constraints.

    Example: You have configured the library to only support surfaces
    with power of two resolution. You have a tile set image with a
    resolution of 256x128 pixels and the resolution of a single tile
    is 14x24. You then need to design your tile set image with 16
    tiles per row and 32 pixels of padding at the end of each row.
*/
error_t gfx_tilemap_init(struct gfx_tilemap_s *t,
			 const struct gfx_surface_s *s,
			 gfx_pos_t tw, gfx_pos_t th,
			 uint_fast8_t first);

/** @This returns the address of the row of pixels at position @em y in a surface. */
ALWAYS_INLINE gfx_addr_t
gfx_yaddr(const struct gfx_surface_s * __restrict__ s,
	  gfx_pos_t y)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return y << s->l2bw;
#else
  return y * s->bw;
#endif
}

/** @This returns the height of a surface in pixels. */
ALWAYS_INLINE gfx_pos_t
gfx_height(const struct gfx_surface_s * __restrict__ s)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return 1 << s->l2h;
#else
  return s->h;
#endif
}

/** @This performs a modulus on the @em y coordinate in a surface. */
#ifndef CONFIG_GFX_LOG2_SURFACE_SIZE
__attribute__((deprecated))
#endif
ALWAYS_INLINE gfx_pos_t
gfx_ymod(const struct gfx_surface_s * __restrict__ s,
	 gfx_pos_t y)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return y & ((1 << s->l2h) - 1);
#else
  return y % s->h;
#endif
}

/** @This returns the width of a surface in pixels. */
ALWAYS_INLINE gfx_pos_t
gfx_width(const struct gfx_surface_s * __restrict__ s)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return 1 << s->l2w;
#else
  return s->w;
#endif
}

/** @This performs a modulus on the @em x coordinate in a surface. */
#ifndef CONFIG_GFX_LOG2_SURFACE_SIZE
__attribute__((deprecated))
#endif
ALWAYS_INLINE gfx_pos_t
gfx_xmod(const struct gfx_surface_s * __restrict__ s,
	 gfx_pos_t x)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return x & ((1 << s->l2w) - 1);
#else
  return x % s->w;
#endif
}

/** @This returns false if the x coordinate falls outside the surface */
ALWAYS_INLINE bool_t
gfx_xcheck(const struct gfx_surface_s * __restrict__ s,
	   gfx_pos_t x)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return !(x >> s->l2w);
#else
  return (gfx_addr_t)x < (gfx_addr_t)s->w;
#endif
}

/** @This returns false if the y coordinate falls outside the surface */
ALWAYS_INLINE bool_t
gfx_ycheck(const struct gfx_surface_s * __restrict__ s,
	   gfx_pos_t y)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return !(y >> s->l2h);
#else
  return (gfx_addr_t)y < (gfx_addr_t)s->h;
#endif
}

/** @internal @This clamps the pixel address so that it always fall in the surface. */
ALWAYS_INLINE gfx_addr_t
gfx_xymod(const struct gfx_surface_s * __restrict__ s,
          gfx_addr_t a)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  return a & s->mask;
#else
  return a < s->bsize ? a : 0;
#endif
}

/** @This returns false if the box falls outside the surface */
inline bool_t
gfx_box_check(const struct gfx_surface_s * __restrict__ s,
	      gfx_pos_t x0, gfx_pos_t y0,
	      gfx_pos_t x1, gfx_pos_t y1)
{
  return gfx_xcheck(s, x0) &&
         gfx_xcheck(s, x1) &&
         gfx_ycheck(s, y0) &&
         gfx_ycheck(s, y1);
}

/** @This prevents coordinates from falling outside a surface either
    by mangling the values or returns an error. */
__attribute__((warn_unused_result))
inline bool_t
gfx_box_safe(const struct gfx_surface_s * __restrict__ s,
             gfx_pos_t *x0, gfx_pos_t *y0,
             gfx_pos_t *x1, gfx_pos_t *y1)
{
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  gfx_pos_t xm = ((1 << s->l2w) - 1);
  *x0 &= xm;
  *x1 &= xm;
  gfx_pos_t ym = ((1 << s->l2h) - 1);
  *y0 &= ym;
  *y1 &= ym;
  return 1;
#else
  return gfx_box_check(s, *x0, *y0, *x1, *y1);
#endif
}

#define _GFX_CPP1(a) a
#define _GFX_CPP3(a, b, c) a##b##c
#define __GFX_CPP3(a, b, c) _GFX_CPP3(a, b, c)

#define _GFX_SWAP(a, b) do { typeof(a) _t = (a); (a) = (b); (b) = _t; } while (0)

#ifdef CONFIG_GFX_BPP0
# define _GFX_BPP0(...) __VA_ARGS__
#elif CONFIG_GFX_DEFAULT_L2BPP == 0
# error CONFIG_GFX_DEFAULT_L2BPP == 0 requires CONFIG_GFX_BPP0
#else
# define _GFX_BPP0(...)
#endif

#ifdef CONFIG_GFX_BPP1
# define _GFX_BPP1(...) __VA_ARGS__
#elif CONFIG_GFX_DEFAULT_L2BPP == 1
# error CONFIG_GFX_DEFAULT_L2BPP == 1 requires CONFIG_GFX_BPP1
#else
# define _GFX_BPP1(...)
#endif

#ifdef CONFIG_GFX_BPP2
# define _GFX_BPP2(...) __VA_ARGS__
#elif CONFIG_GFX_DEFAULT_L2BPP == 2
# error CONFIG_GFX_DEFAULT_L2BPP == 2 requires CONFIG_GFX_BPP2
#else
# define _GFX_BPP2(...)
#endif

#ifdef CONFIG_GFX_BPP3
# define _GFX_BPP3(...) __VA_ARGS__
#elif CONFIG_GFX_DEFAULT_L2BPP == 3
# error CONFIG_GFX_DEFAULT_L2BPP == 3 requires CONFIG_GFX_BPP3
#else
# define _GFX_BPP3(...)
#endif

#ifdef CONFIG_GFX_BPP4
# define _GFX_BPP4(...) __VA_ARGS__
#elif CONFIG_GFX_DEFAULT_L2BPP == 4
# error CONFIG_GFX_DEFAULT_L2BPP == 4 requires CONFIG_GFX_BPP4
#else
# define _GFX_BPP4(...)
#endif

#ifdef CONFIG_GFX_BPP5
# define _GFX_BPP5(...) __VA_ARGS__
#elif CONFIG_GFX_DEFAULT_L2BPP == 5
# error CONFIG_GFX_DEFAULT_L2BPP == 5 requires CONFIG_GFX_BPP5
#else
# define _GFX_BPP5(...)
#endif

/* backslash-region-begin */
#define _GFX_FMT_SWITCH(name, args)
      {
	_GFX_BPP0( case GFX_FMT_1BIT:
		   return name##_0 args; );
	_GFX_BPP1( case GFX_FMT_2BIT:
		   return name##_1 args; );
	_GFX_BPP2( case GFX_FMT_4BIT:
		   return name##_2 args; );
	_GFX_BPP3( case GFX_FMT_8BIT:
		   return name##_3 args; );
	_GFX_BPP4( case GFX_FMT_16BIT:
		   return name##_4 args; );
	_GFX_BPP5( case GFX_FMT_32BIT:
		   return name##_5 args; );
      default:
	return __GFX_CPP3(name, _, CONFIG_GFX_DEFAULT_L2BPP) args;
      }
/* backslash-region-end */

/*  The _GFX_BPP_EXPAND macro expands a macro containing some template
    code which depends on the number of bits per pixel.

    The @em ops macro must take the following arguments:
      bits per pixel:  bpp
      pixels per word: ppw
      log2(bpp):       l2bpp
      log2(ppw):       l2ppw
      pixel mask:      pm
      pixs=1 in word:  ps
      word type:       word_t
*/

#if CONFIG_GFX_LOG2_WORD_WIDTH == 0 /* mkdoc:skip */
typedef uint8_t gfx_word_t;
/* backslash-region-begin */
#define _GFX_BPP_EXPAND(ops)
_GFX_BPP0(ops(1, 8, 0, 3, 0x01, 0xff, gfx_word_t))
_GFX_BPP1(ops(2, 4, 1, 2, 0x03, 0x55, gfx_word_t))
_GFX_BPP2(ops(4, 2, 2, 1, 0x0f, 0x11, gfx_word_t))
_GFX_BPP3(ops(8, 1, 3, 0, 0xff, 0x01, gfx_word_t))
/* backslash-region-end */
# undef _GFX_BPP4
# define _GFX_BPP4(...)
# undef _GFX_BPP5
# define _GFX_BPP5(...)

#elif CONFIG_GFX_LOG2_WORD_WIDTH == 1 /* mkdoc:skip */
typedef uint16_t gfx_word_t;
/* backslash-region-begin */
#define _GFX_BPP_EXPAND(ops)
_GFX_BPP0(ops(1,  16, 0, 4, 0x0001, 0xffff, gfx_word_t))
_GFX_BPP1(ops(2,  8,  1, 3, 0x0003, 0x5555, gfx_word_t))
_GFX_BPP2(ops(4,  4,  2, 2, 0x000f, 0x1111, gfx_word_t))
_GFX_BPP3(ops(8,  2,  3, 1, 0x00ff, 0x0101, gfx_word_t))
_GFX_BPP4(ops(16, 1,  4, 0, 0xffff, 0x0001, gfx_word_t))
# undef _GFX_BPP5
# define _GFX_BPP5(...)
/* backslash-region-end */

#elif CONFIG_GFX_LOG2_WORD_WIDTH == 2 /* mkdoc:skip */
typedef uint32_t gfx_word_t;
/* backslash-region-begin */
#define _GFX_BPP_EXPAND(ops)
_GFX_BPP0(ops(1,  32, 0, 5, 0x00000001, 0xffffffff, gfx_word_t))
_GFX_BPP1(ops(2,  16, 1, 4, 0x00000003, 0x55555555, gfx_word_t))
_GFX_BPP2(ops(4,  8,  2, 3, 0x0000000f, 0x11111111, gfx_word_t))
_GFX_BPP3(ops(8,  4,  3, 2, 0x000000ff, 0x01010101, gfx_word_t))
_GFX_BPP4(ops(16, 2,  4, 1, 0x0000ffff, 0x00010001, gfx_word_t))
_GFX_BPP5(ops(32, 1,  5, 0, 0x00ffffff, 0x00000001, gfx_word_t))
/* backslash-region-end */

#else
# error word width not supported
#endif

/** @internal @This contains properties of a pixel format. */
struct gfx_fmt_desc_s
{
  /* log2 bits per pixel */
  uint8_t l2bpp;
  /* log2 pixels per word */
  uint8_t l2ppw;
  /* pixel value mask */
  gfx_pixel_t pm;
};

/** @This initializes a surface with specified log2 width, log2 height
    and format. The surface data is located at @tt {data + offset} and
    is checked to be within @tt {data + size}. */
error_t gfx_surface_init(struct gfx_surface_s *s, gfx_word_t *data,
                         size_t bytes, gfx_pos_t w, gfx_pos_t h,
                         enum gfx_surface_format fmt);

/** @This computes the size in bits needed to store the surface data.
    This may not be byte aligned. @see gfx_surface_bytes */
error_t gfx_surface_bits(size_t *bits, gfx_pos_t w, gfx_pos_t h,
			 enum gfx_surface_format fmt);

/** @This computes the size in bytes needed to store the surface data. */
error_t gfx_surface_bytes(size_t *bytes, gfx_pos_t w, gfx_pos_t h,
			  enum gfx_surface_format fmt);

/** @This initializes a surface with a dummy single pixel storage. */
void
gfx_surface_dummy(struct gfx_surface_s * __restrict__ s);

/** @This returns a pointer to the surface storage. */
ALWAYS_INLINE gfx_word_t *
gfx_surface_data(const struct gfx_surface_s * __restrict__ s)
{
  return s->ptr;
}

/* backslash-region-begin */
#define _GFX_BPP_L2PPW(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)
enum {
  _GFX_BPP##l2bpp##_L2PPW = l2ppw,
  _GFX_BPP##l2bpp##_PM = pm
};
/* backslash-region-end */
_GFX_BPP_EXPAND(_GFX_BPP_L2PPW);

/** @internal @This contains properties all supported pixel formats. */
static const struct gfx_fmt_desc_s gfx_fmt_desc[16] = {
  {
    .l2bpp = CONFIG_GFX_DEFAULT_L2BPP,
    .l2ppw = __GFX_CPP3(_GFX_BPP, CONFIG_GFX_DEFAULT_L2BPP, _L2PPW),
    .pm = __GFX_CPP3(_GFX_BPP, CONFIG_GFX_DEFAULT_L2BPP, _PM),
  },
  {
    _GFX_BPP0( .l2bpp = 0,
               .l2ppw = _GFX_BPP0_L2PPW,
               .pm = _GFX_BPP0_PM )
  },
  {
    _GFX_BPP1( .l2bpp = 1,
               .l2ppw = _GFX_BPP1_L2PPW,
               .pm = _GFX_BPP1_PM )
  },
  {
    _GFX_BPP2( .l2bpp = 2,
               .l2ppw = _GFX_BPP2_L2PPW,
               .pm = _GFX_BPP2_PM )
  },
  {
    _GFX_BPP3( .l2bpp = 3,
               .l2ppw = _GFX_BPP3_L2PPW,
               .pm = _GFX_BPP3_PM )
  },
  {
    _GFX_BPP4( .l2bpp = 4,
               .l2ppw = _GFX_BPP4_L2PPW,
               .pm = _GFX_BPP4_PM )
  },
  {
    _GFX_BPP5( .l2bpp = 5,
               .l2ppw = _GFX_BPP5_L2PPW,
               .pm = _GFX_BPP5_PM )
  },
};

/** @This draws a single tile from a tilemap on a surface. */
bool_t
gfx_draw_tile(const struct gfx_surface_s * __restrict__ s,
              const struct gfx_tilemap_s * __restrict__ t,
              uint_fast16_t tile, gfx_pos_t x, gfx_pos_t y, bool_t center);

/** @This draws multiple tiles from a tilemap on a surface.
    Tiles index are taken from a string. */
void
gfx_draw_tile_string(const struct gfx_surface_s * __restrict__ s,
                     const struct gfx_tilemap_s * __restrict__ t,
                     const uint8_t *str, uint_fast16_t size,
                     gfx_pos_t x, gfx_pos_t y, enum gfx_direction_e dir,
                     bool_t center);

#endif

