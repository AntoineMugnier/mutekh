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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2017

*/

#include <gfx/pixel.h>
#include <gfx/line.h>
#include <gfx/circle.h>
#include <gfx/arc.h>
#include <gfx/rect.h>
#include <gfx/blit.h>

#include <assert.h>

error_t gfx_surface_storage(size_t *bytes, size_t *row_bytes,
                            gfx_pos_t w, gfx_pos_t h,
                            enum gfx_surface_format fmt)
{
  uint_fast8_t l2ppw = gfx_fmt_desc[fmt].l2ppw;

#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  if ((w & (w - 1)) | (h & (h - 1)))
    return -ENOTSUP;

  uint_fast8_t l2w = bit_msb_index(w);

  /* a single row must not be less than a word */
  if (l2w < l2ppw)
    return -ENOTSUP;

  size_t wpr = 1 << (l2w - l2ppw); /* words per row */
#else
  size_t wpr = ((w - 1) >> l2ppw) + 1;
#endif

  if (bytes)
    {
      size_t b = wpr * h;
      *bytes = (b + !b) * sizeof(gfx_word_t);
    }

  if (row_bytes)
    *row_bytes = wpr * sizeof(gfx_word_t);

  return 0;
}

error_t gfx_surface_init(struct gfx_surface_s *s, gfx_word_t *data,
                         size_t bytes_, gfx_pos_t w, gfx_pos_t h,
                         enum gfx_surface_format fmt)
{
  uint_fast8_t l2bpp = gfx_fmt_desc[fmt].l2bpp;
  uint_fast8_t l2ppw = gfx_fmt_desc[fmt].l2ppw;

  size_t bytes, row_bytes;
  if (gfx_surface_storage(&bytes, &row_bytes, w, h, fmt))
    return -ERANGE;

  if (bytes_ < bytes)
    return -ERANGE;

#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  uint_fast8_t l2w = bit_msb_index(w);
  uint_fast8_t l2h = bit_msb_index(h);

  gfx_addr_t mask = bytes - 1;
  s->mask = mask;
  s->l2bw = l2w - l2ppw;
  s->l2w = l2w;
  s->l2h = l2h;

#else
  s->bsize = bytes / sizeof(gfx_word_t);
  s->bw = row_bytes / sizeof(gfx_word_t);
  s->w = w;
  s->h = h;
#endif

# if 0
  logk("surface ptr:%x w:%u h:%u fmt:%u l2bpp:%u bytes:%u row_bytes:%u\n",
          s->ptr, w, h, fmt, l2bpp, bytes, row_bytes);
# endif

  s->ptr = data;
  s->fmt = fmt;

  return 0;
}

void
gfx_surface_dummy(struct gfx_surface_s * __restrict__ s)
{
  static gfx_word_t dummy;
  s->ptr = &dummy;
#ifdef CONFIG_GFX_LOG2_SURFACE_SIZE
  s->mask = 0;
  s->l2bw = 0;
  s->l2w = 0;
  s->l2h = 0;
#else
  s->bsize = 1;
  s->bw = 1;
  s->w = 1;
  s->h = 1;
#endif
  s->fmt = GFX_FMT_DEFAULT;
}

error_t gfx_tilemap_init(struct gfx_tilemap_s *t,
			 const struct gfx_surface_s *s,
			 gfx_pos_t tw, gfx_pos_t th,
			 uint_fast8_t first)
{
  if (!tw || !th)
    return -ERANGE;

  gfx_pos_t sw = gfx_width(s);
  gfx_pos_t sh = gfx_height(s);

  if (sw < tw || sh < th)
    return -ERANGE;

  t->s = *s;
  t->offset = first;
  t->tw = tw;
  t->th = th;
  t->l2tpr = bit_msb_index(sw / tw);

  return 0;
}

extern inline bool_t
gfx_box_check(const struct gfx_surface_s * __restrict__ s,
	      gfx_pos_t x0, gfx_pos_t y0,
	      gfx_pos_t x1, gfx_pos_t y1);

extern inline bool_t
gfx_box_safe(const struct gfx_surface_s * __restrict__ s,
	     gfx_pos_t *x0, gfx_pos_t *y0,
	     gfx_pos_t *x1, gfx_pos_t *y1);

extern inline int32_t gfx_sin(uint_fast16_t x);

extern inline int32_t gfx_cos(uint_fast16_t x);

extern inline uint32_t gfx_sqrt32(uint32_t x);

const int32_t gfx_sin_table[129] = { /* sin table in Q2.30 fixed point */
  0x00000000, 0x00c90e8f, 0x0192155f, 0x025b0cae, 0x0323ecbe, 0x03ecadcf, 0x04b54824, 0x057db402,
  0x0645e9af, 0x070de171, 0x07d59395, 0x089cf867, 0x09640837, 0x0a2abb58, 0x0af10a22, 0x0bb6ecef,
  0x0c7c5c1e, 0x0d415012, 0x0e05c135, 0x0ec9a7f2, 0x0f8cfcbd, 0x104fb80e, 0x1111d262, 0x11d3443f,
  0x1294062e, 0x135410c2, 0x14135c94, 0x14d1e242, 0x158f9a75, 0x164c7ddd, 0x17088530, 0x17c3a931,
  0x187de2a6, 0x19372a63, 0x19ef7943, 0x1aa6c82b, 0x1b5d1009, 0x1c1249d8, 0x1cc66e99, 0x1d79775b,
  0x1e2b5d38, 0x1edc1952, 0x1f8ba4db, 0x2039f90e, 0x20e70f32, 0x2192e09a, 0x223d66a8, 0x22e69ac7,
  0x238e7673, 0x2434f332, 0x24da0a99, 0x257db64b, 0x261feff9, 0x26c0b162, 0x275ff452, 0x27fdb2a6,
  0x2899e64a, 0x29348937, 0x29cd9577, 0x2a650525, 0x2afad269, 0x2b8ef77c, 0x2c216eaa, 0x2cb2324b,
  0x2d413ccc, 0x2dce88a9, 0x2e5a106f, 0x2ee3cebe, 0x2f6bbe44, 0x2ff1d9c6, 0x30761c17, 0x30f8801f,
  0x317900d6, 0x31f79947, 0x32744493, 0x32eefde9, 0x3367c08f, 0x33de87de, 0x34534f40, 0x34c61236,
  0x3536cc52, 0x35a5793c, 0x361214b0, 0x367c9a7d, 0x36e5068a, 0x374b54ce, 0x37af8158, 0x3811884c,
  0x387165e3, 0x38cf1669, 0x392a9642, 0x3983e1e7, 0x39daf5e8, 0x3a2fcee8, 0x3a8269a2, 0x3ad2c2e7,
  0x3b20d79e, 0x3b6ca4c4, 0x3bb6276d, 0x3bfd5cc4, 0x3c424209, 0x3c84d496, 0x3cc511d8, 0x3d02f756,
  0x3d3e82ad, 0x3d77b191, 0x3dae81ce, 0x3de2f147, 0x3e14fdf7, 0x3e44a5ee, 0x3e71e758, 0x3e9cc076,
  0x3ec52f9f, 0x3eeb3347, 0x3f0ec9f4, 0x3f2ff249, 0x3f4eaafe, 0x3f6af2e3, 0x3f84c8e1, 0x3f9c2bfa,
  0x3fb11b47, 0x3fc395f9, 0x3fd39b5a, 0x3fe12acb, 0x3fec43c6, 0x3ff4e5df, 0x3ffb10c1, 0x3ffec42d,
  0x40000000
};

_GFX_BPP_EXPAND(_GFX_PIXEL_OPS);

extern inline gfx_pixel_t
gfx_get_pixel_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x, gfx_pos_t y);

extern inline void
gfx_put_pixel_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x, gfx_pos_t y, gfx_pixel_t a);

extern inline void
gfx_hline_safe(const struct gfx_surface_s * __restrict__ s,
	       gfx_pos_t x0, gfx_pos_t x1, gfx_pos_t y, gfx_pixel_t a);

extern inline void
gfx_vline_safe(const struct gfx_surface_s * __restrict__ s,
	       gfx_pos_t x, gfx_pos_t y0, gfx_pos_t y1, gfx_pixel_t a);

extern inline void
gfx_clear(const struct gfx_surface_s * __restrict__ s, gfx_pixel_t a);

_GFX_BPP_EXPAND(_GFX_CIRCLE_OPS);

extern inline bool_t
gfx_circle_check(const struct gfx_surface_s * __restrict__ s,
                 gfx_pos_t xc, gfx_pos_t yc,
                 gfx_pos_t r,
                 uint8_t oct);

extern inline void
gfx_draw_circle_safe(const struct gfx_surface_s * __restrict__ s,
                     gfx_pos_t xc, gfx_pos_t yc,
                     gfx_pos_t r, uint8_t oct, gfx_pixel_t a);

extern inline void
gfx_draw_circle_infill_safe(const struct gfx_surface_s * __restrict__ s,
                            gfx_pos_t xc, gfx_pos_t yc,
                            gfx_pos_t r, uint8_t oct, gfx_pixel_t a);

extern inline void
gfx_draw_circle_outfill_safe(const struct gfx_surface_s * __restrict__ s,
                             gfx_pos_t xc, gfx_pos_t yc,
                             gfx_pos_t r, uint8_t oct, gfx_pixel_t a);

_GFX_BPP_EXPAND(_GFX_LINE_OPS);

extern inline void
gfx_draw_line_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x0, gfx_pos_t y0,
		   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a);

_GFX_BPP_EXPAND(_GFX_ARC_OPS);

extern inline void
gfx_draw_arc_angles_safe(const struct gfx_surface_s *s,
                         uint_fast16_t xc, uint_fast16_t yc,
                         uint_fast16_t a0, uint_fast16_t a1,
                         uint_fast16_t r, uint32_t ccw, gfx_pixel_t a);

extern inline void
gfx_draw_arc_xy_safe(const struct gfx_surface_s *s,
                     int32_t x0, int32_t y0,
                     int32_t x1, int32_t y1,
                     uint_fast16_t r, uint32_t ccw, gfx_pixel_t a);

_GFX_BPP_EXPAND(_GFX_RECT_OPS);

extern inline void
gfx_draw_rect_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x0, gfx_pos_t y0,
		   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a);

extern inline void
gfx_draw_rect_r_safe(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t y0,
		     gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a);

extern inline void
gfx_draw_rect_f_safe(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t y0,
		     gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a);

extern inline void
gfx_draw_rect_fr_safe(const struct gfx_surface_s * __restrict__ s,
		      gfx_pos_t x0, gfx_pos_t y0,
		      gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a);

_GFX_BPP_EXPAND(_GFX_BLIT_OPS);

extern inline void
gfx_blit_nc(const struct gfx_surface_s * __restrict__ d,
	    gfx_pos_t x2, gfx_pos_t y2, /* dest */
	    const struct gfx_surface_s * __restrict__ s,
	    gfx_pos_t x0, gfx_pos_t y0, /* src */
	    gfx_pos_t w, gfx_pos_t h);

extern inline void
gfx_blit_safe(const struct gfx_surface_s * __restrict__ d,
	      gfx_pos_t x2, gfx_pos_t y2, /* dest */
	      const struct gfx_surface_s * __restrict__ s,
	      gfx_pos_t x0, gfx_pos_t y0, /* src */
	      gfx_pos_t w, gfx_pos_t h);

extern inline void
gfx_blit_overlap_nc(const struct gfx_surface_s * __restrict__ d,
		    gfx_pos_t x2, gfx_pos_t y2, /* dest */
		    gfx_pos_t x0, gfx_pos_t y0, /* src */
		    gfx_pos_t w, gfx_pos_t h);

extern inline void
gfx_blit_overlap_safe(const struct gfx_surface_s * __restrict__ d,
		      gfx_pos_t x2, gfx_pos_t y2, /* dest */
		      gfx_pos_t x0, gfx_pos_t y0, /* src */
		      gfx_pos_t w, gfx_pos_t h);

bool_t
gfx_draw_tile(const struct gfx_surface_s * __restrict__ s,
              const struct gfx_tilemap_s * __restrict__ t,
              uint_fast16_t tile, gfx_pos_t x, gfx_pos_t y,
              bool_t center)
{
  uint_fast8_t tw = t->tw;
  uint_fast8_t th = t->th;

  /* compute tile coords in the tilemap surface */
  tile -= t->offset;
  uint_fast8_t l2tpr = t->l2tpr;
  uint32_t m = (1 << l2tpr) - 1;
  gfx_pos_t tx = (tile & m) * tw;
  gfx_pos_t ty = (tile >> l2tpr) * th;

  if (center)
    {
      x -= tw >> 1;
      y -= th >> 1;
    }

  const struct gfx_surface_s * __restrict__ ts = &t->s;

  if (gfx_ycheck(ts, ty + th - 1) &&
      /* FIXME perform a single check when called from gfx_draw_tile_string */
      gfx_box_check(s, x, y, x + tw - 1, y + th - 1))
    {
      gfx_blit_nc(s, x, y, ts, tx, ty, tw, th);
      return 1;
    }

  return 0;
}

void
gfx_draw_tile_string(const struct gfx_surface_s * __restrict__ s,
                     const struct gfx_tilemap_s * __restrict__ t,
                     const uint8_t *str, uint_fast16_t size,
                     gfx_pos_t x, gfx_pos_t y, enum gfx_direction_e dir,
                     bool_t center)
{
  gfx_pos_t xd = 0, yd = 0;

  switch (dir)
    {
    case GFX_DIR_LEFT:
      xd = -t->tw;
      break;
    case GFX_DIR_RIGHT:
      xd = t->tw;
      break;
    case GFX_DIR_UP:
      yd = -t->th;
      break;
    case GFX_DIR_DOWN:
      yd = t->th;
      break;
    }

  if (center)
    {
      uint_fast16_t sm1 = size - 1;
      x -= (xd * sm1) >> 1;
      y -= (yd * sm1) >> 1;
    }

  while (size--)
    {
      uint8_t c = *str++;
      gfx_draw_tile(s, t, c, x, y, center);
      x += xd;
      y += yd;
    }
}
