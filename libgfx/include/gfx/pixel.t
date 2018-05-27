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

#ifndef _GFX_PIXEL_H_
#define _GFX_PIXEL_H_

#include <string.h>
#include <gfx/gfx.h>

/** @internal */
#define _GFX_PUT_PIXEL_ARGS(...) (s, __VA_ARGS__, a)

/** @internal Declare and precompute variables for horizontal line loop */
/* backslash-region-begin */
#define _GFX_HLINE_VARS(word_t, l2bpp, l2ppw, x0, x1, x0i, x1i, k0, m0, m1)
gfx_addr_t x0i, x1i;
word_t m0, m1;
bool_t k0;
{
  uint_fast8_t bpw = 8 * sizeof(word_t);
  gfx_pos_t x1_ = x1 + 1;

  gfx_addr_t x0f = (x0 << l2bpp) & (bpw - 1);
  x0i = x0 >> l2ppw;
  m0 = (1ULL << x0f) - 1;

  gfx_addr_t x1f = (x1_ << l2bpp) & (bpw - 1);
  x1i = x1_ >> l2ppw;
  m1 = (1ULL << x1f) - 1;

  k0 = x1i > x0i;
  if (!k0)
    m1 = m1 & ~m0;
}
/* backslash-region-end */

/** @internal Horizontal line loop code */
/* backslash-region-begin */
#define _GFX_HLINE_LOOP(d, x0i, x1i, k0, m0, m1, v)
{
  gfx_addr_t x0j = x0i;
  if (k0)
    {
      if (m0)
        {
          d[x0j] = (d[x0j] & m0) | (v & ~m0);
          x0j++;
        }

      while (x0j < x1i)
        d[x0j++] = v;
    }

  if (m1)
    d[x0j] = (d[x0j] & ~m1) | (v & m1);
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_PIXEL_PROTO(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

/** @internal Change a pixel on a bpp bits surface, wrap pixel address
    on position overflow */
void
gfx_put_pixel_safe_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t x, gfx_addr_t yw, gfx_pixel_t a);

/** @internal Change a pixel on a bpp bits surface, generate bad
    memory access on position overflow */
void
gfx_put_pixel_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                         gfx_pos_t x, gfx_addr_t yw,
			 gfx_pixel_t a);

/** @internal Get pixel value on a bpp bits surface. Wrap pixel
    address on position overflow. */
gfx_pixel_t
gfx_get_pixel_safe_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t x, gfx_addr_t yw);

/** @internal Get pixel value on a bpp bits surface. Generate bad
    memory access on position overflow. */
gfx_pixel_t
gfx_get_pixel_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                         gfx_pos_t x, gfx_addr_t yw);

/** @internal Change a pixels along a vertical line on a bpp bits surface. */
void
gfx_vline_safe_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                       gfx_pos_t x, gfx_addr_t y0w, gfx_addr_t y1w,
		       gfx_addr_t ybw, gfx_pixel_t a);

/** @internal Change a pixels along an horizontal line on a bpp bits
    surface. Generate bad memory access on position overflow. */
void
gfx_hline_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                     gfx_pos_t x0, gfx_pos_t x1,
		     gfx_addr_t yw, gfx_pixel_t a);

/** @internal Change a pixels along a vertical line on a bpp bits
    surface. Generate bad memory access on position overflow. */
void
gfx_vline_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x, gfx_addr_t y0w, gfx_addr_t y1w,
		     uint_fast32_t ybw, gfx_pixel_t a);

/** @internal fill a rectangular area.
    Generate bad memory access on position overflow. */
void
gfx_frect_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t x1,
                     gfx_pos_t y0w, gfx_pos_t y1w,
                     gfx_pos_t ybw, gfx_pixel_t a);

void
gfx_clear_##l2bpp(const struct gfx_surface_s * __restrict__ s, gfx_pixel_t a);
/* backslash-region-end */

_GFX_BPP_EXPAND(_GFX_PIXEL_PROTO);


/* backslash-region-begin */
#define _GFX_PIXEL_OPS(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

extern inline void
gfx_put_pixel_safe_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t x, gfx_addr_t yw, gfx_pixel_t a)
{
  word_t * __restrict__ d = s->ptr;
  uint_fast8_t sh = (x & (ppw - 1)) << l2bpp;
  word_t mask = (pm << sh);
  gfx_addr_t i = (x >> l2ppw) + yw;
  word_t *p = d + gfx_xymod(s, i);
  *p = (*p & ~mask) | (a << sh);
}

extern inline void
gfx_put_pixel_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                         gfx_pos_t x, gfx_addr_t yw,
			 gfx_pixel_t a)
{
  word_t * __restrict__ d = s->ptr;
  uint_fast8_t sh = (x & (ppw - 1)) << l2bpp;
  word_t mask = (pm << sh);
  gfx_addr_t i = (x >> l2ppw) + yw;
  word_t *p = d + i;
  *p = (*p & ~mask) | (a << sh);
}

extern inline gfx_pixel_t
gfx_get_pixel_safe_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t x, gfx_addr_t yw)
{
  const word_t *d = s->ptr;
  uint_fast8_t sh = (x & (ppw - 1)) << l2bpp;
  gfx_addr_t i = (x >> l2ppw) + yw;
  const word_t *p = d + gfx_xymod(s, i);
  return (*p >> sh) & pm;
}

extern inline gfx_pixel_t
gfx_get_pixel_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                         gfx_pos_t x, gfx_addr_t yw)
{
  const word_t *d = s->ptr;
  uint_fast8_t sh = (x & (ppw - 1)) << l2bpp;
  gfx_addr_t i = (x >> l2ppw) + yw;
  const word_t *p = d + i;
  return (*p >> sh) & pm;
}

extern inline void
gfx_hline_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t x1,
		     gfx_addr_t yw, gfx_pixel_t a)
{
  GFX_ASSERT(x0 <= x1);
  word_t * __restrict__ d = (word_t*)s->ptr + yw;
  word_t v = a * ps;
  _GFX_HLINE_VARS(word_t, l2bpp, l2ppw, x0, x1, x0i, x1i, k0, m0, m1);
  _GFX_HLINE_LOOP(d, x0i, x1i, k0, m0, m1, v);
}

extern inline void
gfx_vline_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x, gfx_addr_t y0w, gfx_addr_t y1w,
		     uint_fast32_t ybw, gfx_pixel_t a)
{
  while (y0w <= y1w)
    {
      gfx_put_pixel_nc_##l2bpp(s, x, y0w, a);
      y0w += ybw;
    }
}

extern inline void
gfx_frect_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t x1,
                     gfx_pos_t y0w, gfx_pos_t y1w,
                     gfx_pos_t ybw, gfx_pixel_t a)
{
  word_t * __restrict__ d = s->ptr;
  word_t v = a * ps;
  _GFX_HLINE_VARS(word_t, l2bpp, l2ppw, x0, x1, x0i, x1i, k0, m0, m1);

  while (y0w <= y1w)
    {
      word_t *e = d + y0w;
      _GFX_HLINE_LOOP(e, x0i, x1i, k0, m0, m1, v);
      y0w += ybw;
    }
}

void
gfx_clear_##l2bpp(const struct gfx_surface_s * __restrict__ s, gfx_pixel_t a)
{
  word_t v = a * ps;
  word_t *d = s->ptr;
  size_t c = gfx_yaddr(s, gfx_height(s));

  if (bpp <= 8)
    {
      memset(d, v, sizeof(word_t) * c);
    }
  else
    {
      word_t *e = d + c;
      while (d < e)
        *d++ = v;
    }
}
/* backslash-region-end */

inline gfx_pixel_t
gfx_get_pixel_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x, gfx_pos_t y)
{
  switch (s->fmt)
    _GFX_FMT_SWITCH(gfx_get_pixel_safe, (s, x, gfx_yaddr(s, y)));
}

inline void
gfx_put_pixel_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x, gfx_pos_t y, gfx_pixel_t a)
{
  switch (s->fmt)
    _GFX_FMT_SWITCH(gfx_put_pixel_safe, (s, x, gfx_yaddr(s, y), a));
}

inline void
gfx_hline_safe(const struct gfx_surface_s * __restrict__ s,
	       gfx_pos_t x0, gfx_pos_t x1, gfx_pos_t y, gfx_pixel_t a)
{
  if (gfx_xcheck(s, x0) && gfx_xcheck(s, x1) && gfx_ycheck(s, y))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_hline_nc, (s, x0, x1, gfx_yaddr(s, y), a));
}

inline void
gfx_vline_safe(const struct gfx_surface_s * __restrict__ s,
	       gfx_pos_t x, gfx_pos_t y0, gfx_pos_t y1, gfx_pixel_t a)
{
  if (gfx_xcheck(s, 0) && gfx_ycheck(s, y0) && gfx_ycheck(s, y1))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_vline_nc,
                      (s, x, gfx_yaddr(s, y0),
                       gfx_yaddr(s, y1), gfx_yaddr(s, 1), a));
}

inline void
gfx_clear(const struct gfx_surface_s * __restrict__ s, gfx_pixel_t a)
{
  switch (s->fmt)
    _GFX_FMT_SWITCH(gfx_clear, (s, a));
}

#endif

