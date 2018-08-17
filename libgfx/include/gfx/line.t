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

/**
   @file
   @module {Libraries::Lightweight 2d graphic}
*/

#ifndef _GFX_LINE_H_
#define _GFX_LINE_H_

#include <gfx/gfx.h>

/* backslash-region-begin */
#define _GFX_LINE_DRAW(_put_pixel, s, x0, x1, y0, y1)
{
  const uint_fast8_t bits = sizeof(gfx_pos_t) * 8 - 1;
  gfx_pos_t dx = x1 - x0;
  gfx_pos_t dy = y1 - y0;
  gfx_pos_t sx = dx >> bits; /* sx = dx < 0 ? -1 : 0 */
  gfx_pos_t sy = dy >> bits;
  dx = ((2 * dx) ^ sx) - sx;    /* dx = dx < 0 ? 2 * -dx : 2 * dx */
  dy = ((2 * dy) ^ sy) - sy;
  sx |= 1;                      /* if (sx != -1) sx = 1 */
  sy = gfx_yaddr(s, sy | 1);

  gfx_pos_t x = x0;
  gfx_addr_t yw = gfx_yaddr(s, y0);

  if (dx > dy)
    {
      gfx_pos_t d = 2 * dy - dx;
      while (1)
	{
	  _put_pixel(x, yw);
	  if (x == x1)
	    break;
	  gfx_pos_t m = ~(d >> bits);
	  d += dy - (dx & m);
	  x += sx;
	  yw += sy & m;
	}
    }
  else
    {
      gfx_pos_t d = 2 * dx - dy;
      gfx_addr_t ye = gfx_yaddr(s, y1);
      while (1)
	{
	  _put_pixel(x, yw);
	  if (yw == ye)
	    break;
	  gfx_pos_t m = ~(d >> bits);
	  d += dx - (dy & m);
	  x += sx & m;
	  yw += sy;
	}
    }
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_LINE_PROTO(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

/** @internal */
void
gfx_draw_line_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			 gfx_pos_t x0, gfx_pos_t y0,
			 gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a);
/* backslash-region-end */

_GFX_BPP_EXPAND(_GFX_LINE_PROTO);


/* backslash-region-begin */
#define _GFX_LINE_OPS(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

void
gfx_draw_line_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			   gfx_pos_t x0, gfx_pos_t y0,
			   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a)
{
  _GFX_LINE_DRAW(gfx_put_pixel_nc_##l2bpp _GFX_PUT_PIXEL_ARGS,
		 s, x0, x1, y0, y1);
}
/* backslash-region-end */


inline void
gfx_draw_line_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x0, gfx_pos_t y0,
		   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a)
{
  if (gfx_box_safe(s, &x0, &y0, &x1, &y1))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_line_nc, (s, x0, y0, x1, y1, a));
}

#endif
