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

#include <gfx/gfx.h>

#ifndef _GFX_RECT_H_
#define _GFX_RECT_H_

/* backslash-region-begin */
#define _GFX_RECT_DRAW(_hline, _vline)
{
  y0 = gfx_yaddr(s, y0);
  y1 = gfx_yaddr(s, y1);
  gfx_addr_t dy = gfx_yaddr(s, 1);

  _hline(x0, x1, y0);
  _hline(x0, x1, y1);
  _vline(x0, y0, y1, dy);
  _vline(x1, y0, y1, dy);
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_RECT_R_DRAW(_hline, _vline, _put_pixel)
{
  if (r)
    {
      gfx_pos_t w = x1 - x0;
      gfx_pos_t h = y1 - y0;

      /* clip corner radius */
      if (r > (w >> 1))
        r = w >> 1;
      if (r > (h >> 1))
        r = h >> 1;

      gfx_pos_t x = r, y = 0, m = 0;
      gfx_pos_t e, be, bm;
      gfx_pos_t xa, xb, ya, yb;

      while (y <= x)
        {
          xa = x0 + r - x;
          xb = x1 - r + x;
          ya = gfx_yaddr(s, y0 + r - y);
          yb = gfx_yaddr(s, y1 - r + y);
          _put_pixel(xa, ya);
          _put_pixel(xb, ya);
          _put_pixel(xa, yb);
          _put_pixel(xb, yb);
          xa = x0 + r - y;
          xb = x1 - r + y;
          ya = gfx_yaddr(s, y0 + r - x);
          yb = gfx_yaddr(s, y1 - r + x);
          _put_pixel(xa, ya);
          _put_pixel(xb, ya);
          _put_pixel(xa, yb);
          _put_pixel(xb, yb);

          m = m - (y << 1) - 1;
          e = m + (x << 1) - 1;
          bm = m >> 31;
          be = e >> 31;
          if ((e ^ be) - be < (m ^ bm) - bm)          /* if (abs(e) < abs(m)) */
            {
              m = e;
              x--;
            }
          y++;
        }
    }

  gfx_addr_t dy = gfx_yaddr(s, 1);
  gfx_pos_t y2 = gfx_yaddr(s, y0 + r);
  gfx_pos_t y3 = gfx_yaddr(s, y1 - r);
  y0 = gfx_yaddr(s, y0);
  y1 = gfx_yaddr(s, y1);

  _hline(x0 + r, x1 - r, y0);
  _hline(x0 + r, x1 - r, y1);
  _vline(x0, y2, y3, dy);
  _vline(x1, y2, y3, dy);
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_RECT_FR_DRAW(_frect, _hline)
{
  if (r)
    {
      gfx_pos_t w = x1 - x0;
      gfx_pos_t h = y1 - y0;

      /* clip corner radius */
      if (r > (w >> 1))
        r = w >> 1;
      if (r > (h >> 1))
        r = h >> 1;

      gfx_pos_t x = r, y = 0, m = 0;
      gfx_pos_t e, be, bm;

      while (x >= y)
        {
          gfx_pos_t xa = x0 - x + r;
          gfx_pos_t xb = x1 - r + x;
          _hline(xa, xb, gfx_yaddr(s, y1 - r + y));
          _hline(xa, xb, gfx_yaddr(s, y0 + r - y));

          m = m - (y << 1) - 1;
          e = m + (x << 1) - 1;
          bm = m >> 31;
          be = e >> 31;
          if ((e ^ be) - be < (m ^ bm) - bm)          /* if (abs(e) < abs(m)) */
            {
              m = e;
              x--;
            }
          y++;
        }

      goto dr;
      while (x >= 0)
        {
          m = m + (x << 1) - 1;
          e = m - (y << 1) - 1;
          bm = m >> 31;
          be = e >> 31;
          x--;
          if ((e ^ be) - be < (m ^ bm) - bm)          /* if (abs(e) < abs(m)) */
            {
              m = e;
              y++;
            dr:;
              gfx_pos_t xa = x0 - x + r;
              gfx_pos_t xb = x1 - r + x;
              _hline(xa, xb, gfx_yaddr(s, y1 - r + y));
              _hline(xa, xb, gfx_yaddr(s, y0 + r - y));
            }
        }

      y0 += r + 1;
      y1 -= r;
    }

  /* raw filled rect */
  _frect(x0, x1, gfx_yaddr(s, y0), gfx_yaddr(s, y1), gfx_yaddr(s, 1));
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_RECT_PROTO(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

/** @internal */
void
gfx_draw_rect_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                         gfx_pos_t x0, gfx_pos_t y0,
                         gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a);

/** @internal */
void
gfx_draw_rect_r_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t x0, gfx_pos_t y0,
                           gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a);

/** @internal */
void
gfx_draw_rect_f_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			   gfx_pos_t x0, gfx_pos_t y0,
			   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a);

/** @internal */
void
gfx_draw_rect_fr_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                            gfx_pos_t x0, gfx_pos_t y0,
                            gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a);
/* backslash-region-end */

_GFX_BPP_EXPAND(_GFX_RECT_PROTO);


/* backslash-region-begin */
#define _GFX_RECT_OPS(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

void
gfx_draw_rect_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			   gfx_pos_t x0, gfx_pos_t y0,
			   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a)
{
  _GFX_RECT_DRAW(gfx_hline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS,
		 gfx_vline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS);
}

void
gfx_draw_rect_r_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			     gfx_pos_t x0, gfx_pos_t y0,
			     gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a)
{
  _GFX_RECT_R_DRAW(gfx_hline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS,
		   gfx_vline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS,
		   gfx_put_pixel_nc_##l2bpp _GFX_PUT_PIXEL_ARGS);
}

void
gfx_draw_rect_f_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			   gfx_pos_t x0, gfx_pos_t y0,
			   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a)
{
  gfx_frect_nc_##l2bpp(s, x0, x1,
                       gfx_yaddr(s, y0), gfx_yaddr(s, y1), gfx_yaddr(s, 1), a);
}

void
gfx_draw_rect_fr_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
			      gfx_pos_t x0, gfx_pos_t y0,
			      gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a)
{
  _GFX_RECT_FR_DRAW(gfx_frect_nc_##l2bpp _GFX_PUT_PIXEL_ARGS,
		    gfx_hline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS);
}
/* backslash-region-end */

inline void
gfx_draw_rect_safe(const struct gfx_surface_s * __restrict__ s,
		   gfx_pos_t x0, gfx_pos_t y0,
		   gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a)
{
  if (gfx_box_safe(s, &x0, &y0, &x1, &y1))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_rect_nc, (s, x0, y0, x1, y1, a));
}

inline void
gfx_draw_rect_r_safe(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t y0,
		     gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a)
{
  if (gfx_box_safe(s, &x0, &y0, &x1, &y1))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_rect_r_nc, (s, x0, y0, x1, y1, r, a));
}

inline void
gfx_draw_rect_f_safe(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t x0, gfx_pos_t y0,
		     gfx_pos_t x1, gfx_pos_t y1, gfx_pixel_t a)
{
  if (gfx_box_safe(s, &x0, &y0, &x1, &y1))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_rect_f_nc, (s, x0, y0, x1, y1, a));
}

inline void
gfx_draw_rect_fr_safe(const struct gfx_surface_s * __restrict__ s,
		      gfx_pos_t x0, gfx_pos_t y0,
		      gfx_pos_t x1, gfx_pos_t y1, gfx_pos_t r, gfx_pixel_t a)
{
  if (gfx_box_safe(s, &x0, &y0, &x1, &y1))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_rect_fr_nc, (s, x0, y0, x1, y1, r, a));
}

#endif

