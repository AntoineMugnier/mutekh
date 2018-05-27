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

#ifndef _GFX_CIRCLE_H_
#define _GFX_CIRCLE_H_

#include <gfx/gfx.h>

/* backslash-region-begin */
#define _GFX_CIRCLE_DRAW(_put_pixel, s, xc, yc, r, oct)
{
  gfx_pos_t x = r;
  gfx_pos_t y = 0;
  gfx_pos_t m = 0;
  gfx_pos_t e, be, bm;

  while (x >= y)
    {
      gfx_addr_t ypx = gfx_yaddr(s, yc + x);
      gfx_addr_t ymx = gfx_yaddr(s, yc - x);
      gfx_addr_t ypy = gfx_yaddr(s, yc + y);
      gfx_addr_t ymy = gfx_yaddr(s, yc - y);

      if (oct & 1)
        {
          _put_pixel(xc + x, ypy);
          _put_pixel(xc + y, ypx);
        }
      if (oct & 2)
        {
          _put_pixel(xc - x, ypy);
          _put_pixel(xc - y, ypx);
        }
      if (oct & 4)
        {
          _put_pixel(xc - x, ymy);
          _put_pixel(xc - y, ymx);
        }
      if (oct & 8)
        {
          _put_pixel(xc + x, ymy);
          _put_pixel(xc + y, ymx);
        }

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
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_CIRCLE_INFILL_DRAW(_hline, s, xc, yc, r, oct)
{
  gfx_pos_t x = r, y = 0, m = 0;
  gfx_pos_t e, be, bm;

  while (x >= y)
    {
      gfx_addr_t ypy = gfx_yaddr(s, yc + y);
      gfx_addr_t ymy = gfx_yaddr(s, yc - y);

      if (oct & 1)
        _hline(xc, xc + x, ypy);
      if (oct & 2)
        _hline(xc - x, xc, ypy);
      if (oct & 4)
        _hline(xc - x, xc, ymy);
      if (oct & 8)
        _hline(xc, xc + x, ymy);

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
          gfx_addr_t ypy = gfx_yaddr(s, yc + y);
          gfx_addr_t ymy = gfx_yaddr(s, yc - y);

          if (oct & 1)
            _hline(xc, xc + x, ypy);
          if (oct & 2)
            _hline(xc - x, xc, ypy);
          if (oct & 4)
            _hline(xc - x, xc, ymy);
          if (oct & 8)
            _hline(xc, xc + x, ymy);
        }
    }
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_CIRCLE_OUTFILL_DRAW(_hline, s, xc, yc, r, oct)
{
  gfx_pos_t x = r, y = 0, m = 0;
  gfx_pos_t e, be, bm;

  while (x >= y)
    {
      gfx_addr_t ypy = gfx_yaddr(s, yc + y);
      gfx_addr_t ymy = gfx_yaddr(s, yc - y);

      if (x < r)
        {
          if (oct & 1)
            _hline(xc + x + 1, xc + r, ypy);
          if (oct & 2)
            _hline(xc - r, xc - x - 1, ypy);
          if (oct & 4)
            _hline(xc - r, xc - x - 1, ymy);
          if (oct & 8)
            _hline(xc + x + 1, xc + r, ymy);
        }

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
          gfx_addr_t ypy = gfx_yaddr(s, yc + y);
          gfx_addr_t ymy = gfx_yaddr(s, yc - y);

          if (x < r)
            {
              if (oct & 1)
                _hline(xc + x + 1, xc + r, ypy);
              if (oct & 2)
                _hline(xc - r, xc - x - 1, ypy);
              if (oct & 4)
                _hline(xc - r, xc - x - 1, ymy);
              if (oct & 8)
                _hline(xc + x + 1, xc + r, ymy);
            }
        }
    }
}
/* backslash-region-end */


/* backslash-region-begin */
#define _GFX_CIRCLE_PROTO(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

/** @internal */
void
gfx_draw_circle_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t xc, gfx_pos_t yc,
                           gfx_pos_t r, uint8_t oct, gfx_pixel_t a);

/** @internal */
void
gfx_draw_circle_infill_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                                  gfx_pos_t xc, gfx_pos_t yc,
                                  gfx_pos_t r, uint8_t oct, gfx_pixel_t a);

/** @internal */
void
gfx_draw_circle_outfill_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                                   gfx_pos_t xc, gfx_pos_t yc,
                                   gfx_pos_t r, uint8_t oct, gfx_pixel_t a);
/* backslash-region-end */

_GFX_BPP_EXPAND(_GFX_CIRCLE_PROTO);


/* backslash-region-begin */
#define _GFX_CIRCLE_OPS(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

void
gfx_draw_circle_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                           gfx_pos_t xc, gfx_pos_t yc,
                           gfx_pos_t r, uint8_t oct, gfx_pixel_t a)
{
    _GFX_CIRCLE_DRAW(gfx_put_pixel_nc_##l2bpp _GFX_PUT_PIXEL_ARGS, s, xc, yc, r, oct);
}

void
gfx_draw_circle_infill_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                                  gfx_pos_t xc, gfx_pos_t yc,
                                  gfx_pos_t r, uint8_t oct, gfx_pixel_t a)
{
  if (gfx_circle_check(s, xc, yc, r, oct))
    _GFX_CIRCLE_INFILL_DRAW(gfx_hline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS, s, xc, yc, r, oct);
}

void
gfx_draw_circle_outfill_nc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
                                   gfx_pos_t xc, gfx_pos_t yc,
                                   gfx_pos_t r, uint8_t oct, gfx_pixel_t a)
{
  if (gfx_circle_check(s, xc, yc, r, oct))
    _GFX_CIRCLE_OUTFILL_DRAW(gfx_hline_nc_##l2bpp _GFX_PUT_PIXEL_ARGS, s, xc, yc, r, oct);
}
/* backslash-region-end */


/** @This checks that a circle fits in a surface. */
inline bool_t
gfx_circle_check(const struct gfx_surface_s * __restrict__ s,
                 gfx_pos_t xc, gfx_pos_t yc, /* arc center xy */
                 gfx_pos_t r,                /* radius */
                 uint8_t oct)                /* octant mask */
{
  gfx_pos_t x0 = oct & 6  ? xc - r : xc;
  gfx_pos_t y0 = oct & 12 ? yc - r : yc;
  gfx_pos_t x1 = oct & 9  ? xc + r : xc;
  gfx_pos_t y1 = oct & 3  ? yc + r : yc;

  return gfx_xcheck(s, x0) && gfx_xcheck(s, x1) && gfx_ycheck(s, y0) && gfx_ycheck(s, y1);
}

inline void
gfx_draw_circle_safe(const struct gfx_surface_s * __restrict__ s,
                     gfx_pos_t xc, gfx_pos_t yc,
                     gfx_pos_t r, uint8_t oct, gfx_pixel_t a)
{
  if (gfx_circle_check(s, xc, yc, r, oct))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_circle_nc, (s, xc, yc, r, oct, a));
}

inline void
gfx_draw_circle_infill_safe(const struct gfx_surface_s * __restrict__ s,
                            gfx_pos_t xc, gfx_pos_t yc,
                            gfx_pos_t r, uint8_t oct, gfx_pixel_t a)
{
  if (gfx_circle_check(s, xc, yc, r, oct))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_circle_infill_nc, (s, xc, yc, r, oct, a));
}

inline void
gfx_draw_circle_outfill_safe(const struct gfx_surface_s * __restrict__ s,
                             gfx_pos_t xc, gfx_pos_t yc,
                             gfx_pos_t r, uint8_t oct, gfx_pixel_t a)
{
  if (gfx_circle_check(s, xc, yc, r, oct))
    switch (s->fmt)
      _GFX_FMT_SWITCH(gfx_draw_circle_outfill_nc, (s, xc, yc, r, oct, a));
}

#endif

