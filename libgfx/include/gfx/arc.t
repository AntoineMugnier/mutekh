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

#ifndef _GFX_ARC_H_
#define _GFX_ARC_H_

#include <gfx/gfx.h>

/*

  This code draws an arc with center at (xc, yc), starting at (x,
  y) and ending at (xt, yt) with radius r. The function never
  terminates when xt and yt are both greater than radius / sqrt(2).

  The algorithm works by splitting the drawing plane in 8 octants:

   \         |         /
     \  o5   | o6    /
       \     |     /
   o4    \   |   /   o7
           \ | /
------------ X -----------> x
           / | \
   o3    /   |   \   o0
       /     |     \
     /  o2   | o1    \
   /         |         \
             v y

  The difference between squared arc radius and squared distance
  from current pixel center to arc center is computed:

  err = x*x + y*y - r*r

  The algorithm chooses the next pixel so that the error remains
  small. The error is updated when moving to the next pixel.

  In order to simplify computations of derr_x, the sign of the squared
  difference is reversed between o0, o1, o4, o5 and o2, o3, o4,
  o5. This is possible because the error is always 0 when x=0 or y=0,
  that is when switching from o7, o1, o3, o5 to o0, o2, o4, o6. The
  following formula is used to update the error:

  derr_x = (2*x + dx)
  derr_y = (2*y * dy*dx + dx)

In CW direction:

  o0, o1: dx=-1, dy=+1, derr_x=2x-1, derr_y=-2y-1
  o2, o3: dx=-1, dy=-1, derr_x=2x-1, derr_y=+2y-1
  o4, o5: dx=+1, dy=-1, derr_x=2x+1, derr_y=-2y+1
  o6, o7: dx=+1, dy=+1, derr_x=2x+1, derr_y=+2y+1

*/

/* backslash-region-begin */
#define _GFX_ARC_DRAW(_put_pixel, s, xc, yx, x, y, xt, yt, r2, ccw)
{

  if (!r2)
    return;

  /* squared error */
  int32_t m = r2 - x*x - y*y;

  if ((y > 0) ^ ccw)
    m = -m;

  /* direction */
  ccw = !ccw - 1;

  int32_t dx, dy, dxdy, e, be, bm, t;

  while (1)
    {
      /* compute the x stop value. when yt and y have different sign,
	 the value of t is so large that it never matches. */
      t = xt ^ ((yt ^ y) & 0x80000000);

      while (1)			/* used in o1, o2, o5, o6 */
        {
          _put_pixel(xc + x, gfx_yaddr(s, yc + y));

          if (x == t)		/* check stop condition */
            return;

	  /* initialy use dx and dy variables as masks for abs() */
          dx = y >> 31;
          dy = x >> 31;

	  /* switch to other loop */
          if ((x ^ dy) - dy > (y ^ dx) - dx)	  /* if abs(x) > abs(y) */
            break;

	  /* compute actual values of dx and dy */
          dxdy = dx ^ dy;
          dx ^= ccw;
          dy ^= ~ccw;
          dx = dx | 1;  /* make dx and dy either 1 or -1 */
          dy = dy | 1;

	  /* update the error with move along x axis (apply derr_x) */
          m = m + (x << 1) + dx;
	  /* compute the error when we move along y too (apply derr_y) */
          e = m - (y << 1 ^ dxdy) + dxdy + dx;

	  /* retained move depends on lowest resulting error */
          bm = m >> 31;
          be = e >> 31;
          if ((e ^ be) - be < (m ^ bm) - bm)	  /* if (abs(e) < abs(m)) */
            {
	      /* keep smallest error and move y */
              m = e;
              y += dy;
            }
	  /* move x */
          x += dx;
        }

      t = yt ^ ((xt ^ x) & 0x80000000);

      while (1)			/* used in o0, o3, o4, o7 */
        {
          _put_pixel(xc + x, gfx_yaddr(s, yc + y));

          if (y == t)
            return;

          dx = y >> 31;
          dy = x >> 31;

          if ((x ^ dy) - dy < (y ^ dx) - dx)
            break;

          dxdy = dx ^ dy;
          dx ^= ccw;
          dy ^= ~ccw;
          dx = dx | 1;
          dy = dy | 1;

          m = m - (y << 1 ^ dxdy) + dxdy + dx;
          e = m + (x << 1) + dx;

          bm = m >> 31;
          be = e >> 31;
          if ((e ^ be) - be < (m ^ bm) - bm)
            {
              m = e;
              x += dx;
            }
          y += dy;
        }
    }
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_ARC_PROTO(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

/** @internal */
void
gfx_draw_arc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t xc, gfx_pos_t yc, /* arc center xy */
		     gfx_pos_t x, gfx_pos_t y, /* arc start xy relative to center */
		     gfx_pos_t xt, gfx_pos_t yt, /* arc end xy relative to center */
		     uint32_t r2,		 /* squared radius */
		     uint32_t ccw,               /* direction boolean */
		     gfx_pixel_t a);
/* backslash-region-end */

_GFX_BPP_EXPAND(_GFX_ARC_PROTO);


/* backslash-region-begin */
#define _GFX_ARC_OPS(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

void
gfx_draw_arc_##l2bpp(const struct gfx_surface_s * __restrict__ s,
		     gfx_pos_t xc, gfx_pos_t yc,
		     gfx_pos_t x, gfx_pos_t y,
		     gfx_pos_t xt, gfx_pos_t yt,
		     uint32_t r2, uint32_t ccw, gfx_pixel_t a)
{
  _GFX_ARC_DRAW(gfx_put_pixel_safe_##l2bpp _GFX_PUT_PIXEL_ARGS, s, xc, yc, x, y, xt, yt, r2, ccw);
}
/* backslash-region-end */

inline void
gfx_draw_arc_angles_safe(const struct gfx_surface_s *s,
                         uint_fast16_t xc, uint_fast16_t yc,
                         uint_fast16_t a0, uint_fast16_t a1,
                         uint_fast16_t r, uint32_t ccw, gfx_pixel_t a)
{
  a0 &= 511;
  a1 &= 511;

  if (r > 4096)
    return;

  int32_t x0 = ((int64_t)gfx_cos(a0) * r) >> 30;
  int32_t y0 = ((int64_t)gfx_sin(a0) * r) >> 30;

  int32_t x1 = ((int64_t)gfx_cos(a1) * r) >> 30;
  int32_t y1 = ((int64_t)gfx_sin(a1) * r) >> 30;

  uint32_t r2 = r*r;

  switch (s->fmt)
    _GFX_FMT_SWITCH(gfx_draw_arc, (s, xc, yc, x0, y0, x1, y1, r2, ccw, a));
}

inline void
gfx_draw_arc_xy_safe(const struct gfx_surface_s *s,
                     int32_t x0, int32_t y0,
                     int32_t x1, int32_t y1,
                     uint_fast16_t r, uint32_t ccw, gfx_pixel_t a)
{
#if 0
  if (r > 4096)
    return;

  int32_t r2 = r*r;

  /* middle point */
  int32_t xm = (x0 + x1) >> 1;
  int32_t ym = (y0 + y1) >> 1;

  /* vector from middle point to possible arc center. we have to
     rescale this vector to match the radius. */
  int32_t xp = ym - y0;
  int32_t yp = x0 - xm;

  int32_t dx = xp;
  int32_t dy = yp;
  int32_t d2 = dx*dx + dy*dy;

  uint_fast8_t k = __builtin_clz(dx | dy) - 1;
  dx <<= k;
  dy <<= k;

  printf("dx %u dx %u k %u\n", dx, dy, k);

  unsigned n = 50;
  while (n--)
    {
      /* we want to rescale (xp, yp) so that m is small */
      int32_t m = r2 - d2 - xp*xp - yp*yp;

      printf("m %-5i %-5u %-5u %u\n", m, xp, yp, n);
      gfx_put_pixel_safe_##l2bpp(s, xm + xp, ym + yp, a);

      int32_t kx, ky;
      if (m > 0)
	{
	  int_fast8_t l = 32 - (32 - __builtin_clz(m)) / 2;
	  printf(" l %i\n", l);
	  kx = dx >> l;
	  ky = dy >> l;
	}
      else
	{
	  m = -m;
	  int_fast8_t l = 32 - (32 - __builtin_clz(m)) / 2;
	  printf(" l %i\n", l);
	  kx = -dx >> l;
	  ky = -dy >> l;
	}
      if (!kx && !ky)
	break;
      printf(" x %i %i\n", xp, kx);
      printf(" y %i %i\n", yp, ky);
      xp += kx;
      yp += ky;
    }

  gfx_put_pixel_safe_##l2bpp(s, x0, y0, 0xff);
  gfx_put_pixel_safe_##l2bpp(s, x1, y1, 0xff);

  int32_t xc = xm + xp;
  int32_t yc = ym + yp;

  printf("r2 %u\n", r2);

  r2 = xp*xp + yp*yp;

  switch (s->fmt)
    _GFX_FMT_SWITCH(gfx_draw_arc, (s, xc, yc,
				   x0 - xc, y0 - yc,
				   x1 - xc, y1 - yc,
				   r2, ccw, a));
#endif
}

#endif

