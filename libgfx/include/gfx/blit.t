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

#ifndef _GFX_BLIT_H_
#define _GFX_BLIT_H_

#include <gfx/gfx.h>

/* backslash-region-begin */
#define _GFX_BLIT_ROW_FWD(word_t, get_src, get_dst, put_dst, blend, k2, e, d)
{
  word_t v, o, p;

  o = get_dst(x2j) & ~m1;

  if (k1)
    {
      v = get_src(x0j++);
      o |= (v >> e) & m1;
    }

  if (k0)
    {
      v = get_src(x0j++);
      if (k2)
	o |= (v << d) & m1;
      o = blend(o, get_dst(x2j));
      put_dst(x2j++, o);
      o = v >> e;

      while (x0j < x1i)
	{
	  v = get_src(x0j++);
	  if (k2)
	    o |= v << d;
	  o = blend(o, get_dst(x2j));
	  put_dst(x2j++, o);
	  o = v >> e;
	}
    }

  v = get_src(x0j);
  if (k2)
    o |= (v << d) & m2;

  if (x3i > x2j)
    {
      o = blend(o, get_dst(x2j));
      put_dst(x2j++, o);
      o = v >> e;
    }

  if (x3f)
    {
      p = get_dst(x2j);
      o = blend(o, p);
      put_dst(x2j, (o & m3) | (p & ~m3));
    }
}
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_BLIT_ROW_BWD(word_t, get_src, get_dst, put_dst, blend, k2, e, d)
{
  word_t v = v, o, p;

  o = get_dst(x2j) & ~m1;

  if (k3)
    {
      v = get_src(x0j--);
      o |= (v << e) & m1;
    }

  if (k0)
    {
      v = get_src(x0j--);
      if (k2)
	o |= (v >> d) & m1;
      o = blend(o, get_dst(x2j));
      put_dst(x2j--, o);
      o = v << e;

      while (x0j > x1i)
	{
	  v = get_src(x0j--);
	  if (k2)
	    o |= v >> d;
	  o = blend(o, get_dst(x2j));
	  put_dst(x2j--, o);
	  o = v << e;
	}
    }

  if (x0j >= 0)
    v = get_src(x0j);

  if (k2)
    o |= (v >> d) & m2;

  if (x3i < x2j)
    {
      o = blend(o, get_dst(x2j));
      put_dst(x2j--, o);
      o = v << e;
    }

  if (x3f)
    {
      p = get_dst(x2j);
      o = blend(o, p);
      put_dst(x2j, (o & m3) | (p & ~m3));
    }
}
/* backslash-region-end */

#ifdef CONFIG_GFX_UNROLL
/* backslash-region-begin */
# define _GFX_BLIT_ROWS(dir, word_t, get_src, get_dst, put_dst, blend)
if (e)
  {
    while (ys != yse)
      {
	int32_t x0j = x0i, x2j = x2i;
	_GFX_BLIT_ROW_##dir(word_t, get_src, get_dst, put_dst, blend, 1, e, d);
	yd += ydw;
	ys += ysw;
      }
  }
else
  {
    while (ys != yse)
      {
	int32_t x0j = x0i, x2j = x2i;
	_GFX_BLIT_ROW_##dir(word_t, get_src, get_dst, put_dst, blend, 0, 0, 0);
	yd += ydw;
	ys += ysw;
      }
  }
/* backslash-region-end */
#else
/* backslash-region-begin */
# define _GFX_BLIT_ROWS(dir, word_t, get_src, get_dst, put_dst, blend)
while (ys != yse)
  {
    int32_t x0j = x0i, x2j = x2i;
    _GFX_BLIT_ROW_##dir(word_t, get_src, get_dst, put_dst, blend, e, e, d);
    yd += ydw;
    ys += ysw;
  }
/* backslash-region-end */
#endif

/* backslash-region-begin */
#define _GFX_BLIT_INIT_FWD(word_t, l2bpp, l2ppw)

  /* bits per word */
  uint_fast8_t bpw = 8 * sizeof(word_t);

  /* compute word index in row */
  int32_t x0i = x0 >> l2ppw;
  int32_t x1i = x1 >> l2ppw;
  int32_t x2i = x2 >> l2ppw;
  int32_t x3i = x3 >> l2ppw;
  bool_t k0 = x3i != x2i;

  /* compute bit index in word */
  uint_fast8_t x0f, x1f, x2f, x3f;
  uint_fast8_t d, e;
  bool_t k1;

  if (l2ppw > 0)
    {
      x0f = (x0 << l2bpp) & (bpw - 1);
      x1f = (x1 << l2bpp) & (bpw - 1);
      x2f = (x2 << l2bpp) & (bpw - 1);
      x3f = (x3 << l2bpp) & (bpw - 1);

      k1 = x2f <= x0f;
      if (k1)
	{
	  e = x0f - x2f;
	  d = bpw - e;
	}
      else
	{
	  d = x2f - x0f;
	  e = bpw - d;
	}
    }
  else
    {
      x0f = x1f = x2f = x3f = 0;
      k1 = 1;
      d = e = 0;
    }

  /* precompute word masks */
  word_t m1 = (word_t)0xffffffffU;
  m1 = m1 << x2f;

  word_t m2 = m1;
  if (k1 || k0)
    m2 = (word_t)0xffffffffU;

  word_t m3 = (word_t)0xffffffffU;
  m3 = m3 >> ((bpw - x3f) & (bpw - 1));
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_BLIT_INIT_BWD(word_t, l2bpp, l2ppw)

  /* bits per word */
  uint_fast8_t bpw = 8 * sizeof(word_t);

  /* compute word index in row */
  int32_t x0i = (x0 - 1) >> l2ppw;
  int32_t x1i = (x1 - 1) >> l2ppw;
  int32_t x2i = (x2 - 1) >> l2ppw;
  int32_t x3i = (x3 - 1) >> l2ppw;
  bool_t k0 = x3i != x2i;

  /* compute bit index in word */
  uint32_t x0f, x1f, x2f, x3f;
  uint_fast8_t d, e;
  bool_t k1, k3;

  if (l2ppw > 0)
    {
      x0f = (x0 << l2bpp) & (bpw - 1);
      x1f = (x1 << l2bpp) & (bpw - 1);
      x2f = (x2 << l2bpp) & (bpw - 1);
      x3f = (x3 << l2bpp) & (bpw - 1);

      k1 = x2f >= x0f;
      k3 = (x2f - 1) >= (x0f - 1);
      if (k1)
	{
	  e = x2f - x0f;
	  d = bpw - e;
	}
      else
	{
	  d = x0f - x2f;
	  e = bpw - d;
	}
    }
  else
    {
      x0f = x1f = x2f = x3f = 0;
      k3 = k1 = 1;
      d = e = 0;
    }

  /* precompute word masks */
  word_t m1 = (word_t)0xffffffffU;
  m1 = m1 >> ((bpw - x2f) & (bpw - 1));

  word_t m2 = m1;
  if (k1 || k0)
    m2 = (word_t)0xffffffffU;

  word_t m3 = (word_t)0xffffffffU;
  m3 = m3 << x3f;
/* backslash-region-end */

/* backslash-region-begin */
#define _GFX_BLIT_PROTO(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

/** @internal */
void
gfx_blit_nc_##l2bpp(const struct gfx_surface_s * __restrict__ d,
		    gfx_pos_t x2, gfx_pos_t y2, /* dest */
		    const struct gfx_surface_s * __restrict__ s,
		    gfx_pos_t x0, gfx_pos_t y0, /* src */
		    gfx_pos_t w, gfx_pos_t h);

/** @internal */
void
gfx_blit_overlap_nc_##l2bpp(const struct gfx_surface_s * __restrict__ d,
			    gfx_pos_t x2, gfx_pos_t y2, /* dest */
			    gfx_pos_t x0, gfx_pos_t y0, /* src */
			    gfx_pos_t w, gfx_pos_t h);
/* backslash-region-end */

_GFX_BPP_EXPAND(_GFX_BLIT_PROTO);


#define _GFX_BLIT_GET_SRC(x) ys[x]
#define _GFX_BLIT_GET_DST(x) yd[x]
#define _GFX_BLIT_PUT_DST(x, v) (yd[x] = (v))
#define _GFX_BLIT_BLEND(v, old) (v)

/* backslash-region-begin */
#define _GFX_BLIT_OPS(bpp, ppw, l2bpp, l2ppw, pm, ps, word_t)

void
gfx_blit_nc_##l2bpp(const struct gfx_surface_s * __restrict__ d,
		    gfx_pos_t x2, gfx_pos_t y2, /* dest */
		    const struct gfx_surface_s * __restrict__ s,
		    gfx_pos_t x0, gfx_pos_t y0, /* src */
		    gfx_pos_t w, gfx_pos_t h)
{
  gfx_pos_t x1 = x0 + w;
  gfx_pos_t y1 = y0 + h;
  gfx_pos_t x3 = x2 + w;

  const word_t *ys = (word_t*)s->ptr + gfx_yaddr(s, y0);
  const word_t *yse = (word_t*)s->ptr + gfx_yaddr(s, y1);
  word_t *yd = (word_t*)d->ptr + gfx_yaddr(d, y2);
  gfx_addr_t ysw = gfx_yaddr(s, 1);
  gfx_addr_t ydw = gfx_yaddr(d, 1);

  {
    _GFX_BLIT_INIT_FWD(word_t, l2bpp, l2ppw);
    _GFX_BLIT_ROWS(FWD, word_t, _GFX_BLIT_GET_SRC,
		   _GFX_BLIT_GET_DST, _GFX_BLIT_PUT_DST, _GFX_BLIT_BLEND);
  }
}

void
gfx_blit_overlap_nc_##l2bpp(const struct gfx_surface_s * __restrict__ d,
			    gfx_pos_t x2, gfx_pos_t y2, /* dest */
			    gfx_pos_t x0, gfx_pos_t y0, /* src */
			    gfx_pos_t w, gfx_pos_t h)
{
  gfx_pos_t x1 = x0 + w;
  gfx_pos_t y1 = y0 + h;
  gfx_pos_t x3 = x2 + w;
  gfx_pos_t y3 = y2 + h;

  const word_t *ys, *yse;
  word_t *yd;
  gfx_pos_t ysw, ydw;

  if (y2 > y0)
    {
      ysw = ydw = -gfx_yaddr(d, 1);
      ys = (word_t*)d->ptr + gfx_yaddr(d, y1) + ysw;
      yse = (word_t*)d->ptr + gfx_yaddr(d, y0) + ysw;
      yd = (word_t*)d->ptr + gfx_yaddr(d, y3) + ydw;
    }
  else
    {
      ysw = ydw = gfx_yaddr(d, 1);
      ys = (word_t*)d->ptr + gfx_yaddr(d, y0);
      yse = (word_t*)d->ptr + gfx_yaddr(d, y1);
      yd = (word_t*)d->ptr + gfx_yaddr(d, y2);
    }

  if (x0 <= x1)
    {
      _GFX_BLIT_INIT_FWD(word_t, l2bpp, l2ppw);
      _GFX_BLIT_ROWS(FWD, word_t, _GFX_BLIT_GET_SRC,
		     _GFX_BLIT_GET_DST, _GFX_BLIT_PUT_DST, _GFX_BLIT_BLEND);
    }
  else
    {
      _GFX_SWAP(x0, x1);
      _GFX_SWAP(x3, x2);
      _GFX_BLIT_INIT_BWD(word_t, l2bpp, l2ppw);
      _GFX_BLIT_ROWS(BWD, word_t, _GFX_BLIT_GET_SRC,
		     _GFX_BLIT_GET_DST, _GFX_BLIT_PUT_DST, _GFX_BLIT_BLEND);
    }
}
/* backslash-region-end */

inline void
gfx_blit_nc(const struct gfx_surface_s * __restrict__ d,
	    gfx_pos_t x2, gfx_pos_t y2, /* dest */
	    const struct gfx_surface_s * __restrict__ s,
	    gfx_pos_t x0, gfx_pos_t y0, /* src */
	    gfx_pos_t w, gfx_pos_t h)
{
  if (d->fmt != s->fmt)
    return;			/* FIXME not supported yet */

  switch (d->fmt)
    _GFX_FMT_SWITCH(gfx_blit_nc, (d, x2, y2, s, x0, y0, w, h));
}

inline void
gfx_blit_safe(const struct gfx_surface_s * __restrict__ d,
	      gfx_pos_t x2, gfx_pos_t y2, /* dest */
	      const struct gfx_surface_s * __restrict__ s,
	      gfx_pos_t x0, gfx_pos_t y0, /* src */
	      gfx_pos_t w, gfx_pos_t h)
{
  w &= 0xffff;
  h &= 0xffff;
  gfx_pos_t x1 = x0 + w - 1;
  gfx_pos_t y1 = y0 + h - 1;
  gfx_pos_t x3 = x2 + w - 1;
  gfx_pos_t y3 = y2 + h - 1;

  if (gfx_box_check(s, x0, y0, x1, y1) &&
      gfx_box_check(d, x2, y2, x3, y3))
    gfx_blit_nc(d, x2, y2, s, x0, y0, w, h);
}

inline void
gfx_blit_overlap_nc(const struct gfx_surface_s * __restrict__ d,
		    gfx_pos_t x2, gfx_pos_t y2, /* dest */
		    gfx_pos_t x0, gfx_pos_t y0, /* src */
		    gfx_pos_t w, gfx_pos_t h)
{
  switch (d->fmt)
    _GFX_FMT_SWITCH(gfx_blit_overlap_nc, (d, x2, y2, x0, y0, w, h));
}

inline void
gfx_blit_overlap_safe(const struct gfx_surface_s * __restrict__ d,
		      gfx_pos_t x2, gfx_pos_t y2, /* dest */
		      gfx_pos_t x0, gfx_pos_t y0, /* src */
		      gfx_pos_t w, gfx_pos_t h)
{
  w &= 0xffff;
  h &= 0xffff;
  gfx_pos_t x1 = x0 + w - 1;
  gfx_pos_t y1 = y0 + h - 1;
  gfx_pos_t x3 = x2 + w - 1;
  gfx_pos_t y3 = y2 + h - 1;

  if (gfx_box_check(d, x0, y0, x1, y1) &&
      gfx_box_check(d, x2, y2, x3, y3))
    gfx_blit_overlap_nc(d, x2, y2, x0, y0, w, h);
}

#endif

