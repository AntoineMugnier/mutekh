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

#ifndef _GFX_MATH_H_
#define _GFX_MATH_H_

#include <stdint.h>

#define GFX_SQRT2_2_Q0_32 0xb504f334 /* sqrt(2) / 2 in Q0.32 fixed point */

extern const int32_t gfx_sin_table[129];

/** @This returns a Q2.30 signed fixed point. angle period is 512 */
inline int32_t gfx_sin(uint_fast16_t x)
{
  uint32_t neg = -((x >> 8) & 1);
  uint32_t sym = (x >> 7) & 1;
  uint_fast8_t q = ((x ^ -sym) & 127) + sym;
  uint32_t c = gfx_sin_table[q];
  return (c ^ neg) - neg;
}

/** @see gfx_sin */
inline int32_t gfx_cos(uint_fast16_t x)
{
  return gfx_sin(x + 128);
}

inline uint32_t gfx_sqrt32(uint32_t x)
{
  uint_fast8_t n = (31 - __builtin_clz(x)) & ~1;
  uint32_t t, r = 0;

  for (t = 1 << n; t; t = t >> 2)
    {
      uint32_t q = r + t;
      r = r >> 1;
      if (x >= q)
	{
	  x -= q;
	  r += t;
	}
    }

  /* rounding */
  if (x > r)
    r++;

  return r;
}

#endif

