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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <stdlib.h>

/********************* standard rand */

__compiler_uint_t __rand_seed = 1;

uint32_t __rand_r32(__compiler_uint_t *s)
{
  __compiler_uint_t r = *s;
  r |= !r;
  r = (-(r & 1) & 0x9eb5a9de) ^ (r >> 1);
  *s = r;
  return r & 0x7fffffff;
}

/********************* rand 64 */

static uint64_t rand_64_seed = 1;

static uint32_t _rand_64_r(uint64_t *s)
{
  uint64_t r = *s;

  /* 64 bits lfsr */
  *s = r = (-(r & 1) & 0x81ec82f69eb5a9d3ULL) ^ (r >> 1);

  /* diffusion, use two 32 x 32 -> 64 mul and xor folding */
  const uint64_t c = 2466808117 /* prime */;
  r = ((uint32_t)r * c) ^ ((uint32_t)(r >> 32) * c);

  return r ^ (r >> 32);
}

static uint32_t _rand_64_range_r(uint64_t *s, uint32_t min, uint32_t max)
{
  uint32_t r = 0;
  uint32_t range = max - min;
  if (range)
    {
      r = _rand_64_r(s);
      if (++range)
        r = (r * (uint64_t)range) >> 32;
    }
  return min + r;
}

void rand_64_merge(const void *data, size_t size)
{
  const uint8_t *d = data;
  uint64_t r = rand_64_seed;

  while (size--)
    {
      /* use a single 32 x 32 -> 64 mul per iteration */
      uint32_t s = r ^ *d++;
      r ^= (uint64_t)s * 16777619 /* fnv prime */;
    }

  rand_64_seed = r | !r;
}

uint32_t rand_64()
{
  return _rand_64_r(&rand_64_seed);
}

uint32_t rand_64_range(uint32_t min, uint32_t max)
{
  return _rand_64_range_r(&rand_64_seed, min, max);
}

uint32_t rand_64_r(uint64_t *s)
{
  *s |= !*s;
  return _rand_64_r(s);
}

uint32_t rand_64_range_r(uint64_t *s, uint32_t min, uint32_t max)
{
  *s |= !*s;
  return _rand_64_range_r(s, min, max);
}
