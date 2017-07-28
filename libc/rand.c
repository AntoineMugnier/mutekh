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

static __rand_type_t	random_seed = 1;

/* Use Linear congruential PRNG */
__rand_type_t rand_r(__rand_type_t *seedp)
{
  __rand_type_t	res;

#ifdef CONFIG_LIBC_RAND_LFSR
  __rand_type_t	taps;

  if (sizeof(__rand_type_t) == 1)
    taps = (__rand_type_t)0xb8;
  else if (sizeof(__rand_type_t) == 2)
    taps = (__rand_type_t)0xec83;
  else if (sizeof(__rand_type_t) == 4)
    taps = (__rand_type_t)0xec82f6cbUL;
  else if (sizeof(__rand_type_t) == 8)
    taps = (__rand_type_t)0x81ec82f69eb5a9d3ULL;

  res = (*seedp >> 1) ^ ((__rand_type_t)(-(*seedp & 1)) & taps);
  *seedp = res;
#else
  if (sizeof(__rand_type_t) > 1)
    {
      /* Knuth & Lewis */
      uint32_t	x = *seedp * 1664525 + 1013904223;
      *seedp = x;

      res = (x >> 16) & 0x7fff;
    }
  else
    {
      uint8_t	x = *seedp * 137 + 187;
      *seedp = x;
      res = x;
    }
#endif

  return res;
}

__rand_type_t rand(void)
{
  return rand_r(&random_seed);
}

void srand(__rand_type_t seed)
{
#ifdef CONFIG_LIBC_RAND_LFSR
  if (!seed)
    seed++;
#endif
  random_seed = seed;
}

/* ************************************************** */

__rand_type_t random(void)
{
  /* FIXME */
  return rand();
}

void srandom(__rand_type_t seed)
{
  /* FIXME */
  srand(seed);
}

char *initstate(__rand_type_t seed, char *state, size_t n)
{
  /* FIXME */
  srand(seed);
  return state;
}

char *setstate(char *state)
{
  /* FIXME */
  return state;
}

