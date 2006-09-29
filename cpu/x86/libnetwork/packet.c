/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <hexo/types.h>

uint_fast16_t		packet_checksum(void		*data,
					size_t		size)
{
  uint32_t		result;

  size >>= 2;

  asm ("cld\n\t"
       "xorl %0, %0\n\t"
       "1:\n\t"
       "lodsl\n\t"
       "adcl %%eax, %0\n\t"
       "loopnz 1b\n\t"
       : "=b" (result)
       : "c" (size), "S" (data));

#if 0
  result = (result >> 16) + (result & 0xffff);
  result = (result >> 16) + result;

  return (~result) & 0xffff;
#else
  return result;
#endif
}
