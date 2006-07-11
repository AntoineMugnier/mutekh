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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef NETINET_IN_H_
#define NETINET_IN_H_

#include <hexo/endian.h>

static inline uint32_t htonl(uint32_t x)
{
  return endian_be32(x);
}

static inline uint16_t htons(uint16_t x)
{
  return endian_be16(x);
}

static inline uint32_t ntohl(uint32_t x)
{
  return endian_be32(x);
}

static inline uint16_t ntohs(uint16_t x)
{
  return endian_be16(x);
}

#endif

