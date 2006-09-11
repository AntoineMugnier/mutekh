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

#include <hexo/endian.h>

#include <netinet/packet.h>

/*
 * The packet object constructor.
 */

OBJECT_CONSTRUCTOR(packet_obj)
{
  struct net_packet_s		*packet;

  packet = mem_alloc(sizeof (struct net_packet_s), MEM_SCOPE_THREAD);
  memset(packet, 0, sizeof (struct net_packet_s));

  packet_obj_init(packet);

  return packet;
}

/*
 * The packet object destructor.
 */

OBJECT_DESTRUCTOR(packet_obj)
{
  if (obj->packet)
    mem_free(obj->packet);

  mem_free(obj);
}

/*
 * Compute the checksum of a packet chunk.
 */

uint_fast16_t		packet_checksum(uint8_t		*data,
					size_t		size)
{
  uint_fast32_t		checksum = 0;
  uint16_t		*d = (uint16_t*)data;

  while(size > 1)
    {
      checksum = checksum + endian_16_na_load(d);
      d++;
      size = size - 2;
    }

  if (size)
    checksum = checksum + *(uint8_t*)d;

  while (checksum >> 16)
    checksum = (checksum & 0xffff) + (checksum >> 16);

  return ~checksum;
}
