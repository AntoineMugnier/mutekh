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

#include <pthread.h>

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/ether.h>

/*
 * The packet object constructor.
 */

OBJECT_CONSTRUCTOR(packet_obj)
{
  struct net_packet_s		*packet;

  packet = mem_alloc(sizeof (struct net_packet_s), MEM_SCOPE_CONTEXT);
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
  uint16_t		*d = (uint16_t *)data;

  while(size > 1)
    {
      checksum = checksum + net_16_load(*d);
      d++;
      size = size - 2;
    }

  if (size)
    checksum = checksum + *(uint8_t *)d;

  while (checksum >> 16)
    checksum = (checksum & 0xffff) + (checksum >> 16);

  return (~checksum) & 0xffff;
}

/*
 * packet queue functions.
 */

CONTAINER_FUNC(, packet_queue, DLIST, packet_queue, NOLOCK, queue_entry);
CONTAINER_FUNC(, packet_queue_lock, DLIST, packet_queue_lock, HEXO_SPIN_IRQ, queue_entry_spin);

/*
 * packet dispatching thread.
 */

void				*packet_dispatch(void	*data)
{
  struct net_dispatch_s		*info = (struct net_dispatch_s *)data;
  net_protos_root_t		*protocols = info->protocols;
  packet_queue_lock_root_t	*root = info->packets;
  struct device_s		*dev = info->device;
  struct net_packet_s		*packet;
  struct net_proto_s		*p;

  mem_free(data);

  while (/* XXX */ 1)
    {
      packet = packet_queue_lock_pop(root);
      if (packet)
	{
	  /* dispatch to the matching protocol */
	  if ((p = net_protos_lookup(protocols, packet->proto)))
	    p->desc->pushpkt(dev, packet, p, protocols);
	  else
	    net_debug("NETWORK: no protocol to handle packet (id = 0x%x)\n",
		      packet->proto);

	  packet_obj_refdrop(packet);
	}
      pthread_yield();
    }

  return NULL;
}
