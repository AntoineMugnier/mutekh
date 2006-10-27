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
#include <hexo/types.h>

#include <pthread.h>
#include <semaphore.h>

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/if.h>

#ifdef CONFIG_NETWORK_PROFILING
uint_fast32_t	netobj_new[NETWORK_PROFILING_NB_OBJS] = { 0, 0, 0 };
uint_fast32_t	netobj_del[NETWORK_PROFILING_NB_OBJS] = { 0, 0, 0 };
#endif

/*
 * The packet object constructor.
 */

OBJECT_CONSTRUCTOR(packet_obj)
{
  struct net_packet_s		*packet;

  if ((packet = mem_alloc(sizeof (struct net_packet_s), MEM_SCOPE_NETWORK)) == NULL)
    return NULL;
  memset(packet->header, 0, sizeof (packet->header));
  packet->parent = NULL;
  packet->stage = 0;
  packet->packet = NULL;

  packet_obj_init(packet);

#ifdef CONFIG_NETWORK_PROFILING
  netobj_new[NETWORK_PROFILING_PACKET]++;
#endif

  return packet;
}

/*
 * The packet object destructor.
 */

OBJECT_DESTRUCTOR(packet_obj)
{
  /* if the packet refers to another packet, decrement the ref count */
  if (obj->parent)
    packet_obj_refdrop(obj->parent);

  /* if the packet has data, free it */
  if (obj->packet)
    mem_free(obj->packet);

  mem_free(obj);

#ifdef CONFIG_NETWORK_PROFILING
  netobj_del[NETWORK_PROFILING_PACKET]++;
#endif
}

/*
 * Compute the checksum of a packet chunk.
 */

#ifndef HAS_CPU_PACKET_CHECKSUM
#undef packet_checksum
uint16_t		packet_checksum(const void	*data,
					size_t		size)
{
  uint_fast32_t		checksum = 0;
  uint16_t		*d = (uint16_t *)data;

  /* compute the 32 bits sum of 16 words */
  while(size > 1)
    {
      checksum = checksum + net_16_load(*d);
      d++;
      size = size - 2;
    }

  /* check the parity of size, add the last byte if necessary */
  if (size)
    checksum = checksum + *(uint8_t *)d;

  /* take account of carries of 16 bits words */
  while (checksum >> 16)
    checksum = (checksum & 0xffff) + (checksum >> 16);

  return checksum;
}
#endif

/*
 * Compute the checksum of a packet chunk while copying it.
 */

#ifndef HAS_CPU_PACKET_MEMCPY
#undef packet_memcpy
uint16_t		packet_memcpy(void		*dst,
				      const void	*src,
				      size_t		size)
{
  uint_fast32_t		checksum = 0;
  uint16_t		*d = (uint16_t *)src;
  uint16_t		*p = (uint16_t *)dst;

  /* compute 32 bits sum and copy */
  while(size > 1)
    {
      checksum = checksum + net_16_load(*d);
      net_16_store(*p, *d);
      d++;
      size = size - 2;
    }

  /* count and copy the last remaining byte if needed */
  if (size)
    {
      checksum = checksum + *(uint8_t *)d;
      *(uint8_t *)p = *(uint8_t *)d;
    }

  /* take account of carries of 16 bits words */
  while (checksum >> 16)
    checksum = (checksum & 0xffff) + (checksum >> 16);

  return checksum;
}
#endif


/*
 * packet queue functions.
 */

CONTAINER_FUNC(inline, packet_queue, DLIST, packet_queue, NOLOCK);
CONTAINER_FUNC(inline, packet_queue, DLIST, packet_queue_lock, HEXO_SPIN_IRQ);

/*
 * packet dispatching thread.
 */

void				*packet_dispatch(void	*data)
{
  struct net_dispatch_s		*info = (struct net_dispatch_s *)data;
  packet_queue_root_t		*root = info->packets;
  struct net_if_s		*interface = info->interface;
  sem_t				*sem = info->sem;
  bool_t			*run = info->running;
  struct net_packet_s		*packet;

  mem_free(data);

  while (*run)
    {
      /* wait for a packet */
      sem_wait(sem);

      /* retreive the incoming packet */
      packet = packet_queue_lock_pop(root);
      if (packet != NULL)
	{
	  /* dispatch to the interface */
	  if_pushpkt(interface, packet);

	  /* drop the packet */
	  packet_obj_refdrop(packet);
	}
    }

  return NULL;
}

#ifdef CONFIG_NETWORK_PROFILING
/*
 * Display network layer profiling info.
 */

void				netprofile_show(void)
{
  uint_fast8_t			i;

  for (i = 0; i < NETWORK_PROFILING_NB_OBJS; i++)
    printf("%s%u / %u", i != 0 ? " - " : "", netobj_new[i], netobj_del[i]);
  printf("\n");
}
#endif
