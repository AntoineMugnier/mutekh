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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>

#include <mutek/semaphore.h>

#include <network/packet.h>
#include <network/protos.h>
#include <network/if.h>
#include <mutek/printk.h>

#ifdef CONFIG_NETWORK_PROFILING
uint_fast32_t	netobj_new[NETWORK_PROFILING_NB_OBJS] = { 0, 0, 0 };
uint_fast32_t	netobj_del[NETWORK_PROFILING_NB_OBJS] = { 0, 0, 0 };
#endif

/*
 * The packet object constructor.
 */

struct net_packet_s * packet_obj_new()
{
  struct net_packet_s *obj = mem_alloc(sizeof(*obj), mem_scope_sys);

  memset(obj->header, 0, sizeof (obj->header));
  obj->parent = NULL;
  obj->stage = 0;
  obj->packet = NULL;

#ifdef CONFIG_NETWORK_PROFILING
  netobj_new[NETWORK_PROFILING_PACKET]++;
#endif

  return obj;
}

/*
 * The packet object destructor.
 */

void packet_obj_destroy(struct net_packet_s *obj)
{
  /* if the packet refers to another packet, decrement the ref count */
  if (obj->parent)
    packet_obj_refdec(obj->parent);

  /* if the packet has data, free it */
  if (obj->packet)
    mem_free(obj->packet);

#ifdef CONFIG_NETWORK_PROFILING
  netobj_del[NETWORK_PROFILING_PACKET]++;
#endif

  mem_free(obj);
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
 * clone a packet if refcount > 1
 */

struct net_packet_s		*packet_dup(struct net_packet_s	*orig)
{
  packet_obj_refinc(orig);
  return orig; /* XXX */
}

/*
 * packet queue functions.
 */

GCT_CONTAINER_NOLOCK_FCNS(packet_queue, inline, packet_queue,
                          init, destroy, pop, pushback);
GCT_CONTAINER_FCNS(packet_queue, inline, packet_queue_lock);

#ifdef CONFIG_NETWORK_PROFILING
/*
 * Display network layer profiling info.
 */

void				netprofile_show(void)
{
  uint_fast8_t			i;
  char				*label[NETWORK_PROFILING_NB_OBJS];

  label[NETWORK_PROFILING_IF] = "net_if";
  label[NETWORK_PROFILING_PROTO] = "net_proto";
  label[NETWORK_PROFILING_ROUTE] = "net_route";
  label[NETWORK_PROFILING_PACKET] = "net_packet";
  label[NETWORK_PROFILING_ARP_ENTRY] = "arp_entry";
  label[NETWORK_PROFILING_FRAGMENT] = "ip_fragment";

  for (i = 0; i < NETWORK_PROFILING_NB_OBJS; i++)
    printk("%s: %u new %u del\n", label[i], netobj_new[i], netobj_del[i]);
}
#endif
