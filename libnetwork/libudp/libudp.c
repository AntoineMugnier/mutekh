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

/*
 * User interface to UDP transport layer.
 */

#include <hexo/types.h>
#include <hexo/alloc.h>
#include <hexo/cpu.h>

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/udp.h>
#include <netinet/ip.h>
#include <netinet/ether.h>
#include <netinet/in.h>
#include <netinet/if.h>

#include <netinet/libudp.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>
#include <gpct/cont_dlist.h>

/*
 * Callback container.
 */

static udp_callback_root_t	udp_callbacks = CONTAINER_ROOT_INITIALIZER(udp_callback, HASHLIST, NOLOCK);

CONTAINER_FUNC(static inline, udp_callback, HASHLIST, udp_callback, NOLOCK, address);
CONTAINER_KEY_FUNC(static inline, udp_callback, HASHLIST, udp_callback, NOLOCK, address);

/*
 * Send a datagram via UDP
 */

int_fast8_t		udp_send(struct net_udp_addr_s	*local,
				 struct net_udp_addr_s	*remote,
				 void			*data,
				 size_t			size)
{
  struct net_if_s	*interface = NULL;
  struct net_proto_s	*addressing = NULL;
  struct net_packet_s	*packet;
  net_proto_id_t	id;
  uint8_t		*dest;

  /* check size */
  if (size > 65507)
    return -EINVAL;

  /* look for the good IP module */
  id = local->address.family;
  CONTAINER_FOREACH(net_if, HASHLIST, NOLOCK, &net_interfaces,
  {
    interface = item;
    for (addressing = net_protos_lookup(&interface->protocols, id);
	 addressing != NULL;
	 addressing = net_protos_lookup_next(&interface->protocols, addressing, id))
      if (addressing->desc->f.addressing->matchaddr(addressing, &local->address, NULL, NULL))
	goto ok;

  });

 ok:

  if (interface == NULL || addressing == NULL)
    return -ENOENT;

  /* port selection */
  if (local->port == 0)
    {
      local->port = addressing->desc->f.addressing->reserve_port(addressing,
								 IPPROTO_UDP);
      if (local->port == 0)
	return -EAGAIN;
    }

  /* prepare the packet */
  packet = packet_obj_new(NULL);
  dest = udp_preparepkt(interface, addressing, packet, size, 0);

  /* copy data into the packet */
  memcpy(dest, data, size);

  /* setup destination address */
  memcpy(&packet->tADDR, &remote->address, sizeof (struct net_addr_s));

  /* send UDP packet */
  udp_sendpkt(interface, addressing, packet, local->port, remote->port);

  /* release used port */
  addressing->desc->f.addressing->release_port(addressing, IPPROTO_UDP, local->port);

  return 0;
}

/*
 * Register a callback for receiving packets
 */

int_fast8_t			udp_callback(struct net_udp_addr_s	*local,
					     udp_callback_t		*callback,
					     void			*pv)
{
  struct udp_callback_desc_s	*desc;
  struct net_if_s		*interface = NULL;
  struct net_proto_s		*addressing = NULL;
  net_proto_id_t		id;

  /* look for the good IP module */
  id = local->address.family;
  CONTAINER_FOREACH(net_if, HASHLIST, NOLOCK, &net_interfaces,
  {
    interface = item;
    for (addressing = net_protos_lookup(&interface->protocols, id);
	 addressing != NULL;
	 addressing = net_protos_lookup_next(&interface->protocols, addressing, id))
      if (addressing->desc->f.addressing->matchaddr(addressing, &local->address, NULL, NULL))
	goto ok;

  });

 ok:

  if (interface == NULL || addressing == NULL)
    return -EADDRNOTAVAIL;

  if (local->port == 0)
    {
      if ((local->port = addressing->desc->f.addressing->reserve_port(addressing, IPPROTO_UDP)) == 0)
	return -EAGAIN;
    }
  else
    {
      /* mask the port as reserved */
      if (addressing->desc->f.addressing->mark_port(addressing, IPPROTO_UDP, local->port))
	return -EADDRINUSE;
    }

  /* allocate and build the descriptor */
  desc = mem_alloc(sizeof (struct udp_callback_desc_s), MEM_SCOPE_SYS);

  memcpy(&desc->address, local, sizeof (struct net_udp_addr_s));
  desc->addressing = addressing;
  desc->callback = callback;
  desc->pv = pv;

  /* register the callback */
  udp_callback_push(&udp_callbacks, desc);

  return 0;
}

/*
 * Signal packet reception
 */

void				libudp_signal(struct net_packet_s	*packet,
					      struct udphdr		*hdr)
{
  struct udp_callback_desc_s	*desc;
  struct net_udp_addr_s		local;
  struct net_udp_addr_s		remote;
  struct net_udp_addr_s		*p_local;
  uint8_t			*buff;
  uint_fast16_t			size;

  /* build local address descriptor */
  p_local = &local;

  memcpy(&local.address, &packet->tADDR, sizeof (struct net_addr_s));
  local.port = hdr->dest;

  /* do we have a callback to handle the packet */
  if (!(desc = udp_callback_lookup(&udp_callbacks, (void *)p_local)))
    {
      packet->stage -= 2;

      /* this packet is destinated to no one */
      packet->source_addressing->desc->f.addressing->errormsg(packet,
							      ERROR_PORT_UNREACHABLE);
      return;
    }

  memcpy(&remote.address, &packet->sADDR, sizeof (struct net_addr_s));
  remote.port = hdr->source;

  /* copy the packet */
  size = net_be16_load(hdr->len) - sizeof (struct udphdr);
  buff = mem_alloc(size, MEM_SCOPE_SYS);
  memcpy(buff, packet->header[packet->stage].data, size);

  /* callback */
  desc->callback(p_local, &remote, buff, size, desc->pv);
}

/*
 * Close a listening UDP connection.
 */

void		udp_close(struct net_udp_addr_s	*local)
{
  struct udp_callback_desc_s	*desc;

  if ((desc = udp_callback_lookup(&udp_callbacks, (void *)local)) != NULL)
    {
      desc->addressing->desc->f.addressing->release_port(desc->addressing,
							 IPPROTO_UDP, local->port);
      udp_callback_remove(&udp_callbacks, desc);

      mem_free(desc);
    }
}

/*
 * Cleanup LibUDP.
 */

void		libudp_destroy(void)
{
  struct udp_callback_desc_s	*desc;

  while ((desc = udp_callback_pop(&udp_callbacks)) != NULL)
    {
      desc->addressing->desc->f.addressing->release_port(desc->addressing,
							 IPPROTO_UDP, desc->address.port);
      mem_free(desc);
    }
}
