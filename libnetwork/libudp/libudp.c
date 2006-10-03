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

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/udp.h>
#include <netinet/ip.h>
#include <netinet/ether.h>
#include <netinet/in.h>

#include <netinet/libudp.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>
#include <gpct/cont_dlist.h>

/*
 * Functions for the interface container.
 */

CONTAINER_FUNC(static inline, net_if, HASHLIST, net_if, NOLOCK, list_entry, STRING, name);

/*
 * Callback container.
 */

static udp_callback_root_t	udp_callbacks = CONTAINER_ROOT_INITIALIZER(udp_callback, HASHLIST, NOLOCK);

CONTAINER_FUNC(static inline, udp_callback, HASHLIST, udp_callback, NOLOCK, list_entry, BLOB, address);

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
  uint8_t		*dest;

  /* look for the good IP module */
  CONTAINER_FOREACH(net_if, HASHLIST, net_if, &net_interfaces,
  {
    interface = item;
    /* XXX foreach + lookup will be better */
    CONTAINER_FOREACH(net_protos, HASHLIST, net_protos, &interface->protocols,
    {
      switch (local->address.family)
	{
	  case addr_ipv4:
	    {
	      if (item->id != ETHERTYPE_IP)
		break;
	      struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)item->pv;

	      if (pv->addr == local->address.addr.ipv4)
		{
		  addressing = item;
		  break;
		}
	    }
	    break;
	  default:
	    return -1;
	}
      if (addressing)
	break;
    });
    if (addressing)
      break;
  });

  if (interface == NULL || addressing == NULL)
    return -1;

  packet = packet_obj_new(NULL);

  /* prepare the packet */
  dest = udp_preparepkt(interface, addressing, packet, size, 0);

  /* copy data into the packet */
  memcpy(dest, data, size);

  /* setup source and destination address */
  memcpy(&packet->tADDR, &remote->address, sizeof (struct net_addr_s));

  /* send UDP packet */
  udp_sendpkt(interface, addressing, packet, local->port, remote->port);

  return 0;
}

/*
 * Register a callback for receiving packets
 */

int_fast8_t			udp_callback(struct net_udp_addr_s	*local,
					     udp_callback_t		*callback)
{
  struct udp_callback_desc_s	*desc;

  /* allocate an build the descriptor */
  desc = mem_alloc(sizeof (struct udp_callback_desc_s), MEM_SCOPE_SYS);

  memcpy(&desc->address[0], local, sizeof (struct net_udp_addr_s));
  desc->callback = callback;

  /* register the callback */
  udp_callback_push(&udp_callbacks, desc);

  return 0;
}

/*
 * Signal packet reception
 */

void				udp_signal(struct net_packet_s	*packet,
					   struct udphdr	*hdr)
{
  struct udp_callback_desc_s	*desc;
  struct net_udp_addr_s		*local;
  struct net_udp_addr_s		*remote;
  uint8_t			*buff;
  uint_fast16_t			size;

  /* build local address descriptor */
  local = mem_alloc(sizeof (struct net_udp_addr_s), MEM_SCOPE_SYS);

  memcpy(&local->address, &packet->tADDR, sizeof (struct net_addr_s));
  local->port = hdr->dest;

  /* do we have a callback to handle the packet */
  if (!(desc = udp_callback_lookup(&udp_callbacks, (void*)local)))
    {
      /* this packet is destinated to no one */
      mem_free(local);
      return;
    }

  /* build remote address descriptor */
  remote = mem_alloc(sizeof (struct net_udp_addr_s), MEM_SCOPE_SYS);

  memcpy(&remote->address, &packet->sADDR, sizeof (struct net_addr_s));
  remote->port = hdr->source;

  /* copy the packet */
  size = net_be16_load(hdr->len) - sizeof (struct udphdr);
  buff = mem_alloc(size, MEM_SCOPE_SYS);
  memcpy(buff, packet->header[packet->stage].data, size);

  /* callback */
  desc->callback(local, remote, buff, size);
}

