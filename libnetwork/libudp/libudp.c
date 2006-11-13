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
#include <netinet/route.h>

#include <netinet/libudp.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>
#include <gpct/cont_dlist.h>

/*
 * The descriptors set.
 */

static udp_desc_root_t	descriptors = CONTAINER_ROOT_INITIALIZER(udp_desc, HASHLIST, NOLOCK);

CONTAINER_FUNC(static inline, udp_desc, HASHLIST, udp_desc, NOLOCK, address);
CONTAINER_KEY_FUNC(static inline, udp_desc, HASHLIST, udp_desc, NOLOCK, address);

/*
 * Descriptors contructor and destructor.
 */

OBJECT_CONSTRUCTOR(udp_desc_obj)
{
  struct net_udp_desc_s	*obj;

  if ((obj = mem_alloc(sizeof (struct net_udp_desc_s), MEM_SCOPE_NETWORK)) == NULL)
    return NULL;

  udp_desc_obj_init(obj);

  return obj;
}

OBJECT_DESTRUCTOR(udp_desc_obj)
{
  mem_free(obj);
}

/*
 * Create or connect a connected UDP descriptor.
 */

error_t			udp_connect(struct net_udp_desc_s	**desc,
				    struct net_udp_addr_s	*remote)
{
  struct net_route_s	*route;

  /* look for a route */
  if ((route = route_get(&remote->address)) == NULL)
    return -EHOSTUNREACH;

  /* allocate a descriptor if needed */
  if (*desc == NULL)
    {
      if ((*desc = udp_desc_obj_new(NULL)) == NULL)
	return -ENOMEM;
      (*desc)->bound = 0;
      (*desc)->checksum = 1;
      (*desc)->callback_error = NULL;
    }

  /* setup the connection endpoint */
  memcpy(&(*desc)->remote, remote, sizeof (struct net_udp_addr_s));
  (*desc)->route = route;
  (*desc)->connected = 1;

  return 0;
}

/*
 * Create or bind a listening UDP descriptor.
 */

error_t			udp_bind(struct net_udp_desc_s	**desc,
				 struct net_udp_addr_s	*local,
				 udp_callback_t		*callback,
				 void			*pv)
{
  struct net_udp_desc_s	*d;

  /* alloc a descriptor if needed */
  if (*desc == NULL)
    {
      if ((d = udp_desc_obj_new(NULL)) == NULL)
	return -ENOMEM;
      d->connected = 0;
      d->checksum = 1;
      d->callback_error = NULL;
    }
  else
    {
      d = *desc;

      /* if already bound */
      if (d->bound)
	{
	  /* XXX what to do ? */
	}
    }

  /* bind the socket */
  d->bound = 1;
  d->callback = callback;
  d->pv = pv;
  memcpy(&d->address, local, sizeof (struct net_udp_addr_s));
  if (!udp_desc_push(&descriptors, d))
    {
      udp_desc_obj_refdrop(d);
      return -ENOMEM;
    }

  if (desc != NULL)
    *desc = d;

  return 0;
}

/*
 * Send an UDP packet.
 *
 * If desc is NULL, a temporary descriptor is created, used, and removed.
 * If desc is not NULL, remote can be NULL if the socket is connected.
 */

error_t			udp_send(struct net_udp_desc_s		*desc,
				 struct net_udp_addr_s		*remote,
				 const void			*data,
				 size_t				size)
{
  struct net_route_s	*route;
  struct net_packet_s	*packet;
  uint_fast16_t		local_port;
  uint8_t		*dest;

  /* find a route to the remote host */
  if (desc == NULL)
    {
      if (remote == NULL)
	return -EDESTADDRREQ;

      if ((route = route_get(&remote->address)) == NULL)
	return -EHOSTUNREACH;
    }
  else
    {
      if (remote == NULL)
	{
	  if (!desc->connected)
	    return -EDESTADDRREQ;

	  remote = &desc->remote;
	  route = desc->route;
	}
      else
	{
	  if ((route = route_get(&remote->address)) == NULL)
	    return -EHOSTUNREACH;
	}
    }

  /* select a port */
  if (desc != NULL && desc->bound)
    local_port = desc->address.port;
  else
    {
      local_port = UDP_TEMP_PORT_BASE + (rand() % UDP_TEMP_PORT_RANGE);
    }

  /* prepare the packet */
  if ((packet = packet_obj_new(NULL)) == NULL)
    return -ENOMEM;
  if ((dest = udp_preparepkt(route->interface, route->addressing, packet, size, 0)) == NULL)
    {
      packet_obj_refdrop(packet);

      return -ENOMEM;
    }

  /* copy data into the packet */
  memcpy(dest, data, size);

  /* setup destination address */
  memcpy(&packet->tADDR, &remote->address, sizeof (struct net_addr_s));

  /* send UDP packet */
  udp_sendpkt(route->interface, route->addressing, packet, local_port, remote->port, desc->checksum);

  return 0;
}

/*
 * Close an UDP descriptor.
 */

void			udp_close(struct net_udp_desc_s		*desc)
{
  if (desc->bound)
    udp_desc_remove(&descriptors, desc);
}

/*
 * Signal an incoming packet. This function is called by the UDP
 * protocol module.
 */

void			libudp_signal(struct net_packet_s	*packet,
				      struct udphdr		*hdr)
{
  struct net_udp_desc_s	*desc;
  struct net_udp_addr_s	local;
  struct net_udp_addr_s	remote;
  uint8_t		*buff;
  uint_fast16_t		size;

  /* build local address descriptor */
  memcpy(&local.address, &packet->tADDR, sizeof (struct net_addr_s));
  local.port = hdr->dest;

  /* do we have a callback to handle the packet */
  if ((desc = udp_desc_lookup(&descriptors, (void *)&local)) == NULL)
    {
      packet->stage -= 2;

      /* this packet is destinated to no one */
      packet->source_addressing->desc->f.addressing->errormsg(packet, ERROR_PORT_UNREACHABLE);
      return;
    }

  memcpy(&remote.address, &packet->sADDR, sizeof (struct net_addr_s));
  remote.port = hdr->source;

  /* copy the packet */
  size = net_be16_load(hdr->len) - sizeof (struct udphdr);
  if ((buff = mem_alloc(size, MEM_SCOPE_NETWORK)) == NULL)
    return;
  memcpy(buff, packet->header[packet->stage].data, size);

  /* callback */
  desc->callback(desc, &remote, buff, size, desc->pv);
}

/*
 * This function is used to clean the LibUDP.
 */

void		libudp_destroy(void)
{
  udp_desc_clear(&descriptors);
}

/*
 * Signal an error on an UDP packet. This function is called by a
 * control protocol such as ICMP.
 */

NET_SIGNAL_ERROR(libudp_signal_error)
{
  struct net_udp_desc_s	*desc;
  struct net_udp_addr_s	local;

  memcpy(&local.address, address, sizeof (struct net_addr_s));
  local.port = port;

  /* do we have a callback to handle the error */
  if ((desc = udp_desc_lookup(&descriptors, (void *)&local)) != NULL)
    {
      if (desc->callback_error != NULL)
	{
	  desc->callback_error(desc, error, desc->pv_error);
	}
    }
}

