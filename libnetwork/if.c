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

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/if.h>

#include <netinet/ip.h>
#include <netinet/arp.h>
#include <netinet/icmp.h>
#include <netinet/udp.h>

#include <hexo/device/net.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <stdio.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Functions for the interface container.
 */

CONTAINER_FUNC(static inline, net_if, HASHLIST, net_if, NOLOCK, list_entry, STRING, name);

/*
 * Some local variables.
 */

static net_if_root_t	ifs = CONTAINER_ROOT_INITIALIZER(net_if, HASHLIST, NOLOCK);
static uint_fast8_t	ifid = 0;
static uint_fast8_t	ethid = 0;

/*
 * Register a new interface.
 */

struct net_if_s	*if_register(struct device_s	*dev,
			     net_if_type_t	type,
			     uint8_t		*mac)
{
  struct net_proto_s				*arp;
  struct net_proto_s				*icmp;
  struct net_proto_s				*udp;
  struct net_if_s				*interface;

  /* create new device node */
  interface = mem_alloc(sizeof (struct net_if_s), MEM_SCOPE_SYS);
  interface->rx_bytes = interface->rx_packets = interface->tx_bytes = interface->tx_packets = 0;

  /* initialize standard protocols for the device */
  interface->ip = net_alloc_proto(&ip_protocol);
  arp = net_alloc_proto(&arp_protocol);
  interface->bootproto.rarp = net_alloc_proto(&rarp_protocol);
  icmp = net_alloc_proto(&icmp_protocol);
  udp = net_alloc_proto(&udp_protocol);
  if_register_proto(interface, interface->ip, arp);
  if_register_proto(interface, arp, interface->ip);
  if_register_proto(interface, interface->bootproto.rarp, interface->ip);
  if_register_proto(interface, icmp, interface->ip);
  if_register_proto(interface, udp, interface->ip);
  interface->boottype = IF_BOOT_RARP;

  /* name the interface */
  interface->dev = dev;
  interface->mac = mac;
  if (type == IF_ETHERNET)
    sprintf(interface->name, "eth%d", ethid++);
  else
    sprintf(interface->name, "if%d", ifid++);

  /* add to the interface list */
  net_if_push(&ifs, interface);

  return interface;
}

/*
 * Unregister a net interface.
 */

void			if_unregister(struct net_if_s	*interface)
{
  /* XXX */
}

/*
 * Bring an interface up.
 */

void			if_up(char*		name, ...)
{
  struct device_s	*dev;
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&ifs, name)))
    {
      dev = interface->dev;

      printf("Bringing up interface %s using %s...\n", name,
	     interface->boottype == IF_BOOT_RARP ? "RARP" : "undefined");

      switch (interface->boottype)
	{
	  case IF_BOOT_RARP:
	    rarp_request(interface, interface->bootproto.rarp, NULL);
	    break;
	  case IF_BOOT_DHCP:
	    break;
	  default:
	    /* XXX otherwise, static IP */
	    break;
	}
    }
}

/*
 * Bring an interface down.
 */

void			if_down(char*		name, ...)
{
  struct device_s	*dev;
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&ifs, name)))
    {
      dev = interface->dev;

      /* XXX */
    }
}

/*
 * Register a new protocol.
 */

void			if_register_proto(struct net_if_s	*interface,
					  struct net_proto_s	*proto,
					  ...)
{
  va_list		va;

  va_start(va, proto);

  /* call the protocol constructor */
  if (proto->desc->initproto)
    proto->desc->initproto(interface, proto, va);

  /* insert in the protocol list */
  net_protos_push(&interface->protocols, proto);

  va_end(va);
}

/*
 * Push a packet.
 */

void			if_pushpkt(struct net_if_s	*interface,
				   struct net_packet_s	*packet)
{
  struct net_proto_s		*p;

  interface->rx_bytes += packet->header[0].size;
  interface->rx_packets++;

  if ((p = net_protos_lookup(&interface->protocols, packet->proto)))
    p->desc->pushpkt(interface, packet, p);
  else
    net_debug("NETWORK: no protocol to handle packet (id = 0x%x)\n",
	      packet->proto);
}

/*
 * Prepare a packet.
 */

uint8_t			*if_preparepkt(struct net_if_s		*interface,
				       struct net_packet_s	*packet,
				       size_t			size,
				       size_t			max_padding)
{
  return dev_net_preparepkt(interface->dev, packet, size, max_padding);
}

/*
 * Send a packet.
 */

void			if_sendpkt(struct net_if_s	*interface,
				   struct net_packet_s	*packet,
				   struct net_proto_s	*proto)
{
  interface->tx_bytes += packet->header[0].size;
  interface->tx_packets++;

  return dev_net_sendpkt(interface->dev, packet, proto->id);
}

/*
 * Dump statistics.
 */

void			if_stats(const char	*name)
{
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&ifs, name)))
    {
      printf("%s statistics:\n", name);

      printf("%Lu bytes sent (%u packets), %Lu bytes received (%u packets)\n",
	     interface->tx_bytes, interface->tx_packets,
	     interface->rx_bytes, interface->rx_packets);
    }
}
