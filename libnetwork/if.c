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
#include <netinet/tcp.h>

#include <hexo/device/net.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <stdio.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>
#include <gpct/cont_dlist.h>

/*
 * Functions for the interface container.
 */

CONTAINER_FUNC(static inline, net_if, HASHLIST, net_if, NOLOCK, name);
CONTAINER_KEY_FUNC(static inline, net_if, HASHLIST, net_if, NOLOCK, name);

/*
 * Some local variables.
 */

net_if_root_t		net_interfaces = CONTAINER_ROOT_INITIALIZER(net_if, HASHLIST, NOLOCK);
static uint_fast8_t	ifid = 0;
static uint_fast8_t	ethid = 0;

/*
 * Register a new interface.
 */

struct net_if_s	*if_register(struct device_s	*dev,
			     net_if_type_t	type,
			     uint8_t		*mac,
			     uint_fast16_t	mtu)
{
  struct net_proto_s				*ip;
  struct net_proto_s				*arp;
  struct net_proto_s				*icmp;
  struct net_proto_s				*udp;
  struct net_proto_s				*tcp;
  struct net_if_s				*interface;

  /* create new device node */
  interface = mem_alloc(sizeof (struct net_if_s), MEM_SCOPE_SYS);
  interface->rx_bytes = interface->rx_packets = interface->tx_bytes = interface->tx_packets = 0;
  net_protos_init(&interface->protocols);

  /* XXX initialize standard protocols for the device */
#ifdef CONFIG_NETWORK_IPV4
  ip = net_alloc_proto(&ip_protocol);
  arp = net_alloc_proto(&arp_protocol);
  icmp = net_alloc_proto(&icmp_protocol);
  if_register_proto(interface, arp);
  if_register_proto(interface, icmp);
#endif

#ifdef CONFIG_NETWORK_RARP
  interface->bootproto.rarp = net_alloc_proto(&rarp_protocol);
  if_register_proto(interface, interface->bootproto.rarp, ip);
#endif

#ifdef CONFIG_NETWORK_UDP
  udp = net_alloc_proto(&udp_protocol);
  if_register_proto(interface, udp);
#endif
#ifdef CONFIG_NETWORK_TCP
  tcp = net_alloc_proto(&tcp_protocol);
  if_register_proto(interface, tcp);
#endif

  /* XXX a funny hack for testing, to be removed */
  static uint_fast8_t chiche = 0;
  if (!chiche)
    {
      interface->boottype = IF_BOOT_NONE;
#ifdef CONFIG_NETWORK_IPV4
      if_register_proto(interface, ip, arp, icmp, 0x0a0202f0, 0xffffff00);
#endif
#if 0
      ip = net_alloc_proto(&ip_protocol);
      if_register_proto(interface, ip, arp, icmp, 0x0a0202f1, 0xffffff00);
#endif
    }
  else
    {
      interface->boottype = IF_BOOT_NONE;
#ifdef CONFIG_NETWORK_IPV4
      if_register_proto(interface, ip, arp, icmp, 0x0a020302, 0xffffff00);
#endif
      mtu = 520;
    }
  chiche = 1;

  /* copy properties and name the interface */
  interface->dev = dev;
  interface->mac = mac;
  interface->mtu = mtu;
  if (type == IF_ETHERNET)
    sprintf(interface->name, "eth%d", ethid++);
  else
    sprintf(interface->name, "if%d", ifid++);

  /* add to the interface list */
  net_if_push(&net_interfaces, interface);

  printf("Registered new interface %s (MTU = %u)\n", interface->name, interface->mtu);

  return interface;
}

/*
 * Unregister a net interface.
 */

void			if_unregister(struct net_if_s	*interface)
{
  /* XXX if_unregister */

}

/*
 * Bring an interface up.
 */

void			if_up(char*		name, ...)
{
  struct device_s	*dev;
  struct net_if_s	*interface;
  va_list		va;

  va_start(va, name);

  if ((interface = net_if_lookup(&net_interfaces, name)))
    {
      dev = interface->dev;

      printf("Bringing up interface %s using %s...\n", name,
	     interface->boottype == IF_BOOT_RARP ? "Reverse ARP" : "Static address");

      switch (interface->boottype)
	{
	  case IF_BOOT_RARP:
#ifdef CONFIG_NETWORK_RARP
	    rarp_request(interface, interface->bootproto.rarp, NULL);
#endif
	    break;
	  case IF_BOOT_DHCP:
	    break;
	  default:
	    break;
	}
    }

  va_end(va);
}

/*
 * Bring an interface down.
 */

void			if_down(char*		name, ...)
{
  struct device_s	*dev;
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&net_interfaces, name)))
    {
      dev = interface->dev;

      /* XXX if_down */
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
  struct net_proto_s	*item;

  interface->rx_bytes += packet->header[0].size;
  interface->rx_packets++;

  /* lookup to all modules matching the protocol  */
  for (item = net_protos_lookup(&interface->protocols, packet->proto);
       item != NULL;
       item = net_protos_lookup_next(&interface->protocols, item, packet->proto))
    item->desc->pushpkt(interface, packet, item);
}

/*
 * Prepare a packet.
 */

inline uint8_t		*if_preparepkt(struct net_if_s		*interface,
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
				   net_proto_id_t	proto)
{
  interface->tx_bytes += packet->header[0].size;
  interface->tx_packets++;

  if (!memcmp(interface->mac, packet->tMAC, packet->MAClen))
    {
      /* XXX c'est mal poli on passe devant tout le monde */
      /* XXX maj header */
      packet->proto = proto;
      packet->stage++;
      if_pushpkt(interface, packet);
      packet_obj_refdrop(packet);
      packet_obj_refdrop(packet);
    }
  else
    dev_net_sendpkt(interface->dev, packet, proto);
}

/*
 * Dump statistics.
 */

void			if_stats(const char	*name)
{
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&net_interfaces, name)))
    {
      printf("%s statistics:\n", name);

      printf("%Lu bytes sent (%u packets), %Lu bytes received (%u packets)\n",
	     interface->tx_bytes, interface->tx_packets,
	     interface->rx_bytes, interface->rx_packets);
    }
}

/*
 * Get interface from name.
 */

struct net_if_s	*if_get(const char	*name)
{
  return net_if_lookup(&net_interfaces, name);
}
