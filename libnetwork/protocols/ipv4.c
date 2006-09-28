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
 * IP protocol version 4
 *
 */

#include <netinet/ip.h>
#include <netinet/arp.h>
#include <netinet/packet.h>
#include <netinet/ether.h>
#include <netinet/protos.h>

#include <netinet/if.h>
#include <netinet/route.h>

#include <stdio.h>
#include <stdlib.h>

/*
 * Fragment lists.
 */

CONTAINER_FUNC(static inline, ip_packet, HASHLIST, ip_packet, NOLOCK, list_entry, BLOB, id);

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	ip_protocol =
  {
    .name = "IP",
    .id = ETHERTYPE_IP,
    .pushpkt = ip_pushpkt,
    .preparepkt = ip_preparepkt,
    .initproto = ip_init,
    .pv_size = sizeof (struct net_pv_ip_s),
  };

/*
 * Initialize private data of IP module.
 */

NET_INITPROTO(ip_init)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)proto->pv;
  struct net_proto_s	*arp = va_arg(va, struct net_proto_s *);
  uint_fast32_t		ip = va_arg(va, uint_fast32_t);
  uint_fast32_t		mask = va_arg(va, uint_fast32_t);

  pv->interface = interface;
  pv->arp = arp;
  pv->addr = ip;
  pv->mask =mask;
  ip_packet_init(&pv->fragments);
  pv->id_seq = 1;
}

/*
 * Does an address need to be routed ?
 */

static inline	uint_fast8_t	ip_delivery(struct net_if_s	*interface,
					    struct net_proto_s	*ip,
					    uint32_t		addr)
{
  struct net_pv_ip_s		*pv = (struct net_pv_ip_s *)ip->pv;

  if ((addr & pv->mask) == (pv->addr & pv->mask))
    return IP_DELIVERY_DIRECT;
  else
    return IP_DELIVERY_INDIRECT;
}

/*
 * Receive fragments and try to reassemble a packet.
 */

static uint_fast8_t	ip_fragment_pushpkt(struct net_proto_s	*ip,
					    struct net_packet_s	*packet,
					    struct iphdr	*hdr)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)ip->pv;
  struct ip_packet_s	*p;
  struct net_header_s	*nethdr;
  uint8_t		id[6];
  uint_fast16_t		offs;
  uint_fast16_t		fragment;
  uint_fast16_t		datasz;
  uint_fast16_t		total;

  /* the unique identifier of the packet is the concatenation of the
     source address and the packet id */
  memcpy(id, &packet->sADDR.addr.ipv4, 4);
  memcpy(&id[3], &hdr->id, 2);

  /* extract some useful fields */
  fragment = net_be16_load(hdr->fragment);
  offs = (fragment & IP_FRAG_MASK) * 8;
  datasz = net_be16_load(hdr->tot_len) - hdr->ihl * 4;

  net_debug("fragment id %P offs %d size %d\n", id, 6, offs, datasz);

  /* do we already received packet with same id ? */
  if (!(p = ip_packet_lookup(&pv->fragments, id)))
    {
      p = mem_alloc(sizeof (struct ip_packet_s), MEM_SCOPE_CONTEXT);
      memcpy(p->id, id, 6);
      p->size = 0;
      p->received = 0;
      packet_queue_init(&p->packets);
      ip_packet_push(&pv->fragments, p);
    }
  p->received += datasz;
  /* try to determine the total size */
  if (!(fragment & IP_FLAG_MF))
    {
      p->size = offs + datasz;
      net_debug("packet total size %d\n", p->size);
    }

  total = p->size;

  if (total)
    net_debug("received %d out of %d\n", p->received, total);

  if (total && total == p->received)
    {
      struct net_packet_s	*frag;
      uint_fast16_t		headers_len;
      uint8_t			*data;

      net_debug("packet complete\n");

      /* we received the whole packet, reassemble now */
      nethdr = &packet->header[packet->stage];
      headers_len = (packet->header[0].size - nethdr->size);
      /* allocate a packet large enough */
      data = mem_alloc(total + headers_len, MEM_SCOPE_CONTEXT);

      /* copy previous headers (ethernet, ip, etc.) */
      memcpy(data, packet->packet, headers_len);

      net_debug("copying headers : %d-%d\n", 0, headers_len);

      /* copy current packet to its position */
      memcpy(data + headers_len + offs, nethdr->data, datasz);

      net_debug("copying packet : %d-%d\n", offs, offs + datasz);

      /* release current packet data */
      mem_free(packet->packet);

      /* replace by reassembling packet */
      packet->packet = data;
      data += headers_len;

      nethdr->data = data;
      nethdr->size = total;

      /* loop through previously received packets and reassemble them */
      while ((frag = packet_queue_pop(&p->packets)))
	{
	  /* copy data in place */
	  nethdr = &frag->header[packet->stage];
	  offs = (net_be16_load(((struct iphdr *)nethdr[-1].data)->fragment) & IP_FRAG_MASK) * 8;
	  memcpy(data + offs, nethdr->data, nethdr->size);

	  net_debug("copying packet : %d-%d\n", offs, offs + nethdr->size);

	  /* release our reference to the packet */
	  packet_obj_refdrop(frag);
	}

      /* release memory */
      ip_packet_remove(&pv->fragments, p);
      mem_free(p);

      return 1;
    }

  /* otherwise, this is just a fragment */
  packet_obj_refnew(packet);
  packet_queue_push(&p->packets, packet);
  return 0;
}

/*
 * Receive incoming IP packets.
 */

NET_PUSHPKT(ip_pushpkt)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)protocol->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct iphdr		aligned;
#endif
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  net_proto_id_t	proto;
  struct net_proto_s	*p;
  uint_fast16_t		check;
  uint_fast16_t		computed_check;
  uint_fast16_t		fragment;
  uint_fast16_t		tot_len;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* align the packet on 32 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint32_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct iphdr));
      hdr = &aligned;
    }
#endif

  /* check IP version */
  if (hdr->version != 4)
    return;

  /* update packet info */
  IPV4_ADDR_SET(packet->sADDR, net_be32_load(hdr->saddr));
  IPV4_ADDR_SET(packet->tADDR, net_be32_load(hdr->daddr));

  /* is the packet really for me ? */
  if (packet->tADDR.addr.ipv4 != pv->addr)
    {
      struct net_route_s	*route_entry = NULL;

      /* is there a route for this address ? */
      if ((route_entry = route_get(interface, &packet->tADDR)) != NULL)
	{
	  net_debug("routing to host %P\n", &packet->tADDR.addr.ipv4, 4);
	  /* route the packet */
	  ip_route(interface, packet, route_entry);
	}
      else
	{
	  net_debug("no route to host %P\n", &packet->tADDR.addr.ipv4, 4);
	  /* network unreachable */
	  //icmp_error(interface, ICMP_NETWORK_UNREACHABLE, packet->sIP);
	}

      return ;
    }

  /* verify checksum */
  check = net_16_load(hdr->check);
  net_16_store(hdr->check, 0);
  computed_check = packet_checksum((uint8_t *)hdr, hdr->ihl * 4);
  /* incorrect packet */
  if (check != computed_check)
    {
      net_debug("Rejected incorrect packet\n");
      return;
    }

  /* fix the packet size (if the interface has a minimum transfert unit) */
  tot_len = net_be16_load(hdr->tot_len);
  if (nethdr->size != tot_len)
    {
      int_fast8_t	i;
      uint_fast16_t	delta = nethdr->size - tot_len;

      for (i = packet->stage; i >= 0; i--)
	packet->header[i].size -= delta;
    }

  /* next stage */
  if (!nethdr[1].data)
    {
      uint_fast8_t	hdr_len = hdr->ihl * 4;

      nethdr[1].data = nethdr->data + hdr_len;
      nethdr[1].size = nethdr->size - hdr_len;
    }

  /* increment stage */
  packet->stage++;

  /* is the packet fragmented ? */
  fragment = net_be16_load(hdr->fragment);
  if ((fragment & IP_FLAG_MF) || (fragment & IP_FRAG_MASK))
    {
      /* add fragment */
      if (ip_fragment_pushpkt(protocol, packet, hdr))
	{
	  /* last fragment: reassemble */

	  /* probably nothing here */
	}
      else
	return;	/* abord the packet, the last fragment will unblock it */
    }

  /* dispatch to the matching protocol */
  proto = hdr->protocol;
  if ((p = net_protos_lookup(&interface->protocols, proto)))
    p->desc->pushpkt(interface, packet, protocol, p);
  else
    net_debug("IP: no protocol to handle packet (id = 0x%x)\n", proto);
}

/*
 * Prepare a new IP packet.
 */

NET_PREPAREPKT(ip_preparepkt)
{
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = if_preparepkt(interface, packet, 20 + size, 4 + max_padding - 1);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = if_preparepkt(interface, packet, 20 + size, 0);
#endif

  nethdr = &packet->header[packet->stage];
  nethdr->data = next;
  nethdr->size = 20 + size;

  packet->stage++;

  return next + 20;
}

/*
 * Fragment sending.
 */

static inline uint_fast8_t ip_send_fragment(struct net_proto_s	*ip,
					    struct net_if_s	*interface,
					    struct iphdr	*hdr,
					    struct net_packet_s	*packet,
					    uint_fast16_t	offs,
					    size_t		fragsz,
					    uint_fast8_t	last)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)ip->pv;
  struct net_packet_s	*frag;
  struct net_header_s	*nethdr;
  struct net_route_s	*route_entry;
  struct iphdr		*hdr_frag;
  uint8_t		*dest;
  uint_fast8_t		i;

  /* prepare a new IP packet */
  frag = packet_obj_new(NULL);
  packet_obj_refnew(packet);
  frag->parent = packet;
  frag->header[frag->stage + 1].data = NULL;

  dest = ip_preparepkt(interface, frag, 0, 0);

  net_debug("sending fragment %d-%d\n", offs, offs + fragsz);

  /* fill the data */
  frag->header[frag->stage].data = packet->header[packet->stage + 1].data + offs;
  frag->header[frag->stage].size = fragsz;
  for (i = 0; i < frag->stage; i++)
    {
      frag->header[i].size += fragsz;
    }
  frag->stage--;

  /* copy header */
  nethdr = &frag->header[frag->stage];
  hdr_frag = (struct iphdr *)nethdr->data;
  memcpy(hdr_frag, hdr, 20);

  /* setup fragment specific fields */
  net_be16_store(hdr_frag->fragment, (last ? 0 : IP_FLAG_MF) | (offs / 8));
  net_be16_store(hdr_frag->tot_len, nethdr->size);
  net_16_store(hdr_frag->check, 0);
  /* checksum XXX don't recompute it! */
  net_16_store(hdr_frag->check,
	       packet_checksum((uint8_t *)hdr_frag, 20));

  /* send the fragment */
  frag->stage--;

  IPV4_ADDR_SET(frag->sADDR, net_be32_load(hdr_frag->saddr));
  IPV4_ADDR_SET(frag->tADDR, net_be32_load(hdr_frag->daddr));
  /* need to route ? */
  if (ip_delivery(interface, ip, frag->tADDR.addr.ipv4) == IP_DELIVERY_INDIRECT)
    {
      if ((route_entry = route_get(interface, &frag->tADDR)))
	{
	  if (!(frag->tMAC = arp_get_mac(ip, pv->arp, frag,
					 IPV4_ADDR_GET(route_entry->router))))
	    return 1;
	}
      else
	{
	  /* network unreachable */
	  //icmp_error(interface, ICMP_NETWORK_UNREACHABLE, packet->sIP);
	  return 0;
	}
    }
  else
    {
      /* no route IP -> MAC translation */
      if (!(frag->tMAC = arp_get_mac(ip, pv->arp, frag, frag->tADDR.addr.ipv4)))
	return 1;
    }

  /* send the packet to the driver */
  if_sendpkt(interface, frag, ETHERTYPE_IP);
  return 1;
}

/*
 * Send an IP packet.
 */

void			ip_send(struct net_if_s		*interface,
				struct net_packet_s	*packet,
				struct net_proto_s	*ip,
				struct net_proto_s	*proto)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)ip->pv;
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  struct net_route_s	*route_entry;
  uint_fast16_t		total;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* start filling common IP header fields */
  hdr->version = 4;
  hdr->ihl = 5;
  hdr->tos = 0;
  hdr->ttl = 64;
  hdr->protocol = proto->id;
  net_be32_store(hdr->saddr, pv->addr);
  net_be32_store(hdr->daddr, IPV4_ADDR_GET(packet->tADDR));

  total = nethdr[1].size;

  /* need fragmentation */
  if (total > interface->mtu - 20)
    {
      uint_fast16_t		id;
      uint_fast16_t		offs;
      uint_fast16_t		fragsz;
      uint_fast8_t		sent;
      uint8_t			*data;

      data = nethdr[1].data;
      offs = 0;
      fragsz = (interface->mtu - 20) & ~7;
      /* choose a random identifier */
      id = pv->id_seq++;
      net_be16_store(hdr->id, id);

      /* send the middle fragments */
      sent = 1;
      while (sent && offs + fragsz < total)
	{
	  sent = ip_send_fragment(ip, interface, hdr, packet, offs, fragsz, 0);

	  offs += fragsz;
	}

      /* last packet */
      if (sent)
	ip_send_fragment(ip, interface, hdr, packet, offs, total - offs, 1);

      /* release the original packet */
      packet_obj_refdrop(packet);
      return ;
    }

  /* for the non fragmented packets */
  /* finish the IP header */
  net_be16_store(hdr->tot_len, nethdr->size);
  net_16_store(hdr->id, pv->id_seq++);
  net_16_store(hdr->fragment, 0);
  net_16_store(hdr->check, 0);
  /* checksum */
  net_16_store(hdr->check, packet_checksum((uint8_t *)hdr, hdr->ihl * 4));

  packet->stage--;
  IPV4_ADDR_SET(packet->sADDR, pv->addr);
  /* need to route ? */
  if (ip_delivery(interface, ip, packet->tADDR.addr.ipv4) == IP_DELIVERY_INDIRECT)
    {
      if ((route_entry = route_get(interface, &packet->tADDR)))
	{
	  if (!(packet->tMAC = arp_get_mac(ip, pv->arp, packet,
					 IPV4_ADDR_GET(route_entry->router))))
	    return;
	}
      else
	{
	  /* network unreachable */
	  //icmp_error(interface, ICMP_NETWORK_UNREACHABLE, packet->sIP);

	  return;
	}
    }
  else
    {
      /* no route IP -> MAC translation */
      if (!(packet->tMAC = arp_get_mac(ip, pv->arp, packet, packet->tADDR.addr.ipv4)))
	return;
    }

  /* send the packet to the driver */
  if_sendpkt(interface, packet, ETHERTYPE_IP);
}

/*
 * Route a packet.
 */

void		ip_route(struct net_if_s	*interface,
			 struct net_packet_s	*packet,
			 struct net_route_s	*route)
{
  struct net_pv_ip_s	*pv;
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  uint_fast32_t		router;
  uint_fast16_t		total;

  packet_obj_refnew(packet);

  interface = route->interface;
  pv = (struct net_pv_ip_s *)interface->ip->pv;

  /* get the packet header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* decrement TTL */
  hdr->ttl--;

  if (!nethdr[1].data)
    {
      uint_fast8_t	hdr_len = hdr->ihl * 4;
      nethdr[1].data = nethdr->data + hdr_len;
      nethdr[1].size = net_be16_load(hdr->tot_len) - hdr_len;
    }

  total = nethdr[1].size;

  /* check for fragmentation XXX need to be tested */
  if (total > interface->mtu - 20)
    {
      uint_fast16_t		offs;
      uint_fast16_t		shift;
      uint_fast16_t		fragsz;
      uint_fast8_t		sent;
      uint8_t			*data;
      uint_fast16_t		fragment;

      fragment = net_be16_load(hdr->fragment);

      net_debug("routing with fragmentation\n");

      /* if the Don't Fragment flag is set, destroy the packet */
      if (fragment & IP_FLAG_DF)
	{
	  packet_obj_refdrop(packet);
	  return;
	}

      data = nethdr[1].data;
      offs = 0;
      shift = (fragment & IP_FRAG_MASK) * 8;
      fragsz = (interface->mtu - 20) & ~7;

      /* send the middle fragments */
      sent = 1;
      while (sent && offs + fragsz < total)
	{
	  sent = ip_send_fragment(interface->ip, interface, hdr, packet, shift + offs, fragsz, 0);

	  offs += fragsz;
	}

      /* last packet */
      if (sent)
	ip_send_fragment(interface->ip, interface, hdr, packet, shift + offs, total - offs, !(fragment & IP_FLAG_MF));

      /* release the original packet */
      packet_obj_refdrop(packet);
      return ;
    }

  /* recompute checksum*/
  net_16_store(hdr->check, 0);
  net_16_store(hdr->check, packet_checksum((uint8_t *)hdr, hdr->ihl * 4));

  /* send the packet */
  packet->stage--;

  /* direct or indirect delivery */
  if (route->type & ROUTETYPE_DIRECT)
    {
      net_debug("local delivery on %s\n", interface->name);

      if (!(packet->tMAC = arp_get_mac(route->addressing , pv->arp, packet, packet->tADDR.addr.ipv4)))
	return;
    }
  else
    {
      /* get router address */
      router = IPV4_ADDR_GET(route->router);

      net_debug("remote delivery thru %P on %s\n", &router, 4, interface->name);

      if (!(packet->tMAC = arp_get_mac(route->addressing, pv->arp, packet, router)))
	return;
    }

  /* send the packet to the driver */
  if_sendpkt(interface, packet, ETHERTYPE_IP);
}
