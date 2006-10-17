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
#include <netinet/icmp.h>
#include <netinet/arp.h>
#include <netinet/packet.h>
#include <netinet/ether.h>
#include <netinet/protos.h>

#include <netinet/if.h>
#include <netinet/route.h>

#include <stdio.h>
#include <stdlib.h>
#include <timer.h>

/*
 * Fragment lists.
 */

CONTAINER_FUNC(static inline, ip_packet, HASHLIST, ip_packet, NOLOCK, id);
CONTAINER_KEY_FUNC(static inline, ip_packet, HASHLIST, ip_packet, NOLOCK, id);

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_addressing_interface_s	ip_interface =
  {
    .sendpkt = ip_send,
    .matchaddr = ip_matchaddr,
    .pseudoheader_checksum = ip_pseudoheader_checksum,
    .errormsg = icmp_errormsg
  };

const struct net_proto_desc_s	ip_protocol =
  {
    .name = "IP",
    .id = ETHERTYPE_IP,
    .pushpkt = ip_pushpkt,
    .preparepkt = ip_preparepkt,
    .initproto = ip_init,
    .f.addressing = &ip_interface,
    .pv_size = sizeof (struct net_pv_ip_s)
  };

/*
 * Initialize private data of IP module.
 */

NET_INITPROTO(ip_init)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)proto->pv;
  struct net_proto_s	*arp = va_arg(va, struct net_proto_s *);
  struct net_proto_s	*icmp = va_arg(va, struct net_proto_s *);
  uint_fast32_t		ip = va_arg(va, uint_fast32_t);
  uint_fast32_t		mask = va_arg(va, uint_fast32_t);

  pv->interface = interface;
  pv->arp = arp;
  pv->icmp = icmp;
  pv->addr = ip;
  pv->mask = mask;
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

  /* masked destination address must be equal to masked local address */
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
      /* initialize the reassembly structure */
      p = mem_alloc(sizeof (struct ip_packet_s), MEM_SCOPE_CONTEXT);
      memcpy(p->id, id, 6);
      p->size = 0;
      p->received = 0;
      p->addressing = ip;
      packet_queue_init(&p->packets);
      ip_packet_push(&pv->fragments, p);

      /* start timeout timer */
      p->timeout.callback = ip_fragment_timeout;
      p->timeout.pv = (void *)p;
      p->timeout.delay = IP_REASSEMBLY_TIMEOUT;
      timer_add_event(&timer_ms, &p->timeout);
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
      uint8_t			*ptr;

      net_debug("packet complete\n");

      /* disable timeout */
      timer_cancel_event(&p->timeout, 0);

      /* we received the whole packet, reassemble now */
      nethdr = &packet->header[packet->stage];
      headers_len = (packet->header[0].size - nethdr->size);
      /* allocate a packet large enough */
      data = mem_alloc(total + headers_len + 3, MEM_SCOPE_CONTEXT);

      /* copy previous headers (ethernet, ip, etc.) */
      memcpy(data, packet->packet, headers_len);

      net_debug("copying headers : %d-%d\n", 0, headers_len);

      /* copy current packet to its position */
#ifdef CONFIG_NETWORK_AUTOALIGN
      ptr = (uint8_t *)ALIGN_VALUE((uintptr_t)(data + headers_len), 4);
#else
      ptr = data + headers_len;
#endif
      memcpy(ptr + offs, nethdr->data, datasz);

      net_debug("copying packet : %d-%d\n", offs, offs + datasz);

      /* release current packet data */
      mem_free(packet->packet);

      /* replace by reassembling packet */
      packet->packet = data;
      data = ptr;

      nethdr->data = data;
      nethdr->size = total;

      /* loop through previously received packets and reassemble them */
      while ((frag = packet_queue_pop(&p->packets)))
	{
	  /* copy data in place */
	  nethdr = &frag->header[packet->stage];
	  offs = (net_be16_load(((struct iphdr *)nethdr[-1].data)->fragment) & IP_FRAG_MASK) * 8;

	  /* check for overflow */
	  if (offs >= total)
	    {
	      pv->icmp->desc->f.control->errormsg(frag, ERROR_BAD_HEADER);

	      /* delete all the packets */
	      packet_obj_refdrop(frag);
	      while ((frag = packet_queue_pop(&p->packets)))
		packet_obj_refdrop(frag);
	      packet_obj_refdrop(packet);
	      packet_queue_destroy(&p->packets);
	      ip_packet_remove(&pv->fragments, p);
	      mem_free(p);

	      return 0;
	    }

	  memcpy(data + offs, nethdr->data, nethdr->size);

	  net_debug("copying packet : %d-%d\n", offs, offs + nethdr->size);

	  /* release our reference to the packet */
	  packet_obj_refdrop(frag);
	}

      /* release memory */
      packet_queue_destroy(&p->packets);
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
 * Fragment reassembly timeout.
 */

TIMER_CALLBACK(ip_fragment_timeout)
{
  struct ip_packet_s	*p = (struct ip_packet_s *)pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)p->addressing->pv;
  struct net_packet_s	*packet;

  packet = packet_queue_pop(&p->packets);
  packet->stage--;

  /* report the error */
  pv_ip->icmp->desc->f.control->errormsg(packet, ERROR_FRAGMENT_TIMEOUT);

  /* delete all the packets */
  packet_obj_refdrop(packet);
  while ((packet = packet_queue_pop(&p->packets)))
    packet_obj_refdrop(packet);
  packet_queue_destroy(&p->packets);
  ip_packet_remove(&pv_ip->fragments, p);
  mem_free(p);
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
  uint_fast16_t		computed_check;
  uint_fast16_t		fragment;
  uint_fast16_t		tot_len;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* align the packet on 32 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!IS_ALIGNED(hdr, sizeof (uint32_t)))
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
  packet->source_addressing = protocol;

  /* is the packet really for me ? */
  if (packet->tADDR.addr.ipv4 != pv->addr)
    {
      struct net_route_s	*route_entry = NULL;

      /* is there a route for this address ? */
      if ((route_entry = route_get(interface, &packet->tADDR)) != NULL)
	{
	  net_debug("routing to host %P\n", &packet->tADDR.addr.ipv4, 4);
	  /* route the packet */
	  ip_route(packet, route_entry);
	}
      else
	{
	  net_debug("no route to host %P\n", &packet->tADDR.addr.ipv4, 4);
	  /* network unreachable */
	  pv->icmp->desc->f.control->errormsg(packet, ERROR_NET_UNREACHABLE);
	}

      return ;
    }

  /* verify checksum */
  computed_check = packet_checksum((uint8_t *)hdr, hdr->ihl * 4);
  /* incorrect packet */
  if (computed_check != 0xffff)
    {
      net_debug("IP: Rejected incorrect packet %x\n", computed_check);
      return;
    }

  /* fix the packet size (if the interface has a minimum transfert
     unit).  this must be done because some other modules in the
     protocol stack use the size specified in the header array, and if
     the network device set a bad size, we must adjust it */
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
      if (!ip_fragment_pushpkt(protocol, packet, hdr))
	return;	/* abord the packet, the last fragment will unblock it */
      /* otherwise, the packet has been reassembled and is ready to continue its path */
    }

  /* dispatch to the matching protocol */
  proto = hdr->protocol;
  if ((p = net_protos_lookup(&interface->protocols, proto)))
    p->desc->pushpkt(interface, packet, p);
  else
    pv->icmp->desc->f.control->errormsg(packet, ERROR_PROTO_UNREACHABLE);
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
  uint_fast32_t		check;

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
  memcpy(hdr_frag, hdr, hdr->ihl * 4);

  /* setup fragment specific fields */
  net_be16_store(hdr_frag->fragment, (last ? 0 : IP_FLAG_MF) | (offs / 8));
  net_be16_store(hdr_frag->tot_len, nethdr->size);
  /* finalize checksum computation */
  check = net_16_load(hdr_frag->check);
  check += net_16_load(hdr_frag->fragment);
  check += net_16_load(hdr_frag->tot_len);
  check = check + (check >> 16);
  net_16_store(hdr_frag->check, ~check);

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
	  pv->icmp->desc->f.control->errormsg(frag, ERROR_NET_UNREACHABLE);

	  packet_obj_refdrop(frag);
	  return 0;
	}
    }
  else
    {
      /* no route: IP -> MAC translation */
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

NET_SENDPKT(ip_send)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)protocol->pv;
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  struct net_route_s	*route_entry;
  uint_fast16_t		total;

  packet->source_addressing = protocol;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* start filling common IP header fields */
  hdr->version = 4;
  hdr->ihl = 5;
  hdr->tos = 0;
  hdr->ttl = 64;
  hdr->protocol = proto;
  net_be32_store(hdr->saddr, pv->addr);
  net_be32_store(hdr->daddr, IPV4_ADDR_GET(packet->tADDR));

  total = nethdr[1].size;

  /* need fragmentation */
  if (total > interface->mtu - 20)
    {
      uint_fast16_t		offs;
      uint_fast16_t		fragsz;
      uint_fast8_t		sent;
      uint8_t			*data;

      data = nethdr[1].data;
      offs = 0;
      fragsz = (interface->mtu - 20) & ~7;
      /* choose a random identifier */
      net_be16_store(hdr->id, pv->id_seq++);
      /* compute the (partial) checksum of the header. as the header will be
	 completed during next step, the checksum will be incrementaly
	 updated */
      net_16_store(hdr->tot_len, 0);
      net_16_store(hdr->fragment, 0);
      net_16_store(hdr->check, 0);
      net_16_store(hdr->check, packet_checksum((uint8_t *)hdr, 20));

      /* send the middle fragments */
      sent = 1;
      while (sent && offs + fragsz < total)
	{
	  sent = ip_send_fragment(protocol, interface, hdr, packet, offs, fragsz, 0);

	  offs += fragsz;
	}

      /* last fragment */
      if (sent)
	ip_send_fragment(protocol, interface, hdr, packet, offs, total - offs, 1);

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
  net_16_store(hdr->check, ~packet_checksum((uint8_t *)hdr, hdr->ihl * 4));

  packet->stage--;
  IPV4_ADDR_SET(packet->sADDR, pv->addr);
  /* need to route ? */
  if (ip_delivery(interface, protocol, packet->tADDR.addr.ipv4) == IP_DELIVERY_INDIRECT)
    {
      if ((route_entry = route_get(interface, &packet->tADDR)))
	{
	  if (!(packet->tMAC = arp_get_mac(protocol, pv->arp, packet,
					 IPV4_ADDR_GET(route_entry->router))))
	    return;
	}
      else
	{
	  /* network unreachable */
	  pv->icmp->desc->f.control->errormsg(packet, ERROR_NET_UNREACHABLE);

	  packet_obj_refdrop(packet);
	  return;
	}
    }
  else
    {
      /* no route: IP -> MAC translation */
      if (!(packet->tMAC = arp_get_mac(protocol, pv->arp, packet, packet->tADDR.addr.ipv4)))
	return;
    }

  /* send the packet to the driver */
  if_sendpkt(interface, packet, ETHERTYPE_IP);
}

/*
 * Route a packet.
 */

void		ip_route(struct net_packet_s	*packet,
			 struct net_route_s	*route)
{
  struct net_pv_ip_s	*pv;
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  struct net_if_s	*interface;
  uint_fast32_t		router;
  uint_fast16_t		total;
  uint_fast16_t		check;
  uint_fast8_t		old_ttl;

  packet_obj_refnew(packet);

  interface = route->interface;
  pv = (struct net_pv_ip_s *)route->addressing->pv;

  /* get the packet header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* decrement TTL */
  old_ttl = hdr->ttl--;

  /* check for timeout */
  if (hdr->ttl == 0)
    {
      pv->icmp->desc->f.control->errormsg(packet, ERROR_TIMEOUT);

      packet_obj_refdrop(packet);
      return;
    }

  if (!nethdr[1].data)
    {
      uint_fast8_t	hdr_len = hdr->ihl * 4;
      nethdr[1].data = nethdr->data + hdr_len;
      nethdr[1].size = net_be16_load(hdr->tot_len) - hdr_len;
    }

  total = nethdr[1].size;

  /* check for fragmentation */
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
	  /* report the error */
	  pv->icmp->desc->f.control->errormsg(packet, ERROR_CANNOT_FRAGMENT, route);

	  packet_obj_refdrop(packet);
	  return;
	}

      data = nethdr[1].data;
      offs = 0;
      shift = (fragment & IP_FRAG_MASK) * 8;
      /* XXX interesting idea: compute an optimal fragment size */
      fragsz = (interface->mtu - 20) & ~7;

      /* send the middle fragments */
      sent = 1;
      while (sent && offs + fragsz < total)
	{
	  sent = ip_send_fragment(route->addressing, interface, hdr, packet, shift + offs, fragsz, 0);

	  offs += fragsz;
	}

      /* last fragment */
      if (sent)
	ip_send_fragment(route->addressing, interface, hdr, packet, shift + offs, total - offs, !(fragment & IP_FLAG_MF));

      /* release the original packet */
      packet_obj_refdrop(packet);
      return ;
    }

  /* recompute checksum (in fact, incrementally adjust it) */
  check = net_16_load(hdr->check) + 1;
  net_16_store(hdr->check, check + (check >> 16));

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

/*
 * Address matching function.
 */

NET_MATCHADDR(ip_matchaddr)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)protocol->pv;
  uint_fast32_t		A;
  uint_fast32_t		B;
  uint_fast32_t		M;


  if (a)
    A = IPV4_ADDR_GET(*a);
  else
    A = pv->addr;
  if (b)
    B = IPV4_ADDR_GET(*b);
  else
    B = pv->addr;
  if (mask)
    M = IPV4_ADDR_GET(*mask);
  else /* no mask set, equality test */
    return A == B;

  /* masked equality test */
  return (A & M) == (B & M);
}

/*
 * Compute checksum of the IP pseudo-header (needed by higher protocols).
 */

NET_PSEUDOHEADER_CHECKSUM(ip_pseudoheader_checksum)
{
  struct ip_pseudoheader_s	hdr;

  if (addressing == NULL)
    hdr.source = endian_be32(IPV4_ADDR_GET(packet->sADDR));
  else
    hdr.source = endian_be32(((struct net_pv_ip_s *)addressing->pv)->addr);
  hdr.dest = endian_be32(IPV4_ADDR_GET(packet->tADDR));
  hdr.zero = 0;
  hdr.type = proto;
  hdr.size = endian_be16(size);

  return packet_checksum(&hdr, sizeof (hdr));
}
