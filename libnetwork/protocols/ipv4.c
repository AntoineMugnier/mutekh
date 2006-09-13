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
 * IP protocol
 *
 */

#include <netinet/ip.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <hexo/device.h>

#include <stdio.h>

/*
 * Fragment lists.
 */

CONTAINER_FUNC(static inline, ip_packet, HASHLIST, ip_packet, NOLOCK, list_entry, UNSIGNED, id);

CONTAINER_FUNC(static inline, ip_fragment, DLIST, ip_fragment, NOLOCK, ip_fragment_entry);

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct ip_interface_s	ip_interface =
{
  .send = ip_send
};

const struct net_proto_desc_s	ip_protocol =
  {
    .name = "IP",
    .id = ETHERTYPE_IP,
    .pushpkt = ip_pushpkt,
    .preparepkt = ip_preparepkt,
    .initproto = ip_init,
    .f.ip = &ip_interface,
    .pv_size = sizeof (struct net_pv_ip_s),
  };

/*
 * Initialize private data of IP module.
 */

NET_INITPROTO(ip_init)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)proto->pv;
  struct net_proto_s	*arp = va_arg(va, struct net_proto_s *);

  pv->arp = arp;
  printf("IP %s with ARP (%p)\n", pv->arp ? "bound" : "not bound", pv->arp);
  memset(pv->addr, 0, 4);
  ip_packet_init(&pv->fragments);
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
  struct net_packet_s	*frag;
  uint_fast64_t		id;
  uint_fast16_t		offs;
  uint_fast16_t		fragment;
  uint_fast16_t		datasz;
  uint_fast16_t		total;
  uint_fast16_t		headers_len;
  uint8_t		*data;
  uint32_t		*ip_addr = (uint32_t *)packet->sIP;

  /* the unique identifier of the packet is the concatenation of the
     source address and the packet id */
  id =  (net_32_load(*ip_addr)) + (net_be16_load(hdr->id) << 32);

  /* extract some useful fields */
  fragment = net_be16_load(hdr->fragment);
  offs = (fragment & IP_FRAG_MASK) * 8;
  datasz = net_be16_load(hdr->tot_len) - hdr->ihl * 4;

  printf("fragment id %x offs %d size %d\n", id, offs, datasz);

  /* do we already received packet with same id ? */
  if (!(p = ip_packet_lookup(&pv->fragments, id)))
    {
      p = mem_alloc(sizeof (struct ip_packet_s), MEM_SCOPE_THREAD);
      p->id = id;
      p->size = 0;
      p->received = 0;
      ip_fragment_init(&p->packets);
      ip_packet_push(&pv->fragments, p);
    }
  p->received += datasz;
  /* try to determine the total size */
  if (!(fragment & IP_FLAG_MF))
    {
      p->size = offs + datasz;
      printf("packet total size %d\n", p->size);
    }

  total = p->size;

  if (total)
    printf("received %d out of %d\n", p->received, total);

  if (total && total == p->received)
    {
      printf("packet complete\n");

      /* we received the whole packet, reassemble now */
      nethdr = &packet->header[packet->stage];
      headers_len = (packet->header[0].size - nethdr->size);
      /* allocate a packet large enough */
      data = mem_alloc(total + headers_len, MEM_SCOPE_THREAD);

      /* copy previous headers (ethernet, ip, etc.) */
      memcpy(data, packet->packet, headers_len);

      printf("copying headers : %d-%d\n", 0, headers_len);

      /* copy current packet to its position */
      memcpy(data + headers_len + offs, nethdr->data, datasz);

      printf("copying packet : %d-%d\n", offs, offs + datasz);

      /* release current packet data */
      mem_free(packet->packet);

      /* replace by reassembling packet */
      packet->packet = data;
      data += headers_len;

      nethdr->data = data;
      nethdr->size = total;

      /* loop through previously received packets and reassemble them */
      while ((frag = ip_fragment_pop(&p->packets)))
	{
	  /* copy data in place */
	  nethdr = &frag->header[packet->stage];
	  offs = (net_be16_load(((struct iphdr *)nethdr[-1].data)->fragment) & IP_FRAG_MASK) * 8;
	  memcpy(data + offs, nethdr->data, nethdr->size);

	  printf("copying packet : %d-%d\n", offs, offs + nethdr->size);

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
  ip_fragment_push(&p->packets, packet);
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
  uint_fast16_t		hdr_len;
  net_proto_id_t	proto;
  struct net_proto_s	*p;
  uint_fast16_t		check;
  uint_fast16_t		computed_check;
  uint_fast16_t		fragment;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct iphdr));
      hdr = &aligned;
    }
#endif

  /* update packet info */
  packet->sIP = (uint8_t *)&hdr->saddr;
  packet->tIP = (uint8_t *)&hdr->daddr;

  /* check IP version */
  if (hdr->version != 4)
    return;

  /* verify checksum */
  check = net_16_load(hdr->check);
  net_16_store(hdr->check, 0);
  computed_check = packet_checksum((uint8_t *)hdr, hdr->ihl * 4);
  /* incorrect packet */
  if (check != computed_check)
    {
      printf("Rejected incorrect packet\n");
      return;
    }

  /* is the packet really for me ? */
  if (memcmp(&hdr->daddr, pv->addr, 4))
    return;

  /* next stage */
  if (!nethdr[1].data)
    {
      hdr_len = hdr->ihl * 4;
      nethdr[1].data = nethdr->data + hdr_len;
      nethdr[1].size = net_be16_load(hdr->tot_len) - hdr_len;
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

	  dummy_push(dev, packet, NULL, NULL);

	  /* probably nothing here */
	}
      else
	return;	/* abord the packet, the last fragment will unblock it */
    }

  /* dispatch to the matching protocol */
  proto = hdr->protocol;
  if ((p = net_protos_lookup(protocols, proto)))
    p->desc->pushpkt(dev, packet, p, protocols);
  else
    printf("IP: no protocol to handle packet (id = 0x%x)\n", proto);
}

/*
 * Prepare a new IP packet.
 */

NET_PREPAREPKT(ip_preparepkt)
{
  struct net_header_s	*nethdr;
  uint8_t		*next;

  next = dev_net_preparepkt(dev, packet, 20 + size);

  nethdr = &packet->header[packet->stage];
#ifdef CONFIG_NETWORK_AUTOALIGN
  /* XXX align here */
  /* next = ... */
#endif
  nethdr->data = next;
  nethdr->size = 20 + size;

  /* XXX remove this */
  packet->id = 0;
  packet->fragment = 0;

  packet->stage++;

  return next + 20;
}

/*
 * Send an IP packet.
 */

NET_IP_SEND(ip_send)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s *)ip->pv;
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  uint_fast16_t		total;
  uint_fast16_t		id;
  uint_fast16_t		offs;
  uint_fast16_t		fragsz;;
  struct net_packet_s	*frag;
  uint8_t		*data;
  uint8_t		*dest;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  total = nethdr[1].size;

  /* need fragmentation */
  if (total > IPMTU - 20)
    {
      data = nethdr[1].data;
      offs = 0;
      fragsz = (IPMTU - 20) & ~7;
      /* choose a random identifier */
      id = rand() & 0xffff; /* XXX */

      while (offs + fragsz < total)
	{
	  /* prepare a new IP packet */
	  frag = packet_obj_new(NULL);
	  dest = ip_preparepkt(dev, frag, fragsz);

	  printf("sending fragment %d-%d\n", offs, offs + fragsz);

	  /* fill the data */
	  memcpy(dest, data + offs, fragsz);

	  /* copy destination address */
	  frag->tIP = packet->tIP;

	  /* setup fragment specific fields */
	  frag->id = id;
	  frag->fragment |= IP_FLAG_MF | (offs / 8);

	  /* XXX setup the whole packet here */

	  /* send the fragments */
	  frag->stage--;
	  ip_send(dev, frag, ip, proto);
	  offs += fragsz;
	}

      /* last packet */
      frag = packet_obj_new(NULL);
      dest = ip_preparepkt(dev, frag, total - offs);

      printf("sending last fragment %d-%d\n", offs, total);

      /* fill the data */
      memcpy(dest, data + offs, total - offs);

      /* copy destination address */
      frag->tIP = packet->tIP;

      /* setup fragment specific fields */
      frag->id = id;
      frag->fragment |= offs / 8;

      /* send the last fragment */
      frag->stage--;
      ip_send(dev, frag, ip, proto);

      /* release the original packet */
      packet_obj_refdrop(packet);
      return ;
    }

  /* fill IP header */
  hdr->version = 4;
  hdr->ihl = 5;
  hdr->tos = 0;
  net_be16_store(hdr->tot_len, nethdr->size);
  net_be16_store(hdr->id, packet->id);
  net_be16_store(hdr->fragment, packet->fragment);
  hdr->ttl = 64;
  hdr->protocol = proto->id;
  memcpy(&hdr->saddr, pv->addr, 4);
  memcpy(&hdr->daddr, packet->tIP, 4);
  net_16_store(hdr->check, 0);
  /* checksum */
  net_16_store(hdr->check, packet_checksum((uint8_t *)hdr, hdr->ihl * 4));

  packet->stage--;
  packet->sIP = (uint8_t *)&hdr->saddr;
  packet->tIP = (uint8_t *)&hdr->daddr;
  if (!(packet->tMAC = arp_get_mac(dev, pv->arp, packet, packet->tIP)))
    return ;

  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_IP);
}

