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
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <hexo/device.h>
#include <hexo/driver.h>

#include <stdio.h>
#include <stdlib.h>

/*
 * Fragment lists.
 */

CONTAINER_FUNC(static inline, ip_packet, HASHLIST, ip_packet, NOLOCK, list_entry, UNSIGNED, id);

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
  memset(pv->addr, 0, 4);
  ip_packet_init(&pv->fragments);
  srand((uint_fast32_t)pv);
  pv->id_seq = rand();
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
  uint_fast64_t		id;
  uint_fast16_t		offs;
  uint_fast16_t		fragment;
  uint_fast16_t		datasz;
  uint_fast16_t		total;
  uint32_t		*ip_addr = (uint32_t *)packet->sIP;

  /* the unique identifier of the packet is the concatenation of the
     source address and the packet id */
  id =  (net_32_load(*ip_addr)) + (net_be16_load(hdr->id) << 32); /* XXX */

  /* extract some useful fields */
  fragment = net_be16_load(hdr->fragment);
  offs = (fragment & IP_FRAG_MASK) * 8;
  datasz = net_be16_load(hdr->tot_len) - hdr->ihl * 4;

  net_debug("fragment id %x offs %d size %d\n", id, offs, datasz);

  /* do we already received packet with same id ? */
  if (!(p = ip_packet_lookup(&pv->fragments, id)))
    {
      p = mem_alloc(sizeof (struct ip_packet_s), MEM_SCOPE_CONTEXT);
      p->id = id;
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
      net_debug("Rejected incorrect packet\n");
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
  next = dev_net_preparepkt(dev, packet, 20 + size, 4 + max_padding - 1);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = dev_net_preparepkt(dev, packet, 20 + size, 0);
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

/* XXX refaire ca sans memcpy recount parent tt ca tt ca :) */

static inline	void	ip_send_fragment(struct net_pv_ip_s	*pv,
					 struct device_s	*dev,
					 struct iphdr		*hdr,
					 uint8_t		*data,
					 uint_fast16_t		offs,
					 size_t			fragsz,
					 uint_fast8_t		last)
{
  struct net_packet_s	*frag;
  struct net_header_s	*nethdr;
  struct iphdr		*hdr_frag;
  uint8_t		*dest;

  /* prepare a new IP packet */
  frag = packet_obj_new(NULL);
  dest = ip_preparepkt(dev, frag, fragsz, 0);
  frag->stage--;

  net_debug("sending fragment %d-%d\n", offs, offs + fragsz);

  /* fill the data */
  memcpy(dest, data + offs, fragsz);

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

  frag->sIP = (uint8_t *)&hdr_frag->saddr;
  frag->tIP = (uint8_t *)&hdr_frag->daddr;
  if (!(frag->tMAC = arp_get_mac(dev, pv->arp, frag, frag->tIP)))
    return;

  /* send the packet to the driver */
  dev_net_sendpkt(dev, frag, ETHERTYPE_IP);
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

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr *)nethdr->data;

  /* start filling common IP header fields */
  hdr->version = 4;
  hdr->ihl = 5;
  hdr->tos = 0;
  hdr->ttl = 64;
  hdr->protocol = proto->id;
  /* XXX manage IP as uint32 */
  memcpy(&hdr->saddr, pv->addr, 4);
  memcpy(&hdr->daddr, packet->tIP, 4);

  total = nethdr[1].size;

  /* need fragmentation */
  if (total > IPMTU - 20)
    {
      uint_fast16_t		id;
      uint_fast16_t		offs;
      uint_fast16_t		fragsz;;
      uint8_t			*data;

      data = nethdr[1].data;
      offs = 0;
      fragsz = (IPMTU - 20) & ~7;
      /* choose a random identifier */
      id = pv->id_seq + (rand() & 0xff);
      net_be16_store(hdr->id, id);
      pv->id_seq = id + 1;

      /* send the middle fragments */
      while (offs + fragsz < total)
	{
	  ip_send_fragment(pv, dev, hdr, data, offs, fragsz, 0);

	  offs += fragsz;
	}

      /* last packet */
      ip_send_fragment(pv, dev, hdr, data, offs, total - offs, 1);

      /* release the original packet */
      packet_obj_refdrop(packet);
      return ;
    }

  /* for the non fragmented packets */
  /* finish the IP header */
  net_be16_store(hdr->tot_len, nethdr->size);
  net_16_store(hdr->id, 0);
  net_16_store(hdr->fragment, 0);
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

