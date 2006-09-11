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
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s*)proto->pv;
  struct net_proto_s	*arp = va_arg(va, struct net_proto_s *);

  pv->arp = arp;
  printf("IP %s with ARP (%p)\n", pv->arp ? "bound" : "not bound", pv->arp);
  memset(pv->addr, 0, 4);
}

/*
 * Receive incoming IP packets.
 */

NET_PUSHPKT(ip_pushpkt)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s*)protocol->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct iphdr		aligned;
#endif
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  uint_fast16_t		hdr_len;
  net_proto_id_t	proto;
  struct net_proto_s	*p;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct iphdr));
      hdr = &aligned;
    }
#endif

  /* update packet info */
  packet->sIP = (uint8_t*)&hdr->saddr;
  packet->tIP = (uint8_t*)&hdr->daddr;

  /* XXX check IP version, checksum */

  /* XXX fragments, etc... */

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

  packet->stage++;

  /* dispatch to the matching protocol */
  proto = hdr->protocol;
  if ((p = net_protos_lookup(protocols, proto)))
    p->desc->pushpkt(dev, packet, p, protocols);
  else
    printf("NETWORK: no protocol to handle packet (id = 0x%x)\n", proto);
}

/*
 * Prepare a new IP packet.
 */

NET_PREPAREPKT(ip_preparepkt)
{
  struct net_header_s	*nethdr;

  dev_net_preparepkt(dev, packet, 20 + size); /* XXX */

  nethdr = &packet->header[packet->stage];
  nethdr[1].data = nethdr->data + 20; /* XXX */
  nethdr[1].size = size;

  packet->stage++;
}

/*
 * Send an IP packet.
 */

NET_IP_SEND(ip_send)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s*)ip->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct iphdr		aligned;
#endif
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    hdr = &aligned;
#endif

  /* fill IP header */
  hdr->version = 4;
  hdr->ihl = 5;
  hdr->tos = 0;
  net_be16_store(hdr->tot_len, nethdr->size);
  net_be16_store(hdr->id, 0);	/* XXX */
  hdr->fragment = 0;		/* XXX */
  hdr->flags = 0;		/* XXX */
  hdr->ttl = 64;
  hdr->protocol = proto->id;
  memcpy(&hdr->saddr, pv->addr, 4);
  memcpy(&hdr->daddr, packet->tIP, 4);
  /* checksum */
  endian_16_na_store(&hdr->check, packet_checksum(hdr, hdr->ihl * 4));

#ifdef CONFIG_NETWORK_AUTOALIGN
  if (hdr == &aligned)
    memcpy(nethdr->data, hdr, sizeof (struct iphdr));
  hdr = (struct iphdr*)nethdr->data;
#endif

  packet->stage--;
  packet->sIP = (uint8_t*)&hdr->saddr;
  packet->tIP = (uint8_t*)&hdr->daddr;
  if (!(packet->tMAC = arp_get_mac(dev, pv->arp, packet, packet->tIP)))
    return;

  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_IP);
}

