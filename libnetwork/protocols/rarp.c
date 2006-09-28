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
 * Reverse ARP for IPv4
 *
 */

#include <netinet/arp.h>
#include <netinet/ip.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <netinet/if.h>
#include <netinet/route.h>
#include <netinet/in.h>

#include <hexo/endian.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	rarp_protocol =
  {
    .name = "RARP",
    .id = ETHERTYPE_REVARP,
    .pushpkt = rarp_pushpkt,
    .preparepkt = rarp_preparepkt,
    .initproto = rarp_init,
    .pv_size = sizeof (struct net_pv_rarp_s)
  };

/*
 * Init RARP.
 */

NET_INITPROTO(rarp_init)
{
  struct net_pv_rarp_s	*pv = (struct net_pv_rarp_s *)proto->pv;
  struct net_proto_s	*ip = va_arg(va, struct net_proto_s *);

  pv->ip = ip;
}

/*
 * Receive incoming RARP packets.
 */

NET_PUSHPKT(rarp_pushpkt)
{
  struct net_pv_rarp_s	*pv = (struct net_pv_rarp_s *)protocol->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)pv->ip->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_header_s	*nethdr;
  uint_fast32_t		ip;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* align the packet on 32 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint32_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct ether_arp));
      hdr = &aligned;
    }
#endif

  /* check header */
  if (net_be16_load(hdr->ea_hdr.ar_hrd) != ARPHRD_ETHER ||
      net_be16_load(hdr->ea_hdr.ar_pro) != ETHERTYPE_IP)
    return ;

  /* ARP reply message */
  if (net_be16_load(hdr->ea_hdr.ar_op) == ARPOP_RREPLY)
    {
      if (memcmp(packet->tMAC, hdr->arp_tha, ETH_ALEN))
	return ;

      /* assign IP */
      ip = pv_ip->addr = net_be32_load(hdr->arp_tpa);
      /* guess netmask */
      if (IN_CLASSA(ip))
	pv_ip->mask = IN_CLASSA_NET;
      else if (IN_CLASSB(ip))
	pv_ip->mask = IN_CLASSB_NET;
      else if (IN_CLASSC(ip))
	pv_ip->mask = IN_CLASSC_NET;

      net_debug("Assigned IP: %P, netmask: %P\n", &ip, 4, &pv_ip->mask, 4);
    }
}

/*
 * Prepare a RARP packet.
 */

NET_PREPAREPKT(rarp_preparepkt)
{
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = if_preparepkt(inteface, packet, sizeof (struct ether_arp), 4);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = if_preparepkt(interface, packet, sizeof (struct ether_arp), 0);
#endif

  nethdr = &packet->header[packet->stage];
  nethdr->data = next;
  nethdr->size = sizeof (struct ether_arp);

  nethdr[1].data = NULL;

  return NULL;
}

/*
 * Make a RARP request.
 */

void			rarp_request(struct net_if_s	*interface,
				     struct net_proto_s	*rarp,
				     uint8_t		*mac)
{
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  rarp_preparepkt(interface, packet, 0, 0);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* fill the request */
  net_be16_store(hdr->ea_hdr.ar_hrd, ARPHRD_ETHER);
  net_be16_store(hdr->ea_hdr.ar_pro, ETHERTYPE_IP);
  hdr->ea_hdr.ar_hln = ETH_ALEN;
  hdr->ea_hdr.ar_pln = 4;
  net_be16_store(hdr->ea_hdr.ar_op, ARPOP_RREQUEST);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  if (!mac)
    memcpy(hdr->arp_tha, packet->sMAC, ETH_ALEN);
  else
    memcpy(hdr->arp_tha, mac, ETH_ALEN);
  net_32_store(hdr->arp_spa, 0);
  net_32_store(hdr->arp_tpa, 0);

  packet->tMAC = (uint8_t *)"\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the interface */
  if_sendpkt(interface, packet, ETHERTYPE_REVARP);
}

