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
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <hexo/device.h>
#include <hexo/driver.h>

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
    .f.other = NULL,
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

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
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

      net_debug("Assigned IP: %d.%d.%d.%d\n", hdr->arp_tpa[0],
		hdr->arp_tpa[1], hdr->arp_tpa[2], hdr->arp_tpa[3]);

      memcpy(pv_ip->addr, hdr->arp_tpa, 4);
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
  next = dev_net_preparepkt(dev, packet, sizeof (struct ether_arp), 2);
  next = ALIGN_ADDRESS(next, 2);
#else
  next = dev_net_preparepkt(dev, packet, sizeof (struct ether_arp), 0);
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

void			rarp_request(struct device_s	*dev,
				     struct net_proto_s	*arp,
				     uint8_t		*mac)
{
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  rarp_preparepkt(dev, packet, 0, 0);

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
  memset(hdr->arp_spa, 0, 4);
  memset(hdr->arp_tpa, 0, 4);

  packet->tMAC = (uint8_t *)"\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_REVARP);
}

