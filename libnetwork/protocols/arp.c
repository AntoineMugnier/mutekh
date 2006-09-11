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

#include <netinet/arp.h>
#include <netinet/ip.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <hexo/device.h>

#include <stdio.h>

/*
 * ARP table functions.
 */

CONTAINER_FUNC(static inline, arp_table, HASHLIST, arp_table, NOLOCK, list_entry, BLOB, ip);

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct arp_interface_s	arp_interface =
{
  .request = arp_request,
  .reply = arp_reply,
  .update_table = arp_update_table,
  .get_mac = arp_get_mac
};

const struct net_proto_desc_s	arp_protocol =
  {
    .name = "ARP",
    .id = ETHERTYPE_ARP,
    .pushpkt = arp_pushpkt,
    .preparepkt = arp_preparepkt,
    .initproto = arp_init,
    .f.arp = &arp_interface,
    .pv_size = sizeof (struct net_pv_arp_s),
  };

/*
 * Init ARP.
 */

NET_INITPROTO(arp_init)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)proto->pv;
  struct net_proto_s	*ip = va_arg(va, struct net_proto_s *);

  arp_table_init(&pv->table);
  pv->ip = ip;
  printf("ARP %s with IP (%p)\n", pv->ip ? "bound" : "not bound", pv->ip);
}

/*
 * This function decodes an incoming ARP packet.
 */

NET_PUSHPKT(arp_pushpkt)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)protocol->pv;
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

  /* ARP message */
  switch (net_be16_load(hdr->ea_hdr.ar_op))
    {
      case ARPOP_REQUEST:
	printf("Requested %d.%d.%d.%d\n",
	       hdr->arp_tpa[0], hdr->arp_tpa[1], hdr->arp_tpa[2],
	       hdr->arp_tpa[3]);
	if (!memcmp(hdr->arp_tpa, pv_ip->addr, 4))
	  {
	    printf("It's me !\n");
	    arp_reply(dev, protocol, hdr->arp_sha, hdr->arp_spa);
	  }
	/* no break here since we also need to refresh cache */
      case ARPOP_REPLY:
	arp_update_table(dev, protocol, hdr->arp_spa, hdr->arp_sha);
	break;
      default:
	break;
    }
}

/*
 * This function prepares an ARP packet.
 */

NET_PREPAREPKT(arp_preparepkt)
{
  dev_net_preparepkt(dev, packet, sizeof (struct ether_arp));
}

/*
 * This function request a MAC address given an IP address.
 */

NET_ARP_REQUEST(arp_request)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)pv->ip->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  arp_preparepkt(dev, packet, 0);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    hdr = &aligned;
#endif

  /* fill the request */
  net_be16_store(hdr->ea_hdr.ar_hrd, ARPHRD_ETHER);
  net_be16_store(hdr->ea_hdr.ar_pro, ETHERTYPE_IP);
  hdr->ea_hdr.ar_hln = ETH_ALEN;
  hdr->ea_hdr.ar_pln = 4;
  net_be16_store(hdr->ea_hdr.ar_op, ARPOP_REQUEST);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  memcpy(hdr->arp_spa, pv_ip->addr, 4);
  memset(hdr->arp_tha, 0xff, ETH_ALEN);
  memcpy(hdr->arp_tpa, address, 4);

#ifdef CONFIG_NETWORK_AUTOALIGN
  if (hdr == &aligned)
    memcpy(nethdr->data, hdr, sizeof (struct ether_arp));
#endif

  packet->tMAC = (uint8_t *)"\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_ARP);
}

/*
 * Send an ARP reply
 */

NET_ARP_REPLY(arp_reply)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)pv->ip->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  arp_preparepkt(dev, packet, 0);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    hdr = &aligned;
#endif

  /* fill the reply */
  net_be16_store(hdr->ea_hdr.ar_hrd, ARPHRD_ETHER);
  net_be16_store(hdr->ea_hdr.ar_pro, ETHERTYPE_IP);
  hdr->ea_hdr.ar_hln = ETH_ALEN;
  hdr->ea_hdr.ar_pln = 4;
  net_be16_store(hdr->ea_hdr.ar_op, ARPOP_REPLY);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  memcpy(hdr->arp_spa, pv_ip->addr, 4);
  memcpy(hdr->arp_tha, mac, ETH_ALEN);
  memcpy(hdr->arp_tpa, ip, 4);

#ifdef CONFIG_NETWORK_AUTOALIGN
  if (hdr == &aligned)
    memcpy(nethdr->data, hdr, sizeof (struct ether_arp));
#endif

  packet->tMAC = (uint8_t *)"\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_ARP);
}

/*
 * Update table entry.
 */

NET_ARP_UPDATE_TABLE(arp_update_table)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct arp_entry_s	*arp_entry;

  if ((arp_entry = arp_table_lookup(&pv->table, ip)))
    {
      if (!memcmp(arp_entry->mac, mac, ETH_ALEN))
	return ;
      arp_table_remove(&pv->table, arp_entry);
    }
  else
    arp_entry = mem_alloc(sizeof (struct arp_entry_s), MEM_SCOPE_THREAD);
  memcpy(arp_entry->mac, mac, ETH_ALEN);
  memcpy(arp_entry->ip, ip, 4);
  arp_table_push(&pv->table, arp_entry);
  printf("Added %d.%d.%d.%d as %2x:%2x:%2x:%2x:%2x:%2x\n",
	 arp_entry->ip[0], arp_entry->ip[1], arp_entry->ip[2],
	 arp_entry->ip[3], arp_entry->mac[0], arp_entry->mac[1],
	 arp_entry->mac[2], arp_entry->mac[3], arp_entry->mac[4],
	 arp_entry->mac[5]);
}

/*
 * Get the MAC address corresponding to an IP.
 *
 * Make an ARP request if needed.
 */

NET_ARP_GET_MAC(arp_get_mac)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct arp_entry_s	*arp_entry;

  if ((arp_entry = arp_table_lookup(&pv->table, ip)))
    {
      return arp_entry->mac;
    }
  else
    {
      /* XXX arp_request */
      return NULL;
    }
}

