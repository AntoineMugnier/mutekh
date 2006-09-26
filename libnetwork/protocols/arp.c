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
 * ARP for IPv4
 *
 */

#include <netinet/arp.h>
#include <netinet/ip.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <netinet/if.h>

#include <stdio.h>

/*
 * ARP table functions.
 */

CONTAINER_FUNC(static inline, arp_table, HASHLIST, arp_table, NOLOCK, list_entry, UNSIGNED, ip);

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	arp_protocol =
  {
    .name = "ARP",
    .id = ETHERTYPE_ARP,
    .pushpkt = arp_pushpkt,
    .preparepkt = arp_preparepkt,
    .initproto = arp_init,
    .f.other = NULL,
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
  struct arp_entry_s	*arp_entry;
  struct net_packet_s	*waiting;

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

  /* ARP message */
  switch (net_be16_load(hdr->ea_hdr.ar_op))
    {
      case ARPOP_REQUEST:
	net_debug("ARP Req %P / %P\n", &hdr->arp_tpa, 4, &pv_ip->addr, 4);

	if (net_be32_load(hdr->arp_tpa) == pv_ip->addr)
	  {
	    net_debug("Me!\n");
	    /* force adding the entry */
	    arp_update_table(protocol, net_be32_load(hdr->arp_spa),
			     hdr->arp_sha, ARP_TABLE_DEFAULT);
	    arp_reply(interface, protocol, packet);
	  }
	else
	  {
	    /* try to update the cache */
	    arp_update_table(protocol, net_be32_load(hdr->arp_spa),
			     hdr->arp_sha, ARP_TABLE_NO_UPDATE);
	  }
	break;
      case ARPOP_REPLY:
	/* try to update the cache */
	arp_entry = arp_update_table(protocol, net_be32_load(hdr->arp_spa),
				     hdr->arp_sha, ARP_TABLE_DEFAULT);
	/* send waiting packets */
	while ((waiting = packet_queue_pop(&arp_entry->wait)))
	  {
	    waiting->tMAC = arp_entry->mac;

	    /* send the packet */
	    if_sendpkt(interface, waiting, pv->ip);
	  }
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
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = if_preparepkt(interface, packet, sizeof (struct ether_arp), 4);
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
 * This function request a MAC address given an IP address.
 */

void			arp_request(struct net_if_s	*interface,
				    struct net_proto_s	*arp,
				    uint_fast32_t	address)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)pv->ip->pv;
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  arp_preparepkt(interface, packet, 0, 0);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* fill the request */
  net_be16_store(hdr->ea_hdr.ar_hrd, ARPHRD_ETHER);
  net_be16_store(hdr->ea_hdr.ar_pro, ETHERTYPE_IP);
  hdr->ea_hdr.ar_hln = ETH_ALEN;
  hdr->ea_hdr.ar_pln = 4;
  net_be16_store(hdr->ea_hdr.ar_op, ARPOP_REQUEST);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  net_be32_store(hdr->arp_spa, pv_ip->addr);
  memset(hdr->arp_tha, 0xff, ETH_ALEN);
  net_be32_store(hdr->arp_tpa, address);

  packet->tMAC = hdr->arp_tha;

  packet->stage--;
  /* send the packet to the interface */
  if_sendpkt(interface, packet, arp);
}

/*
 * Send an ARP reply
 */

void			arp_reply(struct net_if_s		*interface,
				  struct net_proto_s		*arp,
				  struct net_packet_s		*packet)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)pv->ip->pv;
  struct ether_arp	*hdr;
  struct net_header_s	*nethdr;

  packet_obj_refnew(packet);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp *)nethdr->data;

  /* fill the reply */
  net_be16_store(hdr->ea_hdr.ar_op, ARPOP_REPLY);

  memcpy(hdr->arp_tha, hdr->arp_sha, ETH_ALEN);
  net_32_store(hdr->arp_tpa, net_32_load(hdr->arp_spa));
  memcpy(hdr->arp_sha, interface->mac, ETH_ALEN);
  net_be32_store(hdr->arp_spa, pv_ip->addr);

  packet->sMAC = hdr->arp_sha;
  packet->tMAC = hdr->arp_tha;

  packet->stage--;
  /* send the packet to the interface */
  if_sendpkt(interface, packet, arp);
}

/*
 * Update table entry.
 */

struct arp_entry_s	*arp_update_table(struct net_proto_s	*arp,
					  uint_fast32_t		ip,
					  uint8_t		*mac,
					  uint_fast8_t		flags)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct arp_entry_s	*arp_entry;

  if ((arp_entry = arp_table_lookup(&pv->table, ip)))
    {
      /* there's already an entry */
      if ((flags & ARP_TABLE_NO_OVERWRITE) ||
	  (arp_entry->valid && !memcmp(arp_entry->mac, mac, ETH_ALEN)))
	return arp_entry;
      /* if we must not update */
      if (flags & ARP_TABLE_NO_UPDATE)
	return NULL;
      /* we are able to update it */
    }
  else
    {
      /* otherwise, allocate a new entry */
      arp_entry = mem_alloc(sizeof (struct arp_entry_s), MEM_SCOPE_CONTEXT);
      arp_entry->ip = ip;
      arp_table_push(&pv->table, arp_entry);
    }
  /* fill the significant fields */
  if (!(flags & ARP_TABLE_IN_PROGRESS))
    memcpy(arp_entry->mac, mac, ETH_ALEN);
  arp_entry->valid = !(flags & ARP_TABLE_IN_PROGRESS);
  if (arp_entry->valid)
    net_debug("Added entry %P for %P\n", mac, ETH_ALEN, &ip, 4);
  return arp_entry;
}

/*
 * Get the MAC address corresponding to an IP.
 *
 * Make an ARP request if needed.
 */

uint8_t			*arp_get_mac(struct net_if_s		*interface,
				     struct net_proto_s		*arp,
				     struct net_packet_s	*packet,
				     uint_fast32_t		ip)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s *)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)pv->ip->pv;
  struct arp_entry_s	*arp_entry;

  if (ip == pv_ip->addr)
    {
      return interface->mac;
    }
  if ((arp_entry = arp_table_lookup(&pv->table, ip)))
    {
      /* is the entry valid ? */
      if (arp_entry->valid)
	return arp_entry->mac;
      /* otherwise, it is validating, so push the packet in the wait queue */
      packet_queue_pushback(&arp_entry->wait, packet);
    }
  else
    {
      arp_entry = arp_update_table(arp, ip, NULL, ARP_TABLE_IN_PROGRESS);
      /* no entry, push the packet in the wait queue*/
      packet_queue_init(&arp_entry->wait);
      packet_queue_push(&arp_entry->wait, packet);
      /* and send a request */
      arp_request(interface, arp, ip);
    }
  return NULL;
}

