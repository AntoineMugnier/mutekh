/*
 * ARP protocol
 *
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

CONTAINER_FUNC(static inline, arp_table, HASHLIST, arp_table, NOLOCK, list_entry, BLOB, ip, 4);

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
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)proto->pv;
  struct net_pv_ip_s	*pv_ip;

  arp_table_init(&pv->table);
  pv->ip = net_protos_lookup(&other, ETHERTYPE_IP);
  printf("ARP %s with IP (%p)\n", pv->ip ? "bound" : "not bound", pv->ip);
  pv_ip = (struct net_pv_ip_s*)pv->ip->pv;
  pv_ip->arp = proto;
}

/*
 * This function decodes an incoming ARP packet.
 */

NET_PUSHPKT(arp_pushpkt)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)protocol->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s*)pv->ip->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_header_s	*nethdr;
  struct arp_entry_s	*arp_entry;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct ether_arp));
      hdr = &aligned;
    }
#endif

  /* check header */
  if (net_be16_load(hdr->arp_hrd) != ARPHRD_ETHER ||
      net_be16_load(hdr->arp_pro) != ETHERTYPE_IP)
    return ;

  /* ARP message */
  switch (net_be16_load(hdr->arp_op))
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
	if ((arp_entry = arp_table_lookup(&pv->table, hdr->arp_spa)))
	  {
	    if (!memcmp(arp_entry->mac, hdr->arp_sha, ETH_ALEN))
	      break;
	    arp_table_remove(&pv->table, arp_entry);
	  }
	else
	  arp_entry = mem_alloc(sizeof (struct arp_entry_s), MEM_SCOPE_THREAD);
	memcpy(arp_entry->mac, hdr->arp_sha, ETH_ALEN);
	memcpy(arp_entry->ip, hdr->arp_spa, 4);
	arp_table_push(&pv->table, arp_entry);
	printf("Added %d.%d.%d.%d as %2x:%2x:%2x:%2x:%2x:%2x\n",
	       arp_entry->ip[0], arp_entry->ip[1], arp_entry->ip[2],
	       arp_entry->ip[3], arp_entry->mac[0], arp_entry->mac[1],
	       arp_entry->mac[2], arp_entry->mac[3], arp_entry->mac[4],
	       arp_entry->mac[5]);
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
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s*)pv->ip->pv;
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
  hdr = (struct ether_arp*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct ether_arp));
    }
#endif

  /* fill the request */
  net_be16_store(hdr->arp_hrd, ARPHRD_ETHER);
  net_be16_store(hdr->arp_pro, ETHERTYPE_IP);
  hdr->arp_hln = ETH_ALEN;
  hdr->arp_pln = 4;
  net_be16_store(hdr->arp_op, ARPOP_REQUEST);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  memcpy(hdr->arp_spa, pv_ip->addr, 4);
  memcpy(hdr->arp_tpa, address, hdr->arp_pln);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(nethdr->data, hdr, sizeof (struct ether_arp));
#endif

  packet->tMAC = (uint8_t*)"\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_ARP);
}

/*
 * ARP reply
 */

NET_ARP_REPLY(arp_reply)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)arp->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s*)pv->ip->pv;
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
  hdr = (struct ether_arp*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct ether_arp));
    }
#endif

  /* fill the reply */
  net_be16_store(hdr->arp_hrd, ARPHRD_ETHER);
  net_be16_store(hdr->arp_pro, ETHERTYPE_IP);
  hdr->arp_hln = ETH_ALEN;
  hdr->arp_pln = 4;
  net_be16_store(hdr->arp_op, ARPOP_REPLY);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  memcpy(hdr->arp_spa, pv_ip->addr, 4);
  memcpy(hdr->arp_tha, mac, ETH_ALEN);
  memcpy(hdr->arp_tpa, ip, hdr->arp_pln);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(nethdr->data, hdr, sizeof (struct ether_arp));
#endif

  packet->tMAC = (uint8_t*)"\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_ARP);
}

/*
 * Update table entry.
 */

NET_ARP_UPDATE_TABLE(arp_update_table)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)arp->pv;
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
 * get the MAC address corresponding to an IP.
 *
 * make an ARP request if needed.
 */

NET_ARP_GET_MAC(arp_get_mac)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)arp->pv;
  struct arp_entry_s	*arp_entry;

  if ((arp_entry = arp_table_lookup(&pv->table, ip)))
    {
      return arp_entry->mac;
    }
  else
    {
      /* XXX arp_request */
    }
}

