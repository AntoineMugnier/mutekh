/*
 * ARP protocol
 *
 */

#include <netinet/arp.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <hexo/device.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct arp_interface_s	arp_interface =
{
  .request = arp_request,
  .reply = arp_reply
};

const struct net_proto_desc_s	arp_protocol =
  {
    .name = "ARP",
    .id = ETHERTYPE_ARP,
    .pushpkt = arp_push,
    .preparepkt = arp_prepare,
    .f.arp = &arp_interface,
    .pv_size = sizeof (struct net_pv_arp_s),
  };

/*
 * This function decodes an incoming ARP packet.
 */

NET_PUSHPKT(arp_push)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)protocol->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_header_s	*nethdr;

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
#if 0
	if (!memcmp(hdr->arp_tpa, pv->ip->addr, 4))
	  {
	    arp_reply(dev, protocol, hdr->arp_sha, hdr->arp_spa);
	  }
#endif
	/* no break here since we also need to refresh cache */
      case ARPOP_REPLY:
	/* XXX refresh arp cache with source */
	/* push(hdr->arp_spa, hdr->arp_sha) */
	break;
      default:
	break;
    }
}

/*
 * This function prepares an ARP packet.
 */

NET_PREPAREPKT(arp_prepare)
{
  dev_net_preparepkt(dev, packet, sizeof (struct ether_arp));
}

/*
 * This function request a MAC address given an IP address.
 */

NET_ARP_REQUEST(arp_request)
{
  struct net_pv_arp_s	*pv = (struct net_pv_arp_s*)arp->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  arp_prepare(dev, packet);

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
#if 0
  memcpy(hdr->arp_spa, pv->ip->addr, 4);
#endif
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
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  arp_prepare(dev, packet);

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
#if 0
  memcpy(hdr->arp_spa, pv->ip->addr, 4);
#endif
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
