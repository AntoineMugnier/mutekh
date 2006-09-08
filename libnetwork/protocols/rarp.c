/*
 * RARP protocol
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

static const struct rarp_interface_s	rarp_interface =
{
  .request = rarp_request
};

const struct net_proto_desc_s	rarp_protocol =
  {
    .name = "RARP",
    .id = ETHERTYPE_REVARP,
    .pushpkt = rarp_pushpkt,
    .preparepkt = rarp_preparepkt,
    .initproto = rarp_init,
    .f.rarp = &rarp_interface,
    .pv_size = sizeof (struct net_pv_rarp_s)
  };

/*
 * Init RARP.
 */

NET_INITPROTO(rarp_init)
{
  struct net_pv_rarp_s	*pv = (struct net_pv_rarp_s*)proto->pv;

  pv->ip = net_protos_lookup(&other, ETHERTYPE_IP);
  printf("RARP %s with IP (%p)\n", pv->ip ? "bound" : "not bound",
	 pv->ip);
}

/*
 * RARP packet incoming.
 */

NET_PUSHPKT(rarp_pushpkt)
{
  struct net_pv_rarp_s	*pv = (struct net_pv_rarp_s*)protocol->pv;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s*)pv->ip->pv;
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

  /* ARP reply message */
  if (net_be16_load(hdr->arp_op) == ARPOP_RREPLY)
    {
      if (memcmp(packet->tMAC, hdr->arp_tha, ETH_ALEN))
	return ;

      printf("Assigned IP: %d.%d.%d.%d\n", hdr->arp_tpa[0],
	     hdr->arp_tpa[1], hdr->arp_tpa[2], hdr->arp_tpa[3]);

      memcpy(pv_ip->addr, hdr->arp_tpa, 4);
    }
}

/*
 * prepare a RARP packet.
 */

NET_PREPAREPKT(rarp_preparepkt)
{
  dev_net_preparepkt(dev, packet, sizeof (struct ether_arp));
}

/*
 * make a RARP request.
 */

NET_RARP_REQUEST(rarp_request)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  rarp_preparepkt(dev, packet);

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
  net_be16_store(hdr->arp_op, ARPOP_RREQUEST);
  memcpy(hdr->arp_sha, packet->sMAC, ETH_ALEN);
  if (!mac)
    memcpy(hdr->arp_tha, packet->sMAC, ETH_ALEN);
  else
    memcpy(hdr->arp_tha, mac, ETH_ALEN);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(nethdr->data, hdr, sizeof (struct ether_arp));
#endif

  packet->tMAC = "\xff\xff\xff\xff\xff\xff";

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_REVARP);
}

