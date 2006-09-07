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
    .pushpkt = rarp_push,
    .preparepkt = rarp_prepare,
    .f.rarp = &rarp_interface,
    .pv_size = 0
  };

/*
 * RARP packet incoming.
 */

NET_PUSHPKT(rarp_push)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  uint8_t		my_mac[ETH_HLEN];

  /* get the header */
  hdr = (struct ether_arp*)packet->header[packet->stage];

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct ether_arp));
      hdr = &aligned;
    }
#endif

  /* check header */
  if (net_be16_load(hdr->arp_hrd) != ARPHRD_ETHER)
    {
      printf("NETWORK: bad hrd\n");
      return ;
    }

  if (net_be16_load(hdr->arp_pro) != ETHERTYPE_IP)
    {
      printf("NETWORK: bad pro\n");
      return ;
    }

  /* ARP reply message */
  if (net_be16_load(hdr->arp_op) == ARPOP_RREPLY)
    {
      /*      if (memcmp(my_mac, hdr->arp_tha; ETH_HLEN))
	{

	}*/
    }


}

/*
 * prepare a RARP packet.
 */

NET_PREPAREPKT(rarp_prepare)
{
  dev_net_preparepkt(dev, packet, sizeof (struct ether_arp));

  packet->size[packet->stage] = sizeof (struct ether_arp);
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

  packet = packet_create();

  rarp_prepare(dev, packet, protocols);

  /* get the header */
  hdr = (struct ether_arp*)packet->header[packet->stage];

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
  /* XXX sha = my MAC */
  memcpy(hdr->arp_tha, mac, hdr->arp_hln);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(packet->header[packet->stage], hdr, sizeof (struct ether_arp));
#endif

  packet->sMAC = hdr->arp_sha;
  packet->tMAC = "\xff\xff\xff\xff\xff\xff";
  packet->MAClen = ETH_ALEN;

  packet->stage--;
  dev_net_sendpkt(dev, packet, ETHERTYPE_REVARP);
}

