/*
 * ARP protocol
 *
 */

#include <netinet/arp.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct arp_interface_s	arp_interface =
{
  .request = arp_request
};

const struct net_proto_s	arp_protocol =
  {
    .name = "ARP",
    .id = ETHERTYPE_ARP,
    .pushpkt = arp_push,
    .preparepkt = arp_prepare,
    .f.arp = &arp_interface
  };

static const struct rarp_interface_s	rarp_interface =
{
  .request = rarp_request
};

const struct net_proto_s	rarp_protocol =
  {
    .name = "RARP",
    .id = ETHERTYPE_REVARP,
    .pushpkt = rarp_push,
    .preparepkt = rarp_prepare,
    .f.rarp = &rarp_interface
  };

/*
 * This function decodes an incoming ARP packet.
 */

NET_PUSHPKT(arp_push)
{

}

/*
 * This function prepares an ARP packet.
 */

NET_PREPAREPKT(arp_prepare)
{
  packet->size[packet->stage] = sizeof (struct ether_arp);
  packet->stage--;

  ether_prepare(dev, packet, protocols);

  packet->stage++;

  /* this field is already valid if fragmentation is used */
  if (!packet->header[packet->stage])
    packet->header[packet->stage] = packet->header[packet->stage - 1] +
      packet->size[packet->stage - 1] - packet->size[packet->stage];
}

/*
 * This function request a MAC address given an IP address.
 */

NET_ARP_REQUEST(arp_request)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_arp	aligned;
#endif
  struct ether_arp	*hdr;
  struct net_packet_s	*packet;
  net_protos_root_t	protocols;
  struct device_s	*dev;

  packet = packet_create();

  /* XXX protocols = ? */
  /* XXX dev = ? */

  arp_prepare(dev, packet, protocols);

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
  net_be16_store(hdr->arp_op, ARPOP_REQUEST);
  /* XXX sha = my MAC addr */
  /* XXX spa = my IP addr */
  memcpy(hdr->arp_tha, "\xff\xff\xff\xff\xff\xff", hdr->arp_hln);
  memcpy(hdr->arp_tpa, address, hdr->arp_pln);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(packet->header[packet->stage], hdr, sizeof (struct ether_arp));
#endif

  packet->sMAC = hdr->arp_sha;
  packet->tMAC = hdr->arp_tha;

  packet->stage--;
  ether_build(dev, packet, protocols, ETHERTYPE_ARP);
}

NET_PUSHPKT(rarp_push)
{
}

NET_PREPAREPKT(rarp_prepare)
{
}

NET_RARP_REQUEST(rarp_request)
{
}

