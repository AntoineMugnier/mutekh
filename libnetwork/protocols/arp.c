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
  /* XXX */
};

const struct ether_proto_s	arp_protocol_eth =
  {
    .name = "ARP",
    .id = ETHERTYPE_ARP,
    .pushpkt = arp_push,
    .f.dummy = &arp_interface
  };

static const struct rarp_interface_s	rarp_interface =
{
  /* XXX */
};

const struct ether_proto_s	rarp_protocol_eth =
  {
    .name = "RARP",
    .id = ETHERTYPE_REVARP,
    .pushpkt = rarp_push,
    .f.dummy = &rarp_interface
  };

ETH_PUSH(arp_push)
{

}

ETH_PUSH(rarp_push)
{

}

