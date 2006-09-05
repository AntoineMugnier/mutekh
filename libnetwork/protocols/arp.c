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

const struct net_proto_s	arp_protocol_eth =
  {
    .name = "ARP",
    .id = ETHERTYPE_ARP,
    .pushpkt = arp_push,
    .f.arp = &arp_interface
  };

static const struct rarp_interface_s	rarp_interface =
{
  /* XXX */
};

const struct net_proto_s	rarp_protocol_eth =
  {
    .name = "RARP",
    .id = ETHERTYPE_REVARP,
    .pushpkt = rarp_push,
    .f.rarp = &rarp_interface
  };

NET_PUSHPKT(arp_push)
{

}

NET_PUSHPKT(rarp_push)
{

}

