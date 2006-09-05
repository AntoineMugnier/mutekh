/*
 * Ethernet layer
 *
 */

#include <netinet/ether.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/in.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct ether_interface_s	ether_interface =
{
  /* XXX */
};

const struct net_proto_s	ether_protocol_eth =
  {
    .name = "Ethernet",
    .id = 0,
    .pushpkt = ether_push,
    .f.ether = &ether_interface
  };

/*
 * This function is called when an interface receives a packet.
 */

NET_PUSHPKT(ether_push)
{
  struct ether_header	*hdr;
  uint_fast16_t		proto;

  /* get the good header */
  hdr = (struct ether_header*)packet->header[packet->stage];

  /* fill some info */
#if 0
  packet->sMAC = ether->ether_shost;
  packet->tMAC = ether->ether_dhost;
#endif

  /* prepare packet for net stage */
  /* XXX */
  packet->stage++;

  /* dispatch to the matching protocol */
  proto = ntohs(hdr->ether_type);
  CONTAINER_FOREACH(net_protos, HASHLIST, net_protos, &protocols,
  {
    struct net_proto_s	*p;

    p = net_protos_get(&protocols, item);
    if (p->id == proto)
      {
	p->pushpkt(dev, packet, protocols);

	return ;
      }
  });
}

