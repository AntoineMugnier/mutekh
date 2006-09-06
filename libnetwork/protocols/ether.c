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

const struct net_proto_s	ether_protocol =
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
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_header	aligned;
#endif
  struct ether_header	*hdr;
  uint_fast16_t		proto;

  /* get the good header */
  hdr = (struct ether_header*)packet->header[packet->stage];

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct ether_header));
      hdr = &aligned;
    }
#endif

  /* fill some info */
  packet->MAClen = sizeof(struct ether_addr);
  packet->sMAC = hdr->ether_shost;
  packet->tMAC = hdr->ether_dhost;

  /* prepare packet for net stage */
  packet->stage++;
  if (!packet->header[packet->stage])
    {
      packet->header[packet->stage] = packet->packet +
	sizeof(struct ether_header);
      packet->size[packet->stage] = packet->size[packet->stage - 1] -
	sizeof(struct ether_header);
    }

  /* dispatch to the matching protocol */
  proto = net_be16_load(hdr->ether_type);
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

  printf("NETWORK: no protocol to handle packet (id = 0x%x)\n", proto);
}

