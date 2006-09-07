/*
 * Ethernet layer
 *
 */

#include <hexo/alloc.h>
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
  .build = ether_build
};

const struct net_proto_s	ether_protocol =
  {
    .name = "Ethernet",
    .id = 0,
    .pushpkt = ether_push,
    .preparepkt = ether_prepare,
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

/*
 * This function prepares an Ethernet packet.
 */

NET_PREPAREPKT(ether_prepare)
{
  uint_fast16_t		total = 0;
  uint_fast8_t		i;

  packet->size[packet->stage] = sizeof (struct ether_header) +
    packet->size[packet->stage + 1];
  for (i = packet->stage; i < NETWORK_MAX_STAGES; i++)
    total += packet->size[i];

  /* XXX about the fragmentation ? */

  packet->packet = mem_alloc(total, MEM_SCOPE_THREAD);

  packet->header[packet->stage] = packet->packet;
}

extern struct device_s	ne2000;

/*
 * This function builds an Ethernet packet.
 */

NET_ETHER_BUILD(ether_build)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_header	aligned;
#endif
  struct ether_header	*hdr;

  /* get a pointer to the header */
  hdr = (struct ether_header*)packet->header[packet->stage];

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct ether_header));
    }
#endif

  /* fill the header */
  memcpy(hdr->ether_shost, packet->sMAC, packet->MAClen);
  memcpy(hdr->ether_dhost, packet->tMAC, packet->MAClen);
  net_be16_store(hdr->ether_type, proto);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(packet->header[packet->stage], hdr, sizeof (struct ether_header));
#endif

  /* XXX send packet */

  dummy_push(dev, packet, protocols);
  dev_net_write(dev, packet->packet, packet->size[packet->stage]);
}

