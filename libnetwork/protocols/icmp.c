/*
 * ICMP protocol
 *
 */

#include <netinet/icmp.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <hexo/device.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct icmp_interface_s	icmp_interface =
{
  .echo = icmp_echo
};

const struct net_proto_desc_s	icmp_protocol =
  {
    .name = "ICMP",
    .id = IPPROTO_ICMP,
    .pushpkt = icmp_pushpkt,
    .preparepkt = icmp_preparepkt,
    .initproto = icmp_init,
    .f.icmp = &icmp_interface,
    .pv_size = 0
  };


NET_INITPROTO(icmp_init)
{

}

NET_PUSHPKT(icmp_pushpkt)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct icmphdr	aligned;
#endif
  struct icmphdr	*hdr;
  struct net_header_s	*nethdr;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct icmphdr*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct icmphdr));
      hdr = &aligned;
    }
#endif

  /* action */
  switch (hdr->type)
    {
      case 8:
	switch (hdr->code)
	  {
	    case 0:
	      printf("Ping request\n");
	      icmp_echo(dev, protocol, packet->sIP,
			net_be16_load(hdr->un.echo.id),
			net_be16_load(hdr->un.echo.sequence),
			nethdr->data + sizeof (struct icmphdr),
			nethdr->size - sizeof (struct icmphdr));
	      break;
	    default:
	      break;
	  }
	break;
      default:
	break;
    }
}

NET_PREPAREPKT(icmp_preparepkt)
{
  ip_preparepkt(dev, packet, sizeof (struct icmphdr) + size);
}

NET_ICMP_ECHO(icmp_echo)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct icmphdr	aligned;
#endif
  struct icmphdr	*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;

  packet = packet_obj_new(NULL);

  icmp_preparepkt(dev, packet, size);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct ether_arp*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct icmphdr));
    }
#endif

  /* fill the echo */
  hdr->type = 0;
  hdr->code = 3;
  /* XXX checksum */
  net_be16_store(hdr->un.echo.id, id);
  net_be16_store(hdr->un.echo.sequence, seq);

  /* copy data */
  memcpy(nethdr->data + sizeof (struct icmphdr),
	 data,
	 size);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(nethdr->data, hdr, sizeof (struct icmphdr));
#endif

  /* target IP */
  packet->tIP = ip;

  packet->stage--;
  /* send the packet to IP */
#if 0
  ip_send(dev, packet, icmp);
#endif
}

