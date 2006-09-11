/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

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
    .pv_size = sizeof (struct net_pv_icmp_s)
  };

/*
 * Initialize ICMP module.
 */

NET_INITPROTO(icmp_init)
{
  struct net_pv_icmp_s	*pv = (struct net_pv_icmp_s *)proto->pv;
  struct net_proto_s	*ip = va_arg(va, struct net_proto_s *);

  pv->ip = ip;
  printf("ICMP %s with IP (%p)\n", pv->ip ? "bound" : "not bound", pv->ip);
}

/*
 * Receive incoming ICMP packets.
 */

NET_PUSHPKT(icmp_pushpkt)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct icmphdr	aligned;
#endif
  struct icmphdr	*hdr;
  struct net_header_s	*nethdr;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct icmphdr *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
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

/*
 * Prepare ICMP part of a packet.
 */

NET_PREPAREPKT(icmp_preparepkt)
{
  ip_preparepkt(dev, packet, sizeof (struct icmphdr) + size);
}

/*
 * Reply an echo request.
 */

NET_ICMP_ECHO(icmp_echo)
{
  struct net_pv_icmp_s	*pv = (struct net_pv_icmp_s *)icmp->pv;
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
  hdr = (struct icmphdr *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct icmphdr));
    }
#endif

  /* fill the echo */
  hdr->type = 0;
  hdr->code = 3;
  net_be16_store(hdr->un.echo.id, id);
  net_be16_store(hdr->un.echo.sequence, seq);

  /* copy data */
  memcpy(nethdr->data + sizeof (struct icmphdr),
	 data,
	 size);

#ifdef CONFIG_NETWORK_AUTOALIGN
  if (hdr == &aligned)
    memcpy(nethdr->data, hdr, sizeof (struct icmphdr));
  hdr = (struct icmphdr *)nethdr->data;
#endif

  /* compute checksum */
  endian_16_na_store(&hdr->checksum, packet_checksum(nethdr->data, nethdr->size));

  /* target IP */
  packet->tIP = ip;

  packet->stage--;
  /* send the packet to IP */
  ip_send(dev, packet, pv->ip, icmp);
}

