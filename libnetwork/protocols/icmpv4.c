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
 * ICMP protocol for IPv4
 *
 */

#include <netinet/icmp.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <netinet/if.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	icmp_protocol =
  {
    .name = "ICMP",
    .id = IPPROTO_ICMP,
    .pushpkt = icmp_pushpkt,
    .preparepkt = icmp_preparepkt,
    .initproto = NULL,
    .pv_size = 0
  };

/*
 * Reply an echo request.
 */

static inline void	icmp_echo(struct net_if_s	*interface,
				  struct net_proto_s	*addressing,
				  struct net_proto_s	*icmp,
				  struct net_packet_s	*packet)
{
  struct icmphdr	*hdr;
  struct net_header_s	*nethdr;
  uint_fast32_t		xchg;

  packet_obj_refnew(packet);

  net_debug("Pong\n");

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct icmphdr *)nethdr->data;

  nethdr[1].data = NULL;

  /* fill the echo */
  hdr->type = 0;
  hdr->code = 3;
  net_16_store(hdr->checksum, 0);

  /* compute checksum */
  net_16_store(hdr->checksum, ~packet_checksum(nethdr->data, nethdr->size));

  /* target IP */
  xchg = packet->tADDR.addr.ipv4;
  packet->tADDR.addr.ipv4 = packet->sADDR.addr.ipv4;
  packet->sADDR.addr.ipv4 = xchg;

  packet->stage--;
  /* send the packet to IP */
  ip_send(interface, packet, addressing, IPPROTO_ICMP);
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
  uint_fast16_t		computed_check;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct icmphdr *)nethdr->data;

  /* align the packet on 32 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint32_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct icmphdr));
      hdr = &aligned;
    }
#endif

  /* verify checksum */
  computed_check = packet_checksum(nethdr->data, nethdr->size);

  /* incorrect packet */
  if (computed_check != 0xffff)
    {
      net_debug("ICMP: Rejected incorrect packet %x\n", computed_check);
      return;
    }

  /* action */
  switch (hdr->type)
    {
      case 8:
	switch (hdr->code)
	  {
	    case 0:
	      net_debug("Ping\n");
	      icmp_echo(interface, addressing, protocol, packet);
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
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = ip_preparepkt(interface, packet, sizeof (struct icmphdr) + size, 4);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = ip_preparepkt(interface, packet, sizeof (struct icmphdr) + size, 0);
#endif

  nethdr = &packet->header[packet->stage];
  nethdr->data = next;
  nethdr->size = sizeof (struct icmphdr) + size;

  nethdr[1].data = NULL;

  return next + sizeof (struct icmphdr);
}
