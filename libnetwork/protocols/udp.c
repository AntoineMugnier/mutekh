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
 * UDP protocol
 *
 */

#include <netinet/udp.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <netinet/if.h>

#include <netinet/libudp.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	udp_protocol =
  {
    .name = "UDP",
    .id = IPPROTO_UDP,
    .pushpkt = udp_pushpkt,
    .preparepkt = NULL,
    .initproto = NULL,
    .pv_size = 0
  };

/*
 * Receive incoming UDP datagrams.
 */

NET_PUSHPKT(udp_pushpkt)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct udphdr		aligned;
#endif
  struct udphdr		*hdr;
  struct net_header_s	*nethdr;
  uint32_t		check;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct udphdr *)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct udphdr));
      hdr = &aligned;
    }
#endif

  /* XXX */
  check = addressing->desc->f.addressing->pseudoheader_checksum(packet, IPPROTO_UDP, net_16_load(hdr->len));
  check += packet_checksum(nethdr->data, net_be16_load(hdr->len));

  /* next stage */
  if (!nethdr[1].data)
    {
      nethdr[1].data = nethdr->data + sizeof (struct udphdr);
      nethdr[1].size = net_be16_load(hdr->len) - sizeof (struct udphdr);
    }
  packet->stage++;

  udp_signal(packet, hdr);
}

/*
 * Prepare UDP packet.
 */

uint8_t			*udp_preparepkt(struct net_if_s		*interface,
					struct net_proto_s	*addressing,
					struct net_packet_s	*packet,
					size_t			size,
					size_t			max_padding)
{
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = addressing->desc->preparepkt(interface, packet, sizeof (struct udphdr) + size, 2);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = addressing->desc->preparepkt(interface, packet, sizeof (struct udphdr) + size, 0);
#endif

  nethdr = &packet->header[packet->stage];
  nethdr->data = next;
  nethdr->size = sizeof (struct udphdr) + size;

  nethdr[1].data = NULL;

  return next + sizeof (struct udphdr);
}

/*
 * Send a packet.
 */

void		udp_sendpkt(struct net_if_s	*interface,
			    struct net_proto_s	*addressing,
			    struct net_packet_s	*packet,
			    uint_fast16_t	source_port,
			    uint_fast16_t	dest_port)
{
  struct udphdr		*hdr;
  struct net_header_s	*nethdr;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct udphdr *)nethdr->data;

  /* fill the header */
  hdr->source = source_port;
  hdr->dest = dest_port;
  net_be16_store(hdr->len, nethdr->size);

  /* XXX compute checksum */
  hdr->check = 0;

  packet->stage--;
  /* send the packet to IP */
  addressing->desc->f.addressing->sendpkt(interface, packet, addressing, IPPROTO_UDP);
}

