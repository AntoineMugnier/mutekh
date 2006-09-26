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

static const struct udp_interface_s	udp_interface =
{
  .send = udp_sendpkt
};

const struct net_proto_desc_s	udp_protocol =
  {
    .name = "UDP",
    .id = IPPROTO_UDP,
    .pushpkt = udp_pushpkt,
    .preparepkt = udp_preparepkt,
    .initproto = udp_init,
    .f.udp = &udp_interface,
    .pv_size = sizeof (struct net_pv_udp_s)
  };

/*
 * Initialize UDP module.
 */

NET_INITPROTO(udp_init)
{
  struct net_pv_udp_s	*pv = (struct net_pv_udp_s *)proto->pv;
  struct net_proto_s	*ip = va_arg(va, struct net_proto_s *);

  pv->ip = ip;
}

/*
 * Receive incoming UDP datagrams.
 */

NET_PUSHPKT(udp_pushpkt)
{
  struct net_pv_udp_s	*pv = (struct net_pv_udp_s *)protocol->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct udphdr		aligned;
#endif
  struct udphdr		*hdr;
  struct net_header_s	*nethdr;

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

  /* XXX udp checksum */

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

NET_PREPAREPKT(udp_preparepkt)
{
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = ip_preparepkt(interface, packet, sizeof (struct udphdr) + size, 2);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = ip_preparepkt(interface, packet, sizeof (struct udphdr) + size, 0);
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

NET_UDP_SEND(udp_sendpkt)
{
  struct net_pv_udp_s	*pv = (struct net_pv_udp_s *)udp->pv;
  struct udphdr		*hdr;
  struct net_packet_s	*packet;
  struct net_header_s	*nethdr;
  uint8_t		*dest;

  packet = packet_obj_new(NULL);

  dest = udp_preparepkt(interface, packet, size, 0);

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct udphdr *)nethdr->data;

  /* XXX fill the header */

  /* copy data */
  memcpy(dest, data, size);

  /* XXX compute checksum */

  /* target IP */
  IPV4_ADDR_SET(packet->tADDR, ip);

  packet->stage--;
  /* send the packet to IP */
  ip_send(interface, packet, pv->ip, udp);
}

