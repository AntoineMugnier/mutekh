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
 * Transmission Control Protocol
 */

#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <netinet/if.h>

#include <netinet/libtcp.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	tcp_protocol =
  {
    .name = "TCP",
    .id = IPPROTO_TCP,
    .pushpkt = tcp_pushpkt,
    .preparepkt = NULL,
    .initproto = NULL,
    .pv_size = 0
  };

/*
 * Receive incoming TCP packets.
 */

NET_PUSHPKT(tcp_pushpkt)
{
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct tcphdr		aligned;
#endif
  struct tcphdr		*hdr;
  struct net_header_s	*nethdr;
  uint_fast8_t		flags;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct tcphdr *)nethdr->data;

  /* align the packet on 32 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint32_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct tcphdr));
      hdr = &aligned;
    }
#endif

  flags = hdr->th_flags;

  /* connection opening */
  if (flags & TH_SYN)
    {
      libtcp_open(packet, hdr);
    }
  else if (flags & TH_FIN) /* connection closing */
    {
      libtcp_close(packet, hdr);
    }
  else if (flags & TH_RST) /* reset connection due to unrecoverable error(s) */
    {
      /* XXX */
    }
  else /* data packet */
    {
      libtcp_push(packet, hdr);
    }
}

/*
 * Prepare a TCP packet.
 */

uint8_t			*tcp_preparepkt(struct net_if_s		*interface,
					struct net_proto_s	*addressing,
					struct net_packet_s	*packet,
					size_t			size,
					size_t			max_padding)
{
  struct net_header_s	*nethdr;
  uint8_t		*next;

#ifdef CONFIG_NETWORK_AUTOALIGN
  next = addressing->desc->preparepkt(interface, packet, sizeof (struct tcphdr) + size, 2);
  next = ALIGN_ADDRESS(next, 4);
#else
  next = addressing->desc->preparepkt(interface, packet, sizeof (struct tcphdr) + size, 0);
#endif

  nethdr = &packet->header[packet->stage];
  nethdr->data = next;
  nethdr->size = sizeof (struct tcphdr) + size;

  nethdr[1].data = NULL;

  return next + sizeof (struct tcphdr);
}

/*
 * Send a TCP control packet (without data).
 */

void	tcp_send_controlpkt(struct net_tcp_session_s	*session,
			    uint_fast8_t		operation)
{
  struct net_packet_s	*packet;
  struct net_proto_s	*addressing = session->addressing;
  struct net_if_s	*interface = session->interface;
  struct net_header_s	*nethdr;
  struct tcphdr		*hdr;

  packet = packet_obj_new(NULL);

  /* prepare the packet */
  tcp_preparepkt(interface, addressing, packet, 0, 0);
  nethdr = &packet->header[packet->stage];
  hdr = (struct tcphdr *)nethdr->data;

  /* fill the packet header */
  switch (operation)
    {
      case TCP_OPEN:
	/* XXX */
	break;
      case TCP_ACK_OPEN:
	/* XXX */
	break;
      case TCP_ACK_DATA:
	/* XXX */
	break;
      case TCP_CLOSE:
	/* XXX */
	break;
      case TCP_ACK_CLOSE:
	/* XXX */
	break;
      default:
	assert(0);
	break;
    }

  packet->stage--;
  /* send the packet to IP */
  addressing->desc->f.addressing->sendpkt(interface, packet, addressing, IPPROTO_TCP);
}

/*
 * Send a TCP data packet.
 */

void	tcp_send_datapkt(struct net_tcp_session_s	*session,
			 void				*data,
			 size_t				size,
			 uint_fast8_t			flags)
{
  struct net_packet_s	*packet;
  struct net_proto_s	*addressing = session->addressing;
  struct net_if_s	*interface = session->interface;
  uint8_t		*dest;
  struct net_header_s	*nethdr;
  struct tcphdr		*hdr;

  packet = packet_obj_new(NULL);

  /* prepare the packet */
  dest = tcp_preparepkt(interface, addressing, packet, 0, 0);
  nethdr = &packet->header[packet->stage];
  hdr = (struct tcphdr *)nethdr->data;

  /* fill the packet header */
  /* XXX */

  /* copy the data */
  /* XXX */

  /* update the session */
  /* XXX */

  packet->stage--;
  /* send the packet to IP */
  addressing->desc->f.addressing->sendpkt(interface, packet, addressing, IPPROTO_TCP);
}

