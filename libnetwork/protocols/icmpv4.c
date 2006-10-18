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

const struct net_control_interface_s	icmp_interface =
  {
    .errormsg = icmp_errormsg
  };

const struct net_proto_desc_s	icmp_protocol =
  {
    .name = "ICMP",
    .id = IPPROTO_ICMP,
    .pushpkt = icmp_pushpkt,
    .preparepkt = icmp_preparepkt,
    .initproto = NULL,
    .destroyproto = NULL,
    .f.control = &icmp_interface,
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
  addressing->desc->f.addressing->sendpkt(interface, packet, addressing, IPPROTO_ICMP);
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
  struct net_proto_s	*addressing = packet->source_addressing;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct icmphdr *)nethdr->data;

  /* align the packet on 32 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!IS_ALIGNED(hdr, sizeof (uint32_t)))
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

  /* XXX action */
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

/*
 * Report error with ICMP.
 */

NET_ERRORMSG(icmp_errormsg)
{
  struct net_proto_s	*addressing = erroneous->source_addressing;
  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)addressing->pv;
  struct net_if_s	*interface = (struct net_if_s *)pv_ip->interface;
  struct net_packet_s	*packet;
  struct icmphdr	*hdr;
  struct iphdr		*hdr_err;
  struct net_header_s	*nethdr;
  uint8_t		*dest;
  uint_fast16_t		offs;
  uint_fast16_t		size;
  va_list		va;

  va_start(va, error);

  /* get a pointer to the erroneous packet */
  nethdr = &erroneous->header[erroneous->stage];
  hdr_err = (struct iphdr *)nethdr->data;

  /* we must not generate error on ICMP packets (except ping) */
  if (hdr_err->protocol == IPPROTO_ICMP)
    {
      struct icmphdr	*hdr_icmp;

      hdr_icmp = (struct icmphdr *)nethdr[1].data;
      if (hdr_icmp->type != 0 && hdr_icmp->type != 8)
	return;
    }

  offs = hdr_err->ihl * 4;
  /* next stage */
  if (!nethdr[1].data)
    {
      nethdr[1].data = nethdr->data + offs;
      nethdr[1].size = nethdr->size - offs;
    }

  size = nethdr[1].size;
  if (size > 8)
    size = 8;

  /* prepare the packet */
  packet = packet_obj_new(NULL);
  dest = icmp_preparepkt(interface, packet, offs + size, 0);
  memcpy(&packet->tADDR, &erroneous->sADDR, sizeof (struct net_addr_s));

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct icmphdr *)nethdr->data;

  net_32_store(hdr->un.gateway, 0);

  /* fill the type and code */
  switch (error)
    {
      case ERROR_NET_UNREACHABLE:
	hdr->type = 3;
	hdr->code = 0;
	break;
      case ERROR_HOST_UNREACHABLE:
	hdr->type = 3;
	hdr->code = 1;
	break;
      case ERROR_PROTO_UNREACHABLE:
	hdr->type = 3;
	hdr->code = 2;
	break;
      case ERROR_PORT_UNREACHABLE:
	hdr->type = 3;
	hdr->code = 3;
	break;
      case ERROR_CANNOT_FRAGMENT:
	{
	  struct net_route_s	*route = va_arg(va, struct net_route_s *);

	  hdr->type = 3;
	  hdr->code = 4;
	  net_be16_store(hdr->un.frag.mtu, route->interface->mtu);
	}
	break;
      case ERROR_NET_DENIED:
	hdr->type = 3;
	hdr->code = 11;
	break;
      case ERROR_HOST_DENIED:
	hdr->type = 3;
	hdr->code = 10;
	break;
      case ERROR_CONGESTION:
	hdr->type = 4;
	hdr->code = 0;
	break;
      case ERROR_TIMEOUT:
	hdr->type = 11;
	hdr->code = 0;
	break;
      case ERROR_FRAGMENT_TIMEOUT:
	hdr->type = 11;
	hdr->code = 1;
	break;
      case ERROR_BAD_HEADER:
	hdr->type = 12;
	hdr->code = 0;
	break;
      default:
	assert(0);
	break;
    }

  /* copy the head of the packet that caused the error */
  memcpy(dest, hdr_err, offs);
  memcpy(dest + offs, erroneous->header[erroneous->stage + 1].data, size);

  net_16_store(hdr->checksum, 0);
  /* compute checksum */
  net_16_store(hdr->checksum, ~packet_checksum(nethdr->data, nethdr->size));

  packet->stage--;
  /* send the packet to the interface */
  addressing->desc->f.addressing->sendpkt(interface, packet, addressing, IPPROTO_ICMP);

  va_end(va);
}

