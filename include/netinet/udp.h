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

#ifndef NETINET_UDP_H_
#define NETINET_UDP_H_

#include <hexo/types.h>

/* The UDP packet header */
struct		udphdr
{
  uint16_t	source;
  uint16_t	dest;
  uint16_t	len;
  uint16_t	check;
} __attribute__ ((packed));


#include <netinet/packet.h>
#include <netinet/protos.h>

#define NET_UDP_SEND(f)	void (f)(struct device_s	*dev,	\
				 struct net_proto_s	*udp,	\
				 uint8_t		*ip,	\
				 uint_fast16_t		port,	\
				 uint8_t		*data,	\
				 size_t			size)

typedef NET_UDP_SEND(net_udp_send_t);

/*
 * UDP protocol interface.
 */

struct			udp_interface_s
{
  net_udp_send_t	*send;
};

/*
 * UDP private data.
 */

struct			net_pv_udp_s
{
  struct net_proto_s	*ip;
};

/*
 * UDP functions
 */

NET_INITPROTO(udp_init);
NET_PUSHPKT(udp_pushpkt);
NET_PREPAREPKT(udp_preparepkt);
NET_UDP_SEND(udp_send);

extern const struct net_proto_desc_s	udp_protocol;

#endif

