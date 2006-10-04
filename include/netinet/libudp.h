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

#ifndef NETINET_LIBUDP_H
#define NETINET_LIBUDP_H

#include <netinet/protos.h>
#include <netinet/packet.h>
#include <netinet/udp.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Types
 */

struct	net_udp_addr_s
{
  struct net_addr_s	address;
  uint_fast16_t		port;
};

#define UDP_CALLBACK(f)	void (f)(struct net_udp_addr_s	*local,		\
				 struct net_udp_addr_s	*remote,	\
				 void			*data,		\
				 size_t			size)
typedef UDP_CALLBACK(udp_callback_t);

CONTAINER_TYPE(udp_callback, HASHLIST, struct udp_callback_desc_s, NOLOCK, 64, BLOB, sizeof (struct net_udp_addr_s));

struct	udp_callback_desc_s
{
  struct net_udp_addr_s	address[1];
  udp_callback_t	*callback;
  udp_callback_entry_t	list_entry;
};

/*
 * Prototypes
 */

int_fast8_t	udp_send(struct net_udp_addr_s	*local,
			 struct net_udp_addr_s	*remote,
			 void			*data,
			 size_t			size);
int_fast8_t	udp_callback(struct net_udp_addr_s	*local,
			     udp_callback_t		*callback);

void		libudp_signal(struct net_packet_s	*packet,
			      struct udphdr		*hdr);

#endif
