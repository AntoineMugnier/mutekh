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

#ifndef NETINET_LIBTCP_H
#define NETINET_LIBTCP_H

#include <netinet/protos.h>
#include <netinet/packet.h>

struct tcphdr;
struct net_tcp_session_s;

#define TCP_EVENT_ACCEPT	1
#define TCP_EVENT_RECEIVE	2
#define TCP_EVENT_CLOSE		4

struct	net_tcp_addr_s
{
  struct net_addr_s	address;
  uint_fast16_t		port;
};

#define TCP_CALLBACK(f)	void (f)(struct net_tcp_session_s *session,	\
				 uint_fast8_t		event,		\
				 void			*data,		\
				 size_t			size)

typedef TCP_CALLBACK(tcp_callback_t);

struct net_tcp_session_s	*tcp_open(struct net_tcp_addr_s	*local,
					  struct net_tcp_addr_s	*remote);
void			tcp_close(struct net_tcp_session_s	*session);
void			tcp_callback(struct net_tcp_addr_s	*local,
				     tcp_callback_t		*callback,
				     uint_fast8_t		event);
void			tcp_send(struct net_tcp_session_s	*session,
				 void				*data,
				 size_t				size);


void		libtcp_open(struct net_packet_s	*packet,
			    struct tcphdr	*hdr);
void		libtcp_close(struct net_packet_s	*packet,
			 struct tcphdr			*hdr);
void		libtcp_push(struct net_packet_s	*packet,
			    struct tcphdr	*hdr);

#endif
