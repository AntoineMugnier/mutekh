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

struct	net_tcp_addr_s
{
  struct net_addr_s	address;
  uint_fast16_t		port;
};

#define TCP_CONNECT(f)	void (f)(struct net_tcp_session_s *session,	\
				 void			*ptr)

typedef TCP_CONNECT(tcp_connect_t);

#define TCP_RECEIVE(f)	void (f)(struct net_tcp_session_s *session,	\
				 void			*data,		\
				 size_t			size,		\
				 void			*ptr)

typedef TCP_RECEIVE(tcp_receive_t);

#define TCP_CLOSE(f)	void (f)(struct net_tcp_session_s	*session,\
				 void				*ptr)

typedef TCP_CLOSE(tcp_close_t);

#define TCP_ACCEPT(f)	void (f)(struct net_tcp_session_s *session,	\
				 struct net_tcp_session_s *client,	\
				 void			*ptr)

typedef TCP_ACCEPT(tcp_accept_t);

int_fast8_t	tcp_open(struct net_tcp_addr_s	*local,
			 struct net_tcp_addr_s	*remote,
			 tcp_connect_t		callback,
			 void			*ptr);

void	tcp_close(struct net_tcp_session_s	*session);

void	tcp_on_receive(struct net_tcp_session_s	*session,
		       tcp_receive_t		*callback,
		       void			*ptr);

void	tcp_on_close(struct net_tcp_session_s	*session,
		     tcp_close_t		*callback,
		     void			*ptr);

void	tcp_on_accept(struct net_tcp_session_s	*session,
		      tcp_accept_t		*callback,
		      void			*ptr);

void	tcp_send(struct net_tcp_session_s	*session,
		 void				*data,
		 size_t				size);

void	libtcp_push(struct net_packet_s	*packet,
		    struct tcphdr	*hdr);

NET_SIGNAL_ERROR(libtcp_signal_error);

#endif

