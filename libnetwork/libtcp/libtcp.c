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
 * User interface to TCP transport layer.
 */

#include <hexo/types.h>
#include <hexo/alloc.h>

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/tcp.h>

#include <netinet/libtcp.h>

/*
 * LibTCP user interface.
 */

/*
 * Open a new TCP connection.
 */

struct net_tcp_session_s	*tcp_open(struct net_tcp_addr_s	*local,
					  struct net_tcp_addr_s	*remote)
{
  /* XXX */
  return NULL;
}

/*
 * Close an opened TCP session.
 */

void			tcp_close(struct net_tcp_session_s	*session)
{
  /* XXX */
}

/*
 * Register a callback for data reception on a given connection.
 */

void			tcp_callback(struct net_tcp_addr_s	*local,
				     tcp_callback_t		*callback,
				     uint_fast8_t		event)
{
  /* XXX */
}

/*
 * Send data using given TCP connection.
 */

void			tcp_send(struct net_tcp_session_s	*session,
				 void				*data,
				 size_t				size)
{
  /* XXX */
}

/*
 * Interface with stack's TCP module.
 */

/*
 * Called incoming connection.
 */

void		libtcp_open(struct net_packet_s	*packet,
			    struct tcphdr	*hdr)
{
  /* XXX */
}

/*
 * Called on remote connection closing.
 */

void		libtcp_close(struct net_packet_s	*packet,
			     struct tcphdr		*hdr)
{
  /* XXX */
}

/*
 * Called on packet incoming (data or control).
 */

void		libtcp_push(struct net_packet_s	*packet,
			    struct tcphdr	*hdr)
{
  /* XXX */
}

