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
#include <hexo/cpu.h>

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/ether.h>

#include <netinet/tcp.h>

#include <netinet/libtcp.h>

#undef net_debug
#define net_debug printf

/*
 * Functions for the interface container.
 */

CONTAINER_FUNC(static inline, net_if, HASHLIST, net_if, NOLOCK, list_entry, STRING, name);

/*
 * Session container.
 */

CONTAINER_FUNC(static inline, tcp_session, HASHLIST, tcp_session, NOLOCK, list_entry, BLOB, remote);

/*
 * TCP session list.
 */

static tcp_session_root_t	sessions = CONTAINER_ROOT_INITIALIZER(tcp_session, HASHLIST, NOLOCK);

/*
 * LibTCP user interface.
 */

/*
 * Open a new TCP connection.
 */

int_fast8_t			tcp_open(struct net_tcp_addr_s	*local,
					 struct net_tcp_addr_s	*remote,
					 tcp_connect_t		callback,
					 void			*ptr)
{
  struct net_tcp_session_s	*session;
  struct net_if_s		*interface = NULL;
  struct net_proto_s		*addressing = NULL;

  /* look for the good IP module */
  CONTAINER_FOREACH(net_if, HASHLIST, net_if, &net_interfaces,
  {
    interface = item;
    /* XXX foreach + lookup will be better */
    CONTAINER_FOREACH(net_protos, HASHLIST, net_protos, &interface->protocols,
    {
      if (item->id == ETHERTYPE_IP)
	{
	  if (item->desc->f.addressing->matchaddr(item, &local->address, NULL, NULL))
	    {
	      addressing = item;
	      break;
	    }
	}
    });
    if (addressing)
      break;
  });

  if (interface == NULL || addressing == NULL)
    return -1;

  /* create session instance */
  session = mem_alloc(sizeof (struct net_tcp_session_s), MEM_SCOPE_SYS);
  session->interface = interface;
  session->addressing = addressing;

  session->connect = callback;
  session->connect_data = ptr;
  session->receive = NULL;
  session->close = NULL;
  session->accept = NULL;

  /* choose a local port */
  local->port = 1024 + (cpu_cycle_count() % 32768) ; /* XXX choose me better! */
  memcpy(&session->local, local, sizeof (struct net_tcp_addr_s));
  memcpy(&session->remote, remote, sizeof (struct net_tcp_addr_s));

  session->send_seq = 1; /* XXX generate it */
  session->send_ack = session->send_seq + 1;
  session->send_win = TCP_DFL_WINDOW;
  session->send_mss = TCP_MSS;

  session->recv_mss = TCP_MSS; /* XXX compute me! */

  /* enter SYN sent mode, waiting for SYN ACK */
  session->state = TCP_STATE_SYN_SENT;

  /* push the new session into the hashlist */
  tcp_session_push(&sessions, session);

  /* XXX timeout here */

  /* send the SYN packet */
  tcp_send_controlpkt(session, TCP_OPEN);
  net_debug("<- SYN\n");

  return 0;
}

/*
 * Close an opened TCP session.
 */

void			tcp_close(struct net_tcp_session_s	*session)
{
  /* enter FIN WAIT state, waiting for FIN ACK */
  session->state = TCP_STATE_FIN_WAIT;
  session->recv_win = 0;

  /* just send a close request */
  tcp_send_controlpkt(session, TCP_FIN);
  session->send_seq++;
  net_debug("<- FIN\n");
}

/*
 * Setup receiving callback.
 */

void	tcp_on_receive(struct net_tcp_session_s	*session,
		       tcp_receive_t		*callback,
		       void			*ptr)
{
  session->receive = callback;
  session->receive_data = ptr;
}

void	tcp_on_close(struct net_tcp_session_s	*session,
		     tcp_close_t		*callback,
		     void			*ptr)
{

}

void	tcp_on_accept(struct net_tcp_session_s	*session,
		      tcp_accept_t		*callback,
		      void			*ptr)
{

}

/*
 * Send data using given TCP connection.
 */

void			tcp_send(struct net_tcp_session_s	*session,
				 void				*data,
				 size_t				size)
{
  net_debug("tcp_send: %P\n", data, size);

  /* XXX bufferisation */

  /* send the data packet */
  tcp_send_datapkt(session, data, size, TH_PUSH);

  /* increment the sequence number */
  session->send_seq += size;
}

/*
 * Interface with stack's TCP module.
 */

/*
 * Called on incoming connection.
 */

void				libtcp_open(struct net_packet_s	*packet,
					    struct tcphdr	*hdr)
{
  struct net_tcp_session_s	*session;
  struct net_tcp_addr_s		key;

  /* look for the corresponding session */
  memcpy(&key.address, &packet->sADDR, sizeof (struct net_addr_s));
  key.port = net_16_load(hdr->th_sport);

  if ((session = tcp_session_lookup(&sessions, (void *)&key)) == NULL)
    return ; /* no session, no one is waiting for this connection */

  if (session->state == TCP_STATE_SYN_SENT)
    {
      tcp_connect_t	*callback = session->connect;
      void		*ptr = session->connect_data;

      if ((hdr->th_flags & TH_ACK) && (net_be32_load(hdr->th_ack) == session->send_ack))
	{
	  net_debug("-> SYN ACK\n");

	  /* ok, connection aknowleged */
	  session->state = TCP_STATE_ESTABLISHED;

	  /* get the sender seq & win */
	  session->recv_seq = net_be32_load(hdr->th_seq);
	  session->recv_ack = session->recv_seq + 1;
	  session->recv_win = net_be16_load(hdr->th_win);

	  /* increment seq */
	  session->send_seq++;

	  /* get mss if present */
	  if (hdr->th_off > 5)
	    {
	      uint32_t	opt;

	      opt = net_be32_load(*(uint32_t *)(hdr + 1));
	      if (opt & (2 << 24))
		session->send_mss = opt & 0xffff;
	    }

	  net_debug("<> ESTABLISHED\n");
	  net_debug("  send MSS = %u\n", session->send_mss);
	  net_debug("  recv MSS = %u\n", session->recv_mss);

	  callback(session, ptr);
	}
      else /* otherwise, this is a bad connection sequence, report error */
	{
	  session->state = TCP_STATE_ERROR;

	  callback(session, ptr);

	  tcp_session_remove(&sessions, session);
	  mem_free(session);
	}

    }
  else if (session->state == TCP_STATE_LISTEN)
    {
      /* incoming connection request */

      /* XXX callback + create new session */
    }
}

/*
 * Called on remote connection closing.
 */

void				libtcp_close(struct net_packet_s	*packet,
					     struct tcphdr		*hdr)
{
  struct net_tcp_session_s	*session;
  struct net_tcp_addr_s		key;

  /* look for the corresponding session */
  memcpy(&key.address, &packet->sADDR, sizeof (struct net_addr_s));
  key.port = net_16_load(hdr->th_sport);

  if ((session = tcp_session_lookup(&sessions, (void *)&key)) == NULL)
    return ;

  /* error when opening */
  if (session->state == TCP_STATE_SYN_SENT)
    {
      net_debug("-> FIN (while connecting)\n");

      /* set error state */
      session->state = TCP_STATE_ERROR;

      session->connect(session, session->connect_data);

      tcp_session_remove(&sessions, session);
      mem_free(session);
    }
  else if (session->state == TCP_STATE_FIN_WAIT) /* it is a FIN acknowlegment */
    {
      net_debug("-> FIN ACK\n");

      tcp_send_controlpkt(session, TCP_ACK_DATA);
      net_debug("<- ACK\n");

      tcp_session_remove(&sessions, session);
      mem_free(session);
    }
  else /* otherwise, it is a FIN request */
    {
      net_debug("-> FIN\n");

      /* no data remaining, close the connection */
      if (1)
	{
	  tcp_send_controlpkt(session, TCP_ACK_FIN);

	  net_debug("<- FIN ACK\n");

	  if (session->close != NULL)
	    session->close(session, session->close_data);

	  tcp_session_remove(&sessions, session);
	  mem_free(session);
	}
      else /* otherwise, wait for the remaining operations and change state */
	session->state = TCP_STATE_FIN_REQ;
    }
}

/*
 * Called on packet incoming (data or control).
 */

void				libtcp_push(struct net_packet_s	*packet,
					    struct tcphdr	*hdr)
{
  struct net_tcp_session_s	*session;
  struct net_tcp_addr_s		key;
  struct net_header_s		*nethdr;

  /* look for the corresponding session */
  memcpy(&key.address, &packet->sADDR, sizeof (struct net_addr_s));
  key.port = net_16_load(hdr->th_sport);

  if ((session = tcp_session_lookup(&sessions, (void *)&key)) == NULL)
    return ;

  /* get the header */
  nethdr = &packet->header[packet->stage];

  /* XXX check for lost packet(s) */

  /* XXX update sequence numbers */

  /* control packet */
  if (nethdr->size == hdr->th_off * 4)
    {
      /* XXX nothing here ?? */
    }
  else /* data packet */
    {
      uint8_t	*data = (nethdr->data + hdr->th_off * 4);
      size_t	length = nethdr->size - hdr->th_off * 4;

      /* deliver data to application */
      if (hdr->th_flags & TH_PUSH || 1) /* XXX || buffer reception plein */
	{
	  if (session->receive != NULL)
	    session->receive(session, data, length, session->receive_data);
	}
      else /* otherwise, push the data into the receive buffer */
	{
	  /* XXX buffer */
	}
    }
}

