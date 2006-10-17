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
 *
 * TODO
 *  - Nagle's (bufferisation)
 *  - Window + Clark's (evitement du SWS)
 *  - Karn's & RTT (retransmission)
 *  - Fast retransmit/Fast recovery
 *  - Van Jacobson's (congestion window)
 *  - Connexion rompues & RST
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
 * Session container.
 */

CONTAINER_FUNC(static inline, tcp_session, HASHLIST, tcp_session, NOLOCK, remote);
CONTAINER_KEY_FUNC(static inline, tcp_session, HASHLIST, tcp_session, NOLOCK, remote);

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
  CONTAINER_FOREACH(net_if, HASHLIST, NOLOCK, &net_interfaces,
  {
    interface = item;
    /* XXX foreach + lookup will be better */
    CONTAINER_FOREACH(net_protos, HASHLIST, NOLOCK, &interface->protocols,
    {
      if (item->id == ETHERTYPE_IP)
	{
	  if (item->desc->f.addressing->matchaddr(item, &local->address, NULL, NULL))
	    {
	      addressing = item;
	      goto ok;
	    }
	}
    });
  });

 ok:

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

  session->curr_seq = 1; /* XXX generate it */
  session->send_win = TCP_DFL_WINDOW;
  session->send_mss = TCP_MSS;

  session->recv_mss = interface->mtu - TCP_HEADERS_LEN;

  /* enter SYN sent mode, waiting for SYN ACK */
  session->state = TCP_STATE_SYN_SENT;

  /* push the new session into the hashlist */
  tcp_session_push(&sessions, session);

  /* XXX timeout here */

  /* send the SYN packet */
  tcp_send_controlpkt(session, TCP_SYN);
  net_debug("<- SYN\n");

  return 0;
}

/*
 * Close an opened TCP session.
 */

void			tcp_close(struct net_tcp_session_s	*session)
{
  if (session->state == TCP_STATE_ESTABLISHED || session->state == TCP_STATE_SYN_RCVD)
    {
      /* enter FIN WAIT-1 state, waiting for FIN ACK */
      session->state = TCP_STATE_FIN_WAIT1;

      /* just send a close request */
      tcp_send_controlpkt(session, TCP_FIN);
      session->curr_seq++;
      net_debug("<- FIN\n");
    }
  else if (session->state == TCP_STATE_CLOSE_WAIT)
    {
      /* goto LAST ACK state */
      session->state = TCP_STATE_LAST_ACK;

      /* just send a FIN */
      tcp_send_controlpkt(session, TCP_FIN);
      session->curr_seq++;
      net_debug("<- FIN\n");
    }
  else
    {
      session->state = TCP_STATE_ERROR;

      if (session->close != NULL)
	session->close(session, session->close_data);

      /* closing on error, just remove the session */

      tcp_session_remove(&sessions, session);
      mem_free(session);
    }
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

/*
 * Setup close callback.
 */

void	tcp_on_close(struct net_tcp_session_s	*session,
		     tcp_close_t		*callback,
		     void			*ptr)
{
  session->close = callback;
  session->close_data = ptr;
}

/*
 * Setup accept callback.
 */

void	tcp_on_accept(struct net_tcp_session_s	*session,
		      tcp_accept_t		*callback,
		      void			*ptr)
{
  session->accept = callback;
  session->accept_data = ptr;
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

  if (1) /* XXX buffer full */
    {
      /* send the data packet */
      tcp_send_datapkt(session, data, size, TH_PUSH);

      /* increment the sequence number */
      session->curr_seq += size;
    }

  /* XXX retransmission timeout */
}

/*
 * Interface with stack's TCP module.
 */

/*
 * Called on incoming connection.
 */

static void			libtcp_open(struct net_tcp_session_s	*session,
					    struct net_packet_s		*packet,
					    struct tcphdr		*hdr)
{
  if (session->state == TCP_STATE_SYN_SENT)
    {
      tcp_connect_t	*callback = session->connect;
      void		*ptr = session->connect_data;

      /* get mss if present */
      if (hdr->th_off > 5)
	{
	  uint32_t	opt;

	  opt = net_be32_load(*(uint32_t *)(hdr + 1));
	  if (opt & (2 << 24))
	    session->send_mss = opt & 0xffff;
	}

      /* make the transition */
      if (hdr->th_flags & TH_ACK)
	{
	  net_debug("-> SYN ACK\n");

	  /* ok, connection aknowleged */
	  session->state = TCP_STATE_ESTABLISHED;

	  /* get the sender seq & win */
	  session->recv_seq = net_be32_load(hdr->th_seq);
	  session->recv_win = net_be16_load(hdr->th_win);

	  /* increment seq and set ack */
	  session->to_ack = session->recv_seq + 1;

	  net_debug("<> ESTABLISHED\n");
	  net_debug("  send MSS = %u\n", session->send_mss);
	  net_debug("  recv MSS = %u\n", session->recv_mss);

	  /* send ACK */
	  net_debug("<- ACK\n");
	  session->curr_seq++;
	  tcp_send_controlpkt(session, TCP_ACK);

	  callback(session, ptr);
	}
      else
	{
	  net_debug("-> SYN\n");

	  /* enter SYN RCVD state  */
	  session->state = TCP_STATE_SYN_RCVD;

	  /* send ACK */
	  net_debug("<- ACK\n");
	  session->curr_seq++;
	  tcp_send_controlpkt(session, TCP_ACK);
	}

    }
  else if (session->state == TCP_STATE_LISTEN) /* incoming connection request */
    {
      struct net_tcp_session_s	*new;

      net_debug("-> SYN\n");

      /* enter SYN RCVD state  */
      session->state = TCP_STATE_SYN_RCVD;

      /* send SYN ACK */
      net_debug("<- SYN ACK\n");
      tcp_send_controlpkt(session, TCP_SYN_ACK);

      /* create a new session */
      new = mem_alloc(sizeof (struct net_tcp_session_s), MEM_SCOPE_SYS);
      new->interface = session->interface;
      new->addressing = session->addressing;

      /* XXX fill me */

      /* push the new session into the hashlist */
      tcp_session_push(&sessions, new);

      /* callback */
      if (session->accept != NULL)
	session->accept(session, new, session->accept_data);
    }
}

/*
 * Called on remote connection closing.
 */

static void			libtcp_close(struct net_tcp_session_s	*session,
					     struct net_packet_s	*packet,
					     struct tcphdr		*hdr)
{
  switch (session->state)
    {
      case TCP_STATE_SYN_SENT: /* error when opening */
      case TCP_STATE_SYN_RCVD:
	net_debug("-> FIN (while connecting)\n");

	tcp_send_controlpkt(session, TCP_FIN);
	net_debug("<- FIN ACK\n");

	/* set error state */
	session->state = TCP_STATE_ERROR;

	/* callback to report the error */
	session->connect(session, session->connect_data);

	tcp_session_remove(&sessions, session);
	mem_free(session);
	break;
      case TCP_STATE_FIN_WAIT1: /* it is a FIN acknowlegment */
	/* goto CLOSING */
	session->state = TCP_STATE_CLOSING;

	if (!(hdr->th_flags & TH_ACK))
	  {
	    net_debug("-> FIN\n");

	    /* send a ACK */
	    session->to_ack++;
	    tcp_send_controlpkt(session, TCP_FIN);
	    net_debug("<- ACK\n");
	    break;
	  }
	/* if FIN ACK, directly goto CLOSING state */
	net_debug("-> FIN ACK\n");
      case TCP_STATE_LAST_ACK:
      case TCP_STATE_CLOSING:
      case TCP_STATE_FIN_WAIT2:
	/* send a ACK */
	session->to_ack++;
	tcp_send_controlpkt(session, TCP_FIN);
	net_debug("<- ACK\n");

	session->state = TCP_STATE_CLOSED;

	if (session->close != NULL)
	  session->close(session, session->close_data);

	/* delete session */
	tcp_session_remove(&sessions, session);
	mem_free(session);
	break;
      default:
	/* otherwise, it is a FIN request */
	net_debug("-> FIN\n");

	/* send a ACK */
	session->to_ack++;
	tcp_send_controlpkt(session, TCP_ACK);
	net_debug("<- ACK\n");

	/* no data remaining, close the connection */
	if (1) /* XXX data remaining ? */
	  {
	    net_debug("<- FIN\n");
	    tcp_send_controlpkt(session, TCP_FIN);

	    session->state = TCP_STATE_CLOSED;

	    if (session->close != NULL)
	      session->close(session, session->close_data);

	    tcp_session_remove(&sessions, session);
	    mem_free(session);
	  }
	else /* otherwise, wait for the remaining operations and change state */
	  session->state = TCP_STATE_CLOSE_WAIT;
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
  size_t			length;
  uint_fast32_t			seq;

  /* look for the corresponding session */
  memcpy(&key.address, &packet->sADDR, sizeof (struct net_addr_s));
  key.port = net_16_load(hdr->th_sport);

  if ((session = tcp_session_lookup(&sessions, (void *)&key)) == NULL)
    return ;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  length = nethdr->size - hdr->th_off * 4;

  /* check for lost packet(s) */
  seq = net_be32_load(hdr->th_seq);
  if (seq != session->to_ack)
    {
      /* XXX a packet was lost (receive) */

    }

  /* update sequence numbers */
  session->recv_seq = seq;
  session->to_ack = session->recv_seq + length;

  /* check for transition from SYN RCVD to ESTABLISHED */
  if (session->state == TCP_STATE_SYN_RCVD)
    {
      net_debug("-> ACK\n");

      net_debug("<> ESTABLISHED\n");
      net_debug("  send MSS = %u\n", session->send_mss);
      net_debug("  recv MSS = %u\n", session->recv_mss);

      session->state = TCP_STATE_ESTABLISHED;
    }

  /* check for SYN flag */
  if (hdr->th_flags & TH_SYN)
    libtcp_open(session, packet, hdr);

  /* control packet */
  if (nethdr->size == hdr->th_off * 4)
    {
      /* XXX nothing here ?? */
    }
  else /* data packet */
    {
      uint8_t	*data = (nethdr->data + hdr->th_off * 4);

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

      /* if needed, send a control packet to acknowledge */
      if (1 && !(hdr->th_flags & TH_FIN)) /* XXX ACK acumulés */
	{
	  tcp_send_controlpkt(session, TCP_ACK);
	  net_debug("<- ACK\n");
	}
    }

  /* check for FIn flag */
  if (hdr->th_flags & TH_FIN)
    libtcp_close(session, packet, hdr);
}

