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
#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/ether.h>

#include <netinet/tcp.h>

#include <netinet/libtcp.h>

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

struct net_tcp_session_s	*tcp_open(struct net_tcp_addr_s	*local,
					  struct net_tcp_addr_s	*remote)
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
    return NULL;

  /* create session instance */
  session = mem_alloc(sizeof (struct net_tcp_session_s), MEM_SCOPE_SYS);
  session->interface = interface;
  session->addressing = addressing;

  /* choose a local port */
  local->port = 0x4242; /* XXX choose me better! */
  memcpy(&session->local, local, sizeof (struct net_tcp_addr_s));
  memcpy(&session->remote, remote, sizeof (struct net_tcp_addr_s));

  session->send_seq = 1; /* XXX generate it */
  session->send_ack = session->send_seq + 1;
  session->send_win = TCP_DFL_WINDOW;

  /* enter SYN sent mode, waiting for SYN ACK */
  session->state = TCP_STATE_SYN_SENT;

  /* push the new session into the hashlist */
  tcp_session_push(&sessions, session);

  /* XXX timeout here */

  sem_init(&session->sem, 0, 0);

  /* send the SYN packet */
  tcp_send_controlpkt(session, TCP_OPEN);
  printf("<- SYN\n");

  /* wait for connection being established (or timeout'ed) */
  sem_wait(&session->sem);
  sem_destroy(&session->sem);

  printf("<> ESTABLISHED\n");

  /* error, delete the session */
  if (session->state == TCP_STATE_ERROR)
    {
      tcp_session_remove(&sessions, session);
      mem_free(session);
      return NULL;
    }

  return session;
}

/*
 * Close an opened TCP session.
 */

void			tcp_close(struct net_tcp_session_s	*session)
{
  /* enter FIN WAIT state, waiting for FIN */
  session->state = TCP_STATE_FIN_WAIT;

  /* just send a close request */
  tcp_send_controlpkt(session, TCP_CLOSE);
  printf("<- FIN\n");
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
      if ((hdr->th_flags & TH_ACK) && (net_be32_load(hdr->th_ack) == session->send_ack))
	{
	  printf("-> SYN ACK\n");

	  /* ok, connection aknowleged */
	  session->state = TCP_STATE_ESTABLISHED;

	  /* get the sender seq & win */
	  session->recv_seq = net_be32_load(hdr->th_seq);
	  session->recv_ack = session->recv_seq + 1;
	  session->recv_win = net_be16_load(hdr->th_win);

	  /* increment seq */
	  session->send_seq++;
	}
      else /* otherwise, this is a bad connection sequence, report error */
	session->state = TCP_STATE_ERROR;

      /* wake up the opening thread */
      sem_post(&session->sem);
    }
  else if (session->state == TCP_STATE_LISTEN)
    {
      /* incoming connection request */

      /* XXX */
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

  /* it is a FIN acknowlegment */
  if (session->state == TCP_STATE_FIN_WAIT)
    {
      printf("-> FIN ACK\n");

      tcp_session_remove(&sessions, session);
      mem_free(session);
    }
  else /* otherwise, it is a FIN request */
    {
      printf("-> FIN\n");
      /* XXX */
    }
}

/*
 * Called on packet incoming (data or control).
 */

void		libtcp_push(struct net_packet_s	*packet,
			    struct tcphdr	*hdr)
{
  /* XXX */
}

