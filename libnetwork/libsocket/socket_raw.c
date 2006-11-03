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

#include <hexo/device.h>
#include <hexo/device/net.h>
#include <hexo/driver.h>

#include <netinet/packet.h>
#include <netinet/socket.h>
#include <netinet/socket_raw.h>
#include <netinet/if.h>
#include <netinet/arp.h>

#include <hexo/alloc.h>

#include <semaphore.h>

static _RECVFROM(recvfrom_raw);

CONTAINER_FUNC(static inline, socket_raw, DLIST, socket_raw, NOLOCK);

static socket_raw_root_t	pf_packet = CONTAINER_ROOT_INITIALIZER(socket_raw, DLIST, NOLOCK);
static socket_raw_root_t	sock_raw = CONTAINER_ROOT_INITIALIZER(socket_raw, DLIST, NOLOCK);

/*
 * Create a RAW socket. Allocate private data.
 */

static _SOCKET(socket_raw)
{
  struct socket_raw_pv_s	*pv;
  pv = fd->pv = mem_alloc(sizeof (struct socket_raw_pv_s), MEM_SCOPE_NETWORK);

  protocol = ntohs(protocol);
  /* setup private data */
  pv->layer = (domain == PF_PACKET ? 2 : 3);
  pv->proto = protocol;
  pv->interface = 0;
  pv->shutdown = -1;
  sem_init(&pv->recv_sem, 0, 0);
  packet_queue_lock_init(&pv->recv_q);

  /* determine if headers must be included or not */
  switch (domain)
    {
      case PF_PACKET:
	pv->header = (type == SOCK_RAW);
	socket_raw_push(&pf_packet, pv);
	break;
      case PF_INET:
      case PF_INET6:
	pv->header = (protocol == IPPROTO_RAW);
	socket_raw_push(&sock_raw, pv);
	break;
      default:
	assert(0);
    }

  return fd;
}

/*
 * Set a RAW socket to listen on a given local address (or interface).
 */

static _BIND(bind_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  /* on PF_PACKET */
  if (pv->layer == 2)
    {
      struct sockaddr_ll	*sll = (struct sockaddr_ll *)addr;
      struct net_if_s		*interface;

      if (len < sizeof (struct sockaddr_ll) || sll->sll_family != AF_PACKET)
	return -1;

      /* bind to the given interface and protocol */
      if ((interface = if_get_by_index(sll->sll_ifindex)) == NULL)
	return -1;

      pv->interface = interface->index;
      pv->proto = ntohs(sll->sll_protocol);

      return 0;
    }
  else /* on SOCK_RAW */
    {

    }

  return -1;
}

/*
 * Get the socket local address.
 */

static _GETSOCKNAME(getsockname_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  /* on PF_PACKET */
  if (pv->layer == 2)
    {
      struct sockaddr_ll	*sll = (struct sockaddr_ll *)addr;

      if (*len < sizeof (struct sockaddr_ll))
	return -1;

      /* not bound... */
      if (pv->interface == 0)
	return -1;

      /* fill socket name */
      sll->sll_family = AF_PACKET;
      sll->sll_ifindex = pv->interface;
      sll->sll_protocol = htons(pv->proto);

      *len = sizeof (struct sockaddr_ll);

      return 0;
    }
  else /* on SOCK_RAW */
    {

    }

  return -1;
}

/*
 * Connect a RAW socket to a remote address.
 */

static _CONNECT(connect_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  /* not supported on PF_PACKET */
  if (pv->layer == 2)
    return -1;

  return -1;
}

/*
 * Get the remote address.
 */

static _GETPEERNAME(getpeername_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  /* not supported on PF_PACKET */
  if (pv->layer == 2)
    return -1;

  return -1;
}

/*
 * Send a chunk of data.
 */

static _SEND(send_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  /* not supported on PF_PACKET */
  if (pv->layer == 2)
    return -1;

  return -1;
}

/*
 * Receive a chunk of data.
 */

static _RECV(recv_raw)
{
  return recvfrom_raw(fd, buf, n, flags, NULL, NULL);
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDTO(sendto_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct sockaddr_ll		*sll = (struct sockaddr_ll *)addr;
  struct net_packet_s		*packet;
  struct net_if_s		*interface;

  if (pv->shutdown == SHUT_WR || pv->shutdown == SHUT_RDWR)
    return -1;

  if (pv->layer == 2 && (addr_len < sizeof (struct sockaddr_ll) || sll->sll_family != AF_PACKET))
    return -1;

  if ((packet = packet_obj_new(NULL)) == NULL)
    return -1;

  /* PF_PACKET */
  if (pv->layer == 2)
    {
      struct net_header_s	*nethdr;

      /* retrieve the interface from the if index */
      interface = if_get_by_index(sll->sll_ifindex);

      if (pv->header)
	{
	  /* alloc a buffer to copy the packet content */
	  if ((packet->packet = mem_alloc(n, MEM_SCOPE_NETWORK)) == NULL)
	    {
	      packet_obj_refdrop(packet);
	      return -1;
	    }

	  /* set the packet content */
	  memcpy(packet->packet, buf, n);
	  nethdr = &packet->header[0];
	  nethdr->data = packet->packet;
	  nethdr->size = n;
	  nethdr[1].data = NULL;
	  packet->stage = -1;
	  /* send it to the driver */
	  if_sendpkt(interface, packet, ntohs(sll->sll_protocol));
	}
      else
	{
	  uint8_t	*next;

	  /* prepare the packet */
	  if ((next = if_preparepkt(interface, packet, n, 0)) == NULL)
	    {
	      packet_obj_refdrop(packet);
	      return -1;
	    }

	  /* set the packet content */
	  nethdr = &packet->header[packet->stage];
	  nethdr->data = next;
	  nethdr->size = n;
	  memcpy(next, buf, n);
	  packet->header[packet->stage + 1].data = NULL;
	  packet->MAClen = sll->sll_halen;
	  packet->tMAC = sll->sll_addr;
	  packet->stage--;
	  /* send to the driver */
	  if_sendpkt(interface, packet, ntohs(sll->sll_protocol));
	}

      return n;
    }
  else /* SOCK_RAW */
    {

    }

  return -1;
}

/*
 * Receive some data and get the source address.
 */

static _RECVFROM(recvfrom_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct net_packet_s		*packet;

  /* try to grab a packet */
 again:
  if (pv->shutdown == SHUT_RD || pv->shutdown == SHUT_RDWR)
    return -1;

  if (flags & MSG_PEEK)
    packet = packet_queue_lock_head(&pv->recv_q);
  else
    packet = packet_queue_lock_pop(&pv->recv_q);

  if (packet == NULL)
    {
      sem_wait(&pv->recv_sem);

      goto again;
    }

  /* process the packet */
  if (pv->layer == 2)
    {
      struct sockaddr_ll	*sll = (struct sockaddr_ll *)addr;
      ssize_t			sz;
      struct net_header_s	*nethdr;

      /* fill the address if required */
      if (sll != NULL)
	{
	  if (*addr_len < sizeof (struct sockaddr_ll))
	    {
	      packet_obj_refdrop(packet);
	      return -1;
	    }

	  sll->sll_family = AF_PACKET;
	  sll->sll_protocol = htons(packet->proto);
	  sll->sll_ifindex = packet->interface->index;
	  sll->sll_hatype = ARPHRD_ETHER; /* XXX */
	  sll->sll_halen = packet->MAClen;
	  memcpy(sll->sll_addr, packet->sMAC, packet->MAClen);
	  if (!memcmp(packet->interface->mac, packet->tMAC, packet->MAClen))
	    sll->sll_pkttype = PACKET_HOST;
	  else if (!memcmp("\xff\xff\xff\xff\xff\xff\xff\xff", packet->tMAC, packet->MAClen))
	    sll->sll_pkttype = PACKET_BROADCAST;
	  else
	    sll->sll_pkttype = PACKET_OTHERHOST;

	  *addr_len = sizeof (struct sockaddr_ll);
	}

      /* copy the data */
      if (pv->header)
	{
	  ssize_t	header_sz;

	  nethdr = &packet->header[0];
	  header_sz = nethdr->size - nethdr[1].size;
	  if (header_sz > n)
	    {
	      header_sz = n;
	      sz = 0;
	    }
	  else
	    sz = nethdr->size > n ? n : nethdr->size;
	  memcpy(buf, nethdr->data, header_sz);
	  if (sz)
	    memcpy(buf + header_sz, nethdr[1].data, sz - header_sz);
	}
      else
	{
	  nethdr = &packet->header[packet->stage];
	  sz = nethdr->size > n ? n : nethdr->size;
	  memcpy(buf, nethdr->data, sz);
	}

      /* drop the packet */
      packet_obj_refdrop(packet);

      return sz;
    }
  else
    {

    }

  return -1;
}

/*
 * Send a message.
 */

static _SENDMSG(sendmsg_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Receive a message.
 */

static _RECVMSG(recvmsg_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Get a socket option value.
 */

static _GETSOCKOPT(getsockopt_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  /* no option to get on PF_PACKET */
  if (pv->layer == 2)
    return -1;

  return -1;
}

/*
 * Set a socket option value.
 */

static _SETSOCKOPT(setsockopt_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  /* sockopt on PF_PACKET */
  if (pv->layer == 2)
    {
      struct packet_mreq	*req = (struct packet_mreq *)optval;
      struct net_if_s		*interface;

      if (optname != PACKET_ADD_MEMBERSHIP && optname != PACKET_DROP_MEMBERSHIP)
	return -1;

      if (optlen < sizeof (struct packet_mreq))
	return -1;

      switch (req->mr_type)
	{
	  /* enable or disable promiscuous mode */
	  case PACKET_MR_PROMISC:
	    if ((interface = if_get_by_index(req->mr_ifindex)) == NULL)
	      return -1;
	    dev_net_setopt(interface->dev, DEV_NET_OPT_PROMISC, optname == PACKET_ADD_MEMBERSHIP);
	    break;
	  /* other options not supported (multicast) */
	  default:
	    return -1;
	}

      return 0;
    }
  else /* on SOCK_RAW */
    {

    }

  return -1;
}

/*
 * Stop dataflow on a socket.
 */

static _SHUTDOWN(shutdown_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  uint_fast8_t			val;

  if (how != SHUT_RDWR && how != SHUT_RD && how != SHUT_WR)
    return -1;

  /* check combinations */
  if (how == SHUT_RDWR || (pv->shutdown == SHUT_RD && how == SHUT_WR) ||
      (pv->shutdown == SHUT_WR && how == SHUT_RD))
    pv->shutdown = SHUT_RDWR;
  else
    pv->shutdown = how;

  /* end all the recv with errors */
  if (pv->shutdown == SHUT_RDWR || pv->shutdown == SHUT_RD)
    {
      sem_getvalue(&pv->recv_sem, &val);
      while (val < 0)
	{
	  sem_post(&pv->recv_sem);
	  sem_getvalue(&pv->recv_sem, &val);
	}
    }

  return 0;
}

/*
 * Following operations are not supported with RAW sockets.
 */

static _LISTEN(listen_raw) { return -1; }
static _ACCEPT(accept_raw) { return -1; }

/*
 * Socket API for RAW and PACKET sockets.
 */

const struct socket_api_s	raw_socket =
  {
    .socket = socket_raw,
    .bind = bind_raw,
    .getsockname = getsockname_raw,
    .connect = connect_raw,
    .getpeername = getpeername_raw,
    .send = send_raw,
    .recv = recv_raw,
    .sendto = sendto_raw,
    .recvfrom = recvfrom_raw,
    .sendmsg = sendmsg_raw,
    .recvmsg = recvmsg_raw,
    .getsockopt = getsockopt_raw,
    .setsockopt = setsockopt_raw,
    .listen = listen_raw,
    .accept = accept_raw,
    .shutdown = shutdown_raw
  };

/*
 * Signal an incoming packet (for RAW sockets).
 *
 * MUST NOT ALTER THE PACKET.
 */

void			libsocket_signal(struct net_if_s	*interface,
					 struct net_packet_s	*packet,
					 net_proto_id_t		protocol)
{
  if (packet->stage == 1) /* for PF_PACKET */
    {
      /* deliver packet to all sockets matching interface and protocol id */
      CONTAINER_FOREACH(socket_raw, DLIST, NOLOCK, &pf_packet,
      {
	if (item->shutdown == SHUT_RD || item->shutdown == SHUT_RDWR)
	  CONTAINER_FOREACH_CONTINUE;

	if (item->interface == 0 || item->interface == interface->index)
	  {
	    if (item->proto == protocol || item->proto == ETH_P_ALL)
	      {
		packet_queue_lock_push(&item->recv_q, packet);
		sem_post(&item->recv_sem);
	      }
	  }
      });
    }
  else if (packet->stage == 2) /* for SOCK RAW*/
    {

    }
}
