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
#include <netinet/socket_packet.h>
#include <netinet/if.h>
#include <netinet/arp.h>

#include <hexo/alloc.h>

#include <semaphore.h>
#include <timer.h>

static _RECVFROM(recvfrom_packet);

CONTAINER_FUNC(static inline, socket_packet, DLIST, socket_packet, NOLOCK);

socket_packet_root_t	pf_packet = CONTAINER_ROOT_INITIALIZER(socket_packet, DLIST, NOLOCK);

/*
 * Create a PACKET socket. Allocate private data.
 */

static _SOCKET(socket_packet)
{
  struct socket_packet_pv_s	*pv;

  if ((pv = fd->pv = mem_alloc(sizeof (struct socket_packet_pv_s), MEM_SCOPE_NETWORK)) == NULL)
    return -ENOMEM;

  /* setup private data */
  pv->proto = ntohs(protocol);
  pv->interface = 0;
  pv->shutdown = -1;
  pv->broadcast = 0;
  pv->recv_timeout = 0;
  sem_init(&pv->recv_sem, 0, 0);
  packet_queue_lock_init(&pv->recv_q);
  pv->header = (type == SOCK_RAW);
  if (!socket_packet_push(&pf_packet, pv))
    {
      sem_destroy(&pv->recv_sem);
      packet_queue_lock_destroy(&pv->recv_q);
      mem_free(pv);
      return -ENOMEM;
    }

  return 0;
}

/*
 * Set a PACKET socket to listen on a given local address (or interface).
 */

static _BIND(bind_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;
  struct sockaddr_ll		*sll = (struct sockaddr_ll *)addr;
  struct net_if_s		*interface;

  if (len < sizeof (struct sockaddr_ll) || sll->sll_family != AF_PACKET)
    {
      fd->error = EINVAL;
      return -1;
    }

  /* bind to the given interface and protocol */
  if ((interface = if_get_by_index(sll->sll_ifindex)) == NULL)
    {
      fd->error = EADDRNOTAVAIL;
      return -1;
    }

  pv->interface = interface->index;
  pv->proto = ntohs(sll->sll_protocol);

  return 0;
}

/*
 * Get the socket local address.
 */

static _GETSOCKNAME(getsockname_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;
  struct sockaddr_ll		*sll = (struct sockaddr_ll *)addr;

  if (*len < sizeof (struct sockaddr_ll))
    {
      fd->error = EINVAL;
      return -1;
    }

  /* not bound... */
  if (pv->interface == 0)
    {
      fd->error = EADDRNOTAVAIL;
      return -1;
    }

  /* fill socket name */
  sll->sll_family = AF_PACKET;
  sll->sll_ifindex = pv->interface;
  sll->sll_protocol = htons(pv->proto);

  *len = sizeof (struct sockaddr_ll);

  return 0;
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDTO(sendto_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;
  struct sockaddr_ll		*sll = (struct sockaddr_ll *)addr;
  struct net_packet_s		*packet;
  struct net_if_s		*interface;
  struct net_header_s		*nethdr;

  if (pv->shutdown == SHUT_WR || pv->shutdown == SHUT_RDWR)
    {
      fd->error = ESHUTDOWN;
      return -1;
    }

  if ((sll == NULL && !pv->header) || addr_len < sizeof (struct sockaddr_ll) ||
      sll->sll_family != AF_PACKET)
    {
      fd->error = (sll == NULL ? EDESTADDRREQ : EINVAL);
      return -1;
    }

  if ((packet = packet_obj_new(NULL)) == NULL)
    {
      fd->error = ENOMEM;
      return -1;
    }

  /* retrieve the interface from the if index */
  interface = if_get_by_index(sll->sll_ifindex);

  if (pv->header)
    {
      /* alloc a buffer to copy the packet content */
      if ((packet->packet = mem_alloc(n, MEM_SCOPE_NETWORK)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = ENOMEM;
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
      uint8_t	bcast[8];
      size_t	maclen = 8;

      assert(!dev_net_getopt(interface->dev, DEV_NET_OPT_BCAST, bcast, &maclen));

      /* is broadcast allowed */
      if (!pv->broadcast && !memcmp(bcast, sll->sll_addr, maclen))
	{
	  packet_obj_refdrop(packet);
	  fd->error = EINVAL;
	  return -1;
	}

      /* prepare the packet */
      if ((next = if_preparepkt(interface, packet, n, 0)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = ENOMEM;
	  return -1;
	}

      /* set the packet content */
      nethdr = &packet->header[packet->stage];
      nethdr->data = next;
      nethdr->size = n;
      nethdr[1].data = NULL;
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

/*
 * Receive some data and get the source address.
 */

static _RECVFROM(recvfrom_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;
  struct net_packet_s		*packet;
  struct sockaddr_ll		*sll = (struct sockaddr_ll *)addr;
  ssize_t			sz;
  struct net_header_s		*nethdr;

  /* try to grab a packet */
 again:
  if (pv->shutdown == SHUT_RD || pv->shutdown == SHUT_RDWR)
    {
      fd->error = ESHUTDOWN;
      return -1;
    }

  if (flags & MSG_PEEK)
    packet = packet_queue_lock_head(&pv->recv_q);
  else
    packet = packet_queue_lock_pop(&pv->recv_q);

  if (packet == NULL)
    {
      struct timer_event_s	timeout;

      if (flags & MSG_DONTWAIT)
	{
	  fd->error = EAGAIN;
	  return -1;
	}

      /* if there is a receive timeout, start a timer */
      if (pv->recv_timeout)
	{
#if 0
	  timeout.callback = recv_timeout;
	  timeout.delay = pv->recv_timeout;

	  timer_add_event(&timer_ms, &timeout); /* XXX */
#endif
	}

      /* wait */
      sem_wait(&pv->recv_sem);

      goto again;
    }

  /* fill the address if required */
  if (sll != NULL)
    {
      if (*addr_len < sizeof (struct sockaddr_ll))
	{
	  fd->error = ENOMEM;
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

/*
 * Set a socket option value.
 */

static _SETSOCKOPT(setsockopt_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;
  struct packet_mreq		*req;
  struct net_if_s		*interface;

  switch (level)
    {
      /* PF_PACKET options */
      case SOL_PACKET:
	if (optname != PACKET_ADD_MEMBERSHIP && optname != PACKET_DROP_MEMBERSHIP)
	  {
	    fd->error = ENOPROTOOPT;
	    return -1;
	  }

	if (optlen < sizeof (struct packet_mreq))
	  {
	    fd->error = EINVAL;
	    return -1;
	  }

	req = (struct packet_mreq *)optval;

	switch (req->mr_type)
	  {
	    /* enable or disable promiscuous mode */
	    case PACKET_MR_PROMISC:
	      {
		bool_t	enabled = optname == PACKET_ADD_MEMBERSHIP;

		if ((interface = if_get_by_index(req->mr_ifindex)) == NULL)
		  {
		    fd->error = EADDRNOTAVAIL;
		    return -1;
		  }
		dev_net_setopt(interface->dev, DEV_NET_OPT_PROMISC, &enabled, sizeof (bool_t));
	      }
	      break;
	      /* other options not supported (multicast) */
	    default:
	      fd->error = ENOPROTOOPT;
	      return -1;
	  }
	break;
      /* socket options */
      case SOL_SOCKET:
	switch (optname)
	  {
	    /* recv timeout */
	    case SO_RCVTIMEO:
	      {
		const struct timeval	*tv;

		if (optlen < sizeof (struct timeval))
		  {
		    fd->error = EINVAL;
		    return -1;
		  }

		tv = optval;
		pv->recv_timeout = tv->tv_sec * 1000 + tv->tv_usec / 1000;
	      }
	      break;
	    /* allow sending/receiving broadcast */
	    case SO_BROADCAST:
	      {
		const int		*enable;

		if (optlen < sizeof (int))
		  {
		    fd->error = EINVAL;
		    return -1;
		  }

		enable = optval;
		pv->broadcast = *enable;
	      }
	      break;
	    default:
	      fd->error = ENOPROTOOPT;
	      return -1;
	  }
	return 0;
	break;
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}

/*
 * Get a socket option value.
 */

static _GETSOCKOPT(getsockopt_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;

  if (level != SOL_SOCKET)
    {
      fd->error = ENOPROTOOPT;
      return -1;
    }

  switch (optname)
    {
      /* recv timeout */
      case SO_RCVTIMEO:
	{
	  struct timeval	*tv;

	  if (*optlen < sizeof (struct timeval))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  tv = optval;
	  tv->tv_usec = (pv->recv_timeout % 1000) * 1000;
	  tv->tv_sec = pv->recv_timeout / 1000;

	  *optlen = sizeof (struct timeval);
	}
	break;
      /* broadcast enabled */
      case SO_BROADCAST:
	{
	  int			*enabled;

	  if (*optlen < sizeof (int))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  enabled = optval;
	  *enabled = pv->broadcast;

	  *optlen = sizeof (int);
	}
	break;
      /* socket type */
      case SO_TYPE:
	{
	  int			*type;

	  if (*optlen < sizeof (int))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  type = optval;
	  *type = (pv->header ? SOCK_RAW : SOCK_DGRAM);

	  *optlen = sizeof (int);
	}
	break;
      /* socket last error */
      case SO_ERROR:
	{
	  int			*error;

	  if (*optlen < sizeof (int))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  error = optval;
	  *error = fd->error;

	  *optlen = sizeof (int);
	}
	break;
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}


/*
 * Stop dataflow on a socket.
 */

static _SHUTDOWN(shutdown_packet)
{
  struct socket_packet_pv_s	*pv = (struct socket_packet_pv_s *)fd->pv;

  if (how != SHUT_RDWR && how != SHUT_RD && how != SHUT_WR)
    {
      fd->error = EINVAL;
      return -1;
    }

  /* check combinations */
  if (how == SHUT_RDWR || (pv->shutdown == SHUT_RD && how == SHUT_WR) ||
      (pv->shutdown == SHUT_WR && how == SHUT_RD))
    pv->shutdown = SHUT_RDWR;
  else
    pv->shutdown = how;

  /* end all the recv with errors */
  if (pv->shutdown == SHUT_RDWR || pv->shutdown == SHUT_RD)
    {
      uint_fast8_t		val;
      struct net_packet_s	*packet;

      /* drop all waiting packets */
      while ((packet = packet_queue_lock_pop(&pv->recv_q)) != NULL)
	packet_obj_refdrop(packet);

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
 * Following operations are not supported with PACKET sockets.
 */

static _LISTEN(listen_packet) { fd->error = EOPNOTSUPP; return -1; }
static _ACCEPT(accept_packet) { fd->error = EOPNOTSUPP; return -1; }
static _CONNECT(connect_packet) { fd->error = EOPNOTSUPP; return -1; }
static _GETPEERNAME(getpeername_packet) { fd->error = EOPNOTSUPP; return -1; }

/*
 * Socket API for PACKET sockets.
 */

const struct socket_api_s	packet_socket =
  {
    .socket = socket_packet,
    .bind = bind_packet,
    .getsockname = getsockname_packet,
    .connect = connect_packet,
    .getpeername = getpeername_packet,
    .sendto = sendto_packet,
    .recvfrom = recvfrom_packet,
    .getsockopt = getsockopt_packet,
    .setsockopt = setsockopt_packet,
    .listen = listen_packet,
    .accept = accept_packet,
    .shutdown = shutdown_packet
  };

/*
 * Signal an incoming packet at level 2 layer.
 */

void		pf_packet_signal(struct net_if_s	*interface,
				 struct net_packet_s	*packet,
				 net_proto_id_t		protocol)
{
  uint8_t	bcast[packet->MAClen];
  bool_t	is_bcast;
  size_t	maclen = packet->MAClen;

  assert(!dev_net_getopt(interface->dev, DEV_NET_OPT_BCAST, bcast, &maclen));
  assert(maclen == packet->MAClen);
  is_bcast = !memcmp(packet->tMAC, bcast, maclen);

  /* deliver packet to all sockets matching interface and protocol id */
  CONTAINER_FOREACH(socket_packet, DLIST, NOLOCK, &pf_packet,
  {
    if (item->shutdown == SHUT_RD || item->shutdown == SHUT_RDWR)
      CONTAINER_FOREACH_CONTINUE;

    if (item->interface == 0 || item->interface == interface->index)
      {
	if (item->proto == protocol || item->proto == ETH_P_ALL)
	  {
	    if (!item->broadcast && is_bcast)
	      CONTAINER_FOREACH_CONTINUE;

	    if (packet_queue_lock_pushback(&item->recv_q, packet))
	      sem_post(&item->recv_sem);
	  }
      }
  });
}
