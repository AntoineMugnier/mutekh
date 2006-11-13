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
#include <netinet/route.h>
#include <netinet/arp.h>

#include <hexo/alloc.h>

#include <semaphore.h>

CONTAINER_FUNC(static inline, socket_raw, DLIST, socket_raw, NOLOCK);

static socket_raw_root_t	sock_raw = CONTAINER_ROOT_INITIALIZER(socket_raw, DLIST, NOLOCK);

/*
 * Create a RAW socket. Allocate private data.
 */

static _SOCKET(socket_raw)
{
  struct socket_raw_pv_s	*pv;

  if ((pv = fd->pv = mem_alloc(sizeof (struct socket_raw_pv_s), MEM_SCOPE_NETWORK)) == NULL)
    return -ENOMEM;

  protocol = ntohs(protocol);
  /* setup private data */
  switch (domain)
    {
      case PF_INET:
	IPV4_ADDR_SET(pv->local, INADDR_ANY);
	IPV4_ADDR_SET(pv->remote, INADDR_NONE);
	pv->any = 1;
	pv->connected = 0;
	pv->family = AF_INET;
	break;
      case PF_INET6:
	/* IPV6 */
      default:
	mem_free(pv);
	return -EPFNOSUPPORT;
    }
  pv->proto = protocol;
  pv->shutdown = -1;
  sem_init(&pv->recv_sem, 0, 0);
  packet_queue_lock_init(&pv->recv_q);

  /* determine if headers must be included or not */
  pv->header = (protocol == IPPROTO_RAW);
  if (!socket_raw_push(&sock_raw, pv))
    {
      sem_destroy(&pv->recv_sem);
      packet_queue_lock_destroy(&pv->recv_q);
      mem_free(pv);
      return -ENOMEM;
    }

  return 0;
}

/*
 * Set a RAW socket to listen on a given local address (or interface).
 */

static _BIND(bind_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct sockaddr_in		*in;
  struct net_if_s		*interface = NULL;
  struct net_proto_s		*addressing = NULL;
  net_proto_id_t		id;
  struct net_addr_s		address;
  bool_t			any;

  if (addr->sa_family != pv->family)
    {
      fd->error = EAFNOSUPPORT;
      return -1;
    }

  /* do protocol dependent things */
  switch (addr->sa_family)
    {
      case AF_INET:
	{
	  uint_fast32_t	ip;

	  if (len < sizeof (struct sockaddr_in))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  in = (struct sockaddr_in *)addr;
	  id = ETHERTYPE_IP;
	  ip = ntohl(in->sin_addr.s_addr);
	  any = (ip == INADDR_ANY);
	  IPV4_ADDR_SET(address, ip);
	}
	break;
      case AF_INET6:
	/* IPV6 */
      default:
	fd->error = EAFNOSUPPORT;
	return -1;
    }

  /* look for address validity */
  CONTAINER_FOREACH(net_if, HASHLIST, NOLOCK, &net_interfaces,
  {
    interface = item;
    for (addressing = net_protos_lookup(&interface->protocols, id);
	 addressing != NULL;
	 addressing = net_protos_lookup_next(&interface->protocols, addressing, id))
      if (addressing->desc->f.addressing->matchaddr(addressing, &address, NULL, NULL))
	goto ok;

  });

 ok:
  if (interface == NULL || addressing == NULL)
    {
      fd->error = EADDRNOTAVAIL;
      return -1;
    }

  /* setup the local address */
  memcpy(&pv->local, &address, sizeof (struct net_addr_s));
  pv->any = any;

  return 0;
}

/*
 * Get the socket local address.
 */

static _GETSOCKNAME(getsockname_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  if (socket_addr_in(fd, &pv->local, addr, len, htons(pv->proto)))
    return -1;

  return 0;
}

/*
 * Connect a RAW socket to a remote address.
 */

static _CONNECT(connect_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct sockaddr_in		*in;
  struct net_addr_s		dest;
  struct net_route_s		*route;

  if (addr->sa_family != pv->family)
    {
      fd->error = EAFNOSUPPORT;
      return -1;
    }

  if (socket_in_addr(fd, &dest, addr, len, NULL))
    return -1;

  if ((route = route_get(&dest)) == NULL)
    {
      fd->error = EADDRNOTAVAIL;
      return -1;
    }

  memcpy(&pv->remote, &dest, sizeof (struct net_addr_s));
  pv->interface = route->interface;
  pv->addressing = route->addressing;
  pv->connected = 1;

  return 0;
}

/*
 * Get the remote address.
 */

static _GETPEERNAME(getpeername_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  if (!pv->connected)
    {
      fd->error = ENOTCONN;
      return -1;
    }

  if (socket_addr_in(fd, &pv->remote, addr, len, htons(pv->proto)))
    return -1;

  return 0;
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDTO(sendto_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct sockaddr_in		*in;
  struct net_packet_s		*packet;
  struct net_if_s		*interface;
  struct net_proto_s		*addressing;
  struct net_header_s		*nethdr;
  uint8_t			*p;

  if ((packet = packet_obj_new(NULL)) == NULL)
    {
      fd->error = ENOMEM;
      return -1;
    }

  /* determine endpoint */
  if (addr == NULL)
    {
      if (!pv->connected)
	{
	  packet_obj_refdrop(packet);
	  fd->error = EDESTADDRREQ;
	  return -1;
	}

      memcpy(&packet->tADDR, &pv->remote, sizeof (struct net_addr_s));
    }
  else
    {
      if (addr->sa_family != pv->family)
	{
	  fd->error = EAFNOSUPPORT;
	  return -1;
	}

      if (socket_in_addr(fd, &packet->tADDR, addr, addr_len, NULL))
	{
	  packet_obj_refdrop(packet);
	  return -1;
	}
    }

  /* now, deduce the source */
  if (pv->connected)
    {
      /* simply read the pv info */
      interface = pv->interface;
      addressing = pv->addressing;
    }
  else
    {
      /* otherwise, determine the route */
      struct net_route_s	*route;

      if ((route = route_get(&packet->tADDR)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = EHOSTUNREACH;
	  return -1;
	}
      interface = route->interface;
      addressing = route->addressing;
    }

  /* prepare the packet */
  if (pv->header)
    {
      if ((p = if_preparepkt(interface, packet, n, 0)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = ENOMEM;
	  return -1;
	}
    }
  else
    {
      if ((p = addressing->desc->preparepkt(interface, packet, n, 0)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = ENOMEM;
	  return -1;
	}
    }
  nethdr = &packet->header[packet->stage];
  nethdr->data = p;
  nethdr->size = n;
  nethdr[1].data = NULL;

  /* copy the buffer into the packet */
  memcpy(p, buf, n);

  /* adjust some IP header fields */
  if (pv->header)
    {
      switch (pv->family)
	{
	  case AF_INET:
	    /* XXX */
	    break;
	  case AF_INET6:
	    /* IPV6 */
	  default:
	    assert(0);
	}
    }

  /* send the packet */
  packet->stage--;
  addressing->desc->f.addressing->sendpkt(interface, packet, addressing, pv->proto);

  return n;
}

/*
 * Receive some data and get the source address.
 */

static _RECVFROM(recvfrom_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct net_packet_s		*packet;
  struct sockaddr_in		*in;
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
      if (flags & MSG_DONTWAIT)
	{
	  fd->error = EAGAIN;
	  return -1;
	}

      sem_wait(&pv->recv_sem);

      goto again;
    }

  /* fill the address if required */
  if (addr != NULL)
    {
      if (socket_addr_in(fd, &packet->sADDR, addr, addr_len, htons(pv->proto)))
	{
	  packet_obj_refdrop(packet);
	  return -1;
	}
    }

  /* copy the data */
  if (pv->header)
    {
      /* XXX */
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
 * Get a socket option value.
 */

static _GETSOCKOPT(getsockopt_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  switch (level) /* XXX */
    {
      case SOL_SOCKET:
	break;
      case SOL_IP:
	break;
      case SOL_RAW:
	break;
      default:
	return -1;
    }

  return 0;
}

/*
 * Set a socket option value.
 */

static _SETSOCKOPT(setsockopt_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

  switch (level) /* XXX */
    {
      case SOL_SOCKET:
	break;
      case SOL_IP:
	break;
      case SOL_RAW:
	break;
      default:
	return -1;
    }

  return 0;
}

/*
 * Stop dataflow on a socket.
 */

static _SHUTDOWN(shutdown_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;

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
 * Following operations are not supported with RAW sockets.
 */

static _LISTEN(listen_raw) { fd->error = EOPNOTSUPP; return -1; }
static _ACCEPT(accept_raw) { fd->error = EOPNOTSUPP; return -1; }

/*
 * Socket API for RAW sockets.
 */

const struct socket_api_s	raw_socket =
  {
    .socket = socket_raw,
    .bind = bind_raw,
    .getsockname = getsockname_raw,
    .connect = connect_raw,
    .getpeername = getpeername_raw,
    .sendto = sendto_raw,
    .recvfrom = recvfrom_raw,
    .getsockopt = getsockopt_raw,
    .setsockopt = setsockopt_raw,
    .listen = listen_raw,
    .accept = accept_raw,
    .shutdown = shutdown_raw
  };

/*
 * Signal an incoming packet at level 3 layer.
 */

void		sock_raw_signal(struct net_proto_s	*addressing,
				struct net_packet_s	*packet,
				net_proto_id_t		protocol)
{
  /* deliver packet to all sockets matching interface and protocol id */
  CONTAINER_FOREACH(socket_raw, DLIST, NOLOCK, &sock_raw,
  {
    if (item->shutdown == SHUT_RD || item->shutdown == SHUT_RDWR)
      CONTAINER_FOREACH_CONTINUE;

    if (item->any || addressing->desc->f.addressing->matchaddr(addressing, &packet->tADDR, &item->local, NULL))
      {
	if (item->proto == protocol || item->proto == IPPROTO_RAW)
	  {
	    if (packet_queue_lock_pushback(&item->recv_q, packet))
	      sem_post(&item->recv_sem);
	  }
      }
  });
}
