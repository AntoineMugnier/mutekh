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
#include <netinet/ip.h>
#include <netinet/icmp.h>
#include <netinet/route.h>
#include <netinet/arp.h>

#include <hexo/alloc.h>

#include <semaphore.h>

static socket_table_root_t	sock_raw = CONTAINER_ROOT_INITIALIZER(socket_table, DLIST, NOLOCK);

/*
 * Receive timeout callback.
 */

static TIMER_CALLBACK(recv_timeout)
{
  socket_t			fd = (socket_t)pv;
  struct socket_raw_pv_s	*pv_packet = (struct socket_raw_pv_s *)fd->pv;

  sem_post(&pv_packet->recv_sem);
}

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
  pv->icmp_mask = 0;
  sem_init(&pv->recv_sem, 0, 0);
  packet_queue_lock_init(&pv->recv_q);

  /* determine if headers must be included or not */
  pv->header = (protocol == IPPROTO_RAW);
  if (!socket_table_push(&sock_raw, fd))
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

  if (!any)
    {
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
  struct net_addr_s		dest;
  struct net_route_s		*route;

  if (pv->connected)
    {
      fd->error = EISCONN;
      return -1;
    }

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
  pv->route = route;
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
 * Send a message
 */

static _SENDMSG(sendmsg_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct sockaddr		*addr;
  struct net_packet_s		*packet;
  struct net_if_s		*interface;
  struct net_proto_s		*addressing;
  struct net_header_s		*nethdr;
  struct net_route_s		*route = NULL;
  uint8_t			*p;
  size_t			i;
  size_t			n;

  if (flags & MSG_OOB || flags & MSG_TRUNC)
    {
      fd->error = EOPNOTSUPP;
      return -1;
    }

  if (fd->shutdown == SHUT_WR || fd->shutdown == SHUT_RDWR)
    {
      fd->error = ESHUTDOWN;
      return -1;
    }

  if (message == NULL)
    {
      fd->error = EINVAL;
      return -1;
    }

  if ((packet = packet_obj_new(NULL)) == NULL)
    {
      fd->error = ENOMEM;
      return -1;
    }

  addr = message->msg_name;

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

      if (socket_in_addr(fd, &packet->tADDR, addr, message->msg_namelen, NULL))
	{
	  packet_obj_refdrop(packet);
	  return -1;
	}
    }

  /* now, deduce the source */
  if (pv->connected)
    {
      if (pv->route->invalidated)
	{
	  fd->error = EPIPE;
	  return -1;
	}

      /* simply read the pv info */
      interface = pv->route->interface;
      addressing = pv->route->addressing;
    }
  else
    {
      /* otherwise, determine the route */
      if ((route = route_get(&packet->tADDR)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = EHOSTUNREACH;
	  return -1;
	}
      interface = route->interface;
      addressing = route->addressing;
    }

  /* compute buffer length */
  for (i = 0, n = 0; i < message->msg_iovlen; i++)
    n += message->msg_iov[i].iov_len;

  /* prepare the packet */
  if (pv->header)
    {
      if ((p = if_preparepkt(interface, packet, n, 0)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = ENOMEM;
	  goto error;
	}
    }
  else
    {
      if ((p = addressing->desc->preparepkt(interface, packet, n, 0)) == NULL)
	{
	  packet_obj_refdrop(packet);
	  fd->error = ENOMEM;
	  goto error;
	}
    }
  nethdr = &packet->header[packet->stage];
  nethdr->data = p;
  nethdr->size = n;
  nethdr[1].data = NULL;

  /* copy the buffer into the packet */
  for (i = 0, n = 0; i < message->msg_iovlen; n += message->msg_iov[i].iov_len, i++)
    memcpy(p + n, message->msg_iov[i].iov_base, message->msg_iov[i].iov_len);

  /* adjust some IP header fields */
  if (pv->header)
    {
      switch (pv->family)
	{
	  case AF_INET:
	    {
	      struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)addressing->pv;
	      struct iphdr		*ip;

	      ip = (struct iphdr *)p;
	      if (net_32_load(ip->saddr) == 0)
		net_be32_store(ip->saddr, pv_ip->addr);
	      if (net_16_load(ip->id) == 0)
		net_be16_store(ip->id, pv_ip->id_seq++);
	      net_be16_store(ip->tot_len, n);
	      net_16_store(ip->check, 0);
	      net_16_store(ip->check, ~packet_checksum(p, ip->ihl * 4));
	    }
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

  if (route != NULL)
    route_obj_refdrop(route);

  return n;

 error:
  if (route != NULL)
    route_obj_refdrop(route);

  return -1;
}

/*
 * Receive a message
 */

static _RECVMSG(recvmsg_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  struct sockaddr		*addr;
  struct net_packet_s		*packet;
  ssize_t			sz;
  struct net_header_s		*nethdr;
  timer_delay_t			start;
  struct timer_event_s		timeout;
  bool_t			timeout_started = 0;

  if (flags & MSG_OOB || flags & MSG_TRUNC)
    {
      fd->error = EOPNOTSUPP;
      return -1;
    }

  if (message == NULL)
    {
      fd->error = EINVAL;
      return -1;
    }

  start = timer_get_tick(&timer_ms);

  /* try to grab a packet */
 again:
  if (fd->shutdown == SHUT_RD || fd->shutdown == SHUT_RDWR)
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

      /* if there is a receive timeout, start a timer */
      if (!timeout_started && fd->recv_timeout)
	{
	  timeout.callback = recv_timeout;
	  timeout.delay = fd->recv_timeout;
	  timeout.pv = fd;
	  timeout_started = 1;
	  timer_add_event(&timer_ms, &timeout);
	}

      sem_wait(&pv->recv_sem);

      /* has timeout expired ? */
      if (timeout_started)
	if (timer_get_tick(&timer_ms) - start >= (fd->recv_timeout * 5) / 6)
	  {
	    fd->error = EAGAIN;
	    return -1;
	  }

      goto again;
    }

  timer_cancel_event(&timeout, 0);

  addr = message->msg_name;

  /* fill the address if required */
  if (addr != NULL)
    {
      if (socket_addr_in(fd, &packet->sADDR, addr, &message->msg_namelen, htons(pv->proto)))
	{
	  packet_obj_refdrop(packet);
	  return -1;
	}
    }

  /* copy the data */
  if (pv->header)
    {
      nethdr = &packet->header[packet->stage - 1];
      /* XXX */
    }
  else
    {
      size_t	i;
      size_t	chunksz;

      nethdr = &packet->header[packet->stage];
      for (i = 0, sz = 0; i < message->msg_iovlen; i++, sz += chunksz)
	{
	  chunksz = message->msg_iov[i].iov_len;
	  if (sz + chunksz > nethdr->size)
	    {
	      chunksz = nethdr->size - sz;
	      memcpy(message->msg_iov[i].iov_base, nethdr->data + sz, chunksz);
	      sz += chunksz;
	      break;
	    }
	  else
	    memcpy(message->msg_iov[i].iov_base, nethdr->data + sz, chunksz);
	}
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

  switch (level)
    {
      case SOL_SOCKET:
	return getsockopt_socket(fd, optname, optval, optlen);
      case SOL_IP:
	if (optname == IP_HDRINCL)
	  {
	    int_fast32_t	*enabled;

	    if (*optlen < sizeof (int_fast32_t))
	      {
		fd->error = EINVAL;
		return -1;
	      }

	    enabled = optval;
	    *enabled = pv->header;

	    *optlen = sizeof (int_fast32_t);
	  }
	else
	  return getsockopt_inet(fd, optname, optval, optlen);
	break;
      case SOL_RAW:
	if (optname == ICMP_FILTER)
	  {
	    struct icmp_filter	*filt;

	    if (pv->proto != IPPROTO_ICMP)
	      {
		fd->error = EOPNOTSUPP;
		return -1;
	      }

	    if (*optlen < sizeof (struct icmp_filter))
	      {
		fd->error = EINVAL;
		return -1;
	      }

	    filt = optval;
	    filt->data = pv->icmp_mask;

	    *optlen = sizeof (struct icmp_filter);
	    break;
	  }
      default:
	fd->error = ENOPROTOOPT;
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

  switch (level)
    {
      case SOL_SOCKET:
	if (optname == SO_BINDTODEVICE)
	  {
	    struct net_if_s	*interface;

	    if ((interface = if_get_by_name((char *)optval)) == NULL)
	      {
		fd->error = EINVAL;
		return -1;
	      }

	    pv->local_interface = interface;
	    pv->any = 0;
	  }
	else
	  return setsockopt_socket(fd, optname, optval, optlen);
      case SOL_IP:
	if (optname == IP_HDRINCL)
	  {
	    const int_fast32_t	*enable;

	    if (optlen < sizeof (int_fast32_t))
	      {
		fd->error = EINVAL;
		return -1;
	      }

	    enable = optval;
	    pv->header = *enable;
	  }
	else
	  return setsockopt_inet(fd, optname, optval, optlen);
	break;
      case SOL_RAW:
	if (optname == ICMP_FILTER)
	  {
	    const struct icmp_filter	*filt;

	    if (pv->proto != IPPROTO_ICMP)
	      {
		fd->error = EOPNOTSUPP;
		return -1;
	      }

	    if (optlen < sizeof (struct icmp_filter))
	      {
		fd->error = EINVAL;
		return -1;
	      }

	    filt = optval;
	    pv->icmp_mask = filt->data;
	  }
      default:
	fd->error = ENOPROTOOPT;
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

  if (shutdown_socket(fd, how))
    return -1;

  /* end all the recv with errors */
  if (fd->shutdown == SHUT_RDWR || fd->shutdown == SHUT_RD)
    {
      uint_fast8_t		val;

      /* drop all waiting packets */
      packet_queue_lock_clear(&pv->recv_q);

      sem_getvalue(&pv->recv_sem, &val);
      while (val < 0)
	{
	  sem_post(&pv->recv_sem);
	  sem_getvalue(&pv->recv_sem, &val);
	}

      if (fd->shutdown == SHUT_RDWR)
	{
	  socket_table_remove(&sock_raw, fd);

	  if (pv->connected)
	    route_obj_refdrop(pv->route);
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
    .sendmsg = sendmsg_raw,
    .recvmsg = recvmsg_raw,
    .getsockopt = getsockopt_raw,
    .setsockopt = setsockopt_raw,
    .listen = listen_raw,
    .accept = accept_raw,
    .shutdown = shutdown_raw
  };

/*
 * Signal an incoming packet at level 3 layer.
 */

void		sock_raw_signal(struct net_if_s		*interface,
				struct net_proto_s	*addressing,
				struct net_packet_s	*packet,
				net_proto_id_t		protocol)
{
  bool_t	is_bcast;

  /* determine is broadcast destination */
  switch (packet->tADDR.family)
    {
      case addr_ipv4:
	{
	  struct net_pv_ip_s	*pv_ip = (struct net_pv_ip_s *)addressing->pv;

	  is_bcast = (packet->tADDR.addr.ipv4 & ~pv_ip->mask) == ~pv_ip->mask;
	}
	break;
      default:
	/* IPV6 */
	is_bcast = 0;
	break;
    }

  /* deliver packet to all sockets matching interface and protocol id */
  CONTAINER_FOREACH(socket_table, DLIST, NOLOCK, &sock_raw,
  {
    struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)item->pv;

    if (item->shutdown == SHUT_RD || item->shutdown == SHUT_RDWR)
      CONTAINER_FOREACH_CONTINUE;

    if (pv->any || addressing->desc->f.addressing->matchaddr(addressing, &packet->tADDR, &pv->local, NULL) || pv->local_interface == interface)
      {
	if (pv->proto == protocol || pv->proto == IPPROTO_RAW)
	  {
	    if (!item->broadcast && is_bcast)
	      CONTAINER_FOREACH_CONTINUE;

	    /* do ICMP type filtering */
	    if (protocol == IPPROTO_ICMP)
	      {
		struct net_header_s	*nethdr;

		nethdr = &packet->header[packet->stage];
		if (nethdr->size >= sizeof (struct icmphdr))
		  {
		    struct icmphdr	*icmp;

		    icmp = (struct icmphdr *)nethdr->data;
		    if (pv->icmp_mask & (1 << icmp->type))
		      CONTAINER_FOREACH_CONTINUE;
		  }
	      }

	    if (packet_queue_lock_pushback(&pv->recv_q, packet))
	      sem_post(&pv->recv_sem);
	  }
      }
  });
}
