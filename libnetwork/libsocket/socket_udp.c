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

#include <netinet/packet.h>
#include <netinet/socket.h>
#include <netinet/socket_udp.h>
#include <netinet/libudp.h>
#include <netinet/udp.h>

#include <hexo/alloc.h>

static UDP_CALLBACK(socket_recv_callback)
{

}

static UDP_ERROR_CALLBACK(socket_err_callback)
{

}

/*
 * Create an UDP socket. Allocate private data.
 */

static _SOCKET(socket_udp)
{
  struct socket_udp_pv_s	*pv;
  pv = fd->pv = mem_alloc(sizeof (struct socket_udp_pv_s), MEM_SCOPE_NETWORK);
  pv->desc = NULL;

  switch (domain)
    {
      case PF_INET:
	pv->family = AF_INET;
	break;
      case PF_INET6:
	pv->family = AF_INET6;
	break;
      default:
	mem_free(pv);
	return -EPFNOSUPPORT;
    }

  return 0;
}

/*
 * Set an UDP socket to listen on a given local address.
 */

static _BIND(bind_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  struct net_udp_addr_s		local;
  error_t			err;
  net_port_t			port;

  if (addr->sa_family != pv->family)
    {
      fd->error = EAFNOSUPPORT;
      return -1;
    }

  if (socket_in_addr(fd, &local.address, addr, len, &port))
    return -1;

  local.port = ntohs(port);

  err = udp_bind(&pv->desc, &local, socket_recv_callback, pv);

  if (err)
    {
      fd->error = -err;
      return -1;
    }

  return 0;
}

/*
 * Get the socket local address.
 */

static _GETSOCKNAME(getsockname_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  if (pv->desc == NULL || !pv->desc->bound)
    {
      fd->error = -EINVAL; /* XXX check this behaviour */
      return -1;
    }

  if (socket_addr_in(fd, &pv->desc->address, addr, len, htons(pv->desc->port)))
    return -1;

  return 0;
}

/*
 * Bind an UDP socket to a remote address, so later sending operation may be faster.
 */

static _CONNECT(connect_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  struct net_udp_addr_s		remote;
  error_t			err;
  net_port_t			port;

  if (addr->sa_family != pv->family)
    {
      fd->error = EAFNOSUPPORT;
      return -1;
    }

  if (socket_in_addr(fd, &remote.address, addr, len, &port))
    return -1;

  remote.port = ntohs(port);

  err = udp_connect(&pv->desc, &remote);

  if (err)
    {
      fd->error = -err;
      return -1;
    }

  return 0;
}

/*
 * Get the remote address.
 */

static _GETPEERNAME(getpeername_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  if (pv->desc == NULL || !pv->desc->connected)
    {
      fd->error = -EINVAL;
      return -1;
    }

  if (socket_addr_in(fd, &pv->desc->remote.address, addr, len, htons(pv->desc->remote.port)))
    return -1;

  return 0;
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDMSG(sendmsg_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  struct sockaddr		*addr;
  error_t			err;
  uint8_t			*buf;
  size_t			n;
  size_t			i;

  if (message == NULL)
    {
      fd->error = EINVAL;
      return -1;
    }

  addr = message->msg_name;

  /* build the packet */
  if (message->msg_iovlen == 1)
    {
      buf = message->msg_iov[0].iov_base;
      n = message->msg_iov[0].iov_len;
    }
  else
    {
      for (i = 0, n = 0; i < message->msg_iovlen; i++)
	n += message->msg_iov[i].iov_len;

      if ((buf = mem_alloc(n, MEM_SCOPE_NETWORK)) == NULL)
	{
	  fd->error = ENOMEM;
	  return -1;
	}
    }

  /* send it */
  if (addr == NULL)
    {
      err = udp_send(pv->desc, NULL, buf, n);
    }
  else
    {
      struct net_udp_addr_s	remote;
      net_port_t		port;

      if (socket_in_addr(fd, &remote.address, addr, message->msg_namelen, &port))
	{
	  if (message->msg_iovlen != 1)
	    mem_free(buf);
	  return -1;
	}

      remote.port = ntohs(port);

      err = udp_send(pv->desc, &remote, buf, n);
    }

  if (message->msg_iovlen != 1)
    mem_free(buf);

  if (err)
    {
      fd->error = -err;
      return -1;
    }

  return n;
}

/*
 * Receive some data and get the source address.
 */

static _RECVMSG(recvmsg_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  /* XXX */
  return -1;
}

/*
 * Get a socket option value.
 */

static _GETSOCKOPT(getsockopt_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  switch (level)
    {
      case SOL_SOCKET:
	return getsockopt_socket(fd, optname, optval, optlen);
      case SOL_IP:
	return getsockopt_inet(fd, optname, optval, optlen);
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}

/*
 * Set a socket option value.
 */

static _SETSOCKOPT(setsockopt_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  switch (level)
    {
      case SOL_SOCKET:
	return setsockopt_socket(fd, optname, optval, optlen);
      case SOL_IP:
	return setsockopt_inet(fd, optname, optval, optlen);
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}

/*
 * Stop dataflow on a socket.
 */

static _SHUTDOWN(shutdown_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  if (shutdown_socket(fd, how))
    return -1;

  /* close the descriptor if needed */
  if (pv->desc != NULL && fd->shutdown == SHUT_RDWR)
    {
      udp_close(pv->desc);
      pv->desc = NULL;
    }

  return 0;
}

/*
 * Following operations are not supported in UDP.
 */

static _LISTEN(listen_udp) { return -1; }
static _ACCEPT(accept_udp) { return -1; }

/*
 * Socket API for UDP datagrams.
 */

const struct socket_api_s	udp_socket =
  {
    .socket = socket_udp,
    .bind = bind_udp,
    .getsockname = getsockname_udp,
    .connect = connect_udp,
    .getpeername = getpeername_udp,
    .sendmsg = sendmsg_udp,
    .recvmsg = recvmsg_udp,
    .getsockopt = getsockopt_udp,
    .setsockopt = setsockopt_udp,
    .listen = listen_udp,
    .accept = accept_udp,
    .shutdown = shutdown_udp
  };
