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

#include <errno.h>

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
  pv->shutdown = -1;

  switch (domain)
    {
      case PF_INET:
	pv->family = AF_INET;
	break;
      case PF_INET6:
	pv->family = AF_INET6;
	break;
      default:
	errno = EPFNOSUPPORT;
	mem_free(fd);
	mem_free(pv);
	return NULL;
    }

  return fd;
}

/*
 * Set an UDP socket to listen on a given local address.
 */

static _BIND(bind_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  struct net_udp_addr_s		local;
  error_t			err;

  if (addr->sa_family != pv->family)
    {
      errno = fd->error = EAFNOSUPPORT;
      return -1;
    }

  switch (addr->sa_family)
    {
      case AF_INET:
	{
	  struct sockaddr_in	*in = (struct sockaddr_in *)addr;

	  if (len < sizeof (struct sockaddr_in))
	    {
	      errno = fd->error = EINVAL;
	      return -1;
	    }

	  IPV4_ADDR_SET(local.address, in->sin_addr.s_addr);
	  local.port = in->sin_port;
	}
	break;
      case AF_INET6:
	/* IPV6 */
      default:
	errno = fd->error = EAFNOSUPPORT;
	return -1;
    }

  err = udp_bind(&pv->desc, &local, socket_recv_callback, pv);

  if (err)
    {
      errno = fd->error = -err;
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
      errno = fd->error = -EINVAL; /* XXX check this behaviour */
      return -1;
    }

  switch (pv->desc->address.address.family)
    {
      case addr_ipv4:
	{
	  struct sockaddr_in	*in = (struct sockaddr_in *)addr;

	  if (*len < sizeof (struct sockaddr_in))
	    {
	      errno = fd->error = EINVAL;
	      return -1;
	    }

	  /* fill the address structure */
	  in->sin_family = AF_INET;
	  in->sin_port = htons(pv->desc->address.port);
	  in->sin_addr.s_addr = htonl(IPV4_ADDR_GET(pv->desc->address.address));

	  *len = sizeof (struct sockaddr_in);
	}
	break;
      default:
	errno = fd->error = EAFNOSUPPORT;
	return -1;
    }

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

  if (addr->sa_family != pv->family)
    {
      errno = fd->error = EAFNOSUPPORT;
      return -1;
    }

  switch (addr->sa_family)
    {
      case AF_INET:
	{
	  struct sockaddr_in	*in = (struct sockaddr_in *)addr;

	  if (len < sizeof (struct sockaddr_in))
	    {
	      errno = fd->error = EINVAL;
	      return -1;
	    }

	  IPV4_ADDR_SET(remote.address, in->sin_addr.s_addr);
	  remote.port = in->sin_port;
	}
	break;
      case AF_INET6:
	/* IPV6 */
      default:
	errno = fd->error = EAFNOSUPPORT;
	return -1;
    }

  err = udp_connect(&pv->desc, &remote);

  if (err)
    {
      errno = fd->error = -err;
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
      errno = fd->error = -EINVAL;
      return -1;
    }

  switch (pv->desc->remote.address.family)
    {
      case addr_ipv4:
	{
	  struct sockaddr_in	*in = (struct sockaddr_in *)addr;

	  if (*len < sizeof (struct sockaddr_in))
	    {
	      errno = fd->error = EINVAL;
	      return -1;
	    }

	  /* fill the address structure */
	  in->sin_family = AF_INET;
	  in->sin_port = htons(pv->desc->remote.port);
	  in->sin_addr.s_addr = htonl(IPV4_ADDR_GET(pv->desc->remote.address));

	  *len = sizeof (struct sockaddr_in);
	}
	break;
      default:
	errno = fd->error = EAFNOSUPPORT;
	return -1;
    }

  return 0;
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDTO(sendto_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  error_t			err;

  if (addr == NULL)
    {
      err = udp_send(pv->desc, NULL, buf, n);
    }
  else
    {
      struct net_udp_addr_s	remote;

      switch (addr->sa_family)
	{
	  case AF_INET:
	    {
	      struct sockaddr_in	*in = (struct sockaddr_in *)addr;

	      if (addr_len < sizeof (struct sockaddr_in))
		{
		  errno = fd->error = EINVAL;
		  return -1;
		}

	      IPV4_ADDR_SET(remote.address, in->sin_addr.s_addr);
	      remote.port = in->sin_port;
	    }
	    break;
	  case AF_INET6:
	    /* IPV6 */
	  default:
	    errno = fd->error = EAFNOSUPPORT;
	    return -1;
	}

      err = udp_send(pv->desc, &remote, buf, n);
    }

  if (err)
    {
      errno = fd->error = -err;
      return -1;
    }

  return n;
}

/*
 * Receive some data and get the source address.
 */

static _RECVFROM(recvfrom_udp)
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
  /* XXX */
  return -1;
}

/*
 * Set a socket option value.
 */

static _SETSOCKOPT(setsockopt_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Stop dataflow on a socket.
 */

static _SHUTDOWN(shutdown_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  if (how != SHUT_RDWR && how != SHUT_RD && how != SHUT_WR)
    {
      errno = fd->error = EINVAL;
      return -1;
    }

  /* check combinations */
  if (how == SHUT_RDWR || (pv->shutdown == SHUT_RD && how == SHUT_WR) ||
      (pv->shutdown == SHUT_WR && how == SHUT_RD))
    pv->shutdown = SHUT_RDWR;
  else
    pv->shutdown = how;

  /* close the descriptor if needed */
  if (pv->desc != NULL && pv->shutdown == SHUT_RDWR)
    {
      udp_close(pv->desc);
      pv->desc = NULL;
    }

  return -1;
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
    .sendto = sendto_udp,
    .recvfrom = recvfrom_udp,
    .getsockopt = getsockopt_udp,
    .setsockopt = setsockopt_udp,
    .listen = listen_udp,
    .accept = accept_udp,
    .shutdown = shutdown_udp
  };
