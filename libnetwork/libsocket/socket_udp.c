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

/*
 * Create an UDP socket. Allocate private data.
 */

static _SOCKET(socket_udp)
{
  struct socket_udp_pv_s	*pv;
  pv = fd->pv = mem_alloc(sizeof (struct socket_udp_pv_s), MEM_SCOPE_NETWORK);
  pv->desc = NULL;

  return fd;
}

/*
 * Set an UDP socket to listen on a given local address.
 */

static _BIND(bind_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;

  return -1;
}

/*
 * Get the socket local address.
 */

static _GETSOCKNAME(getsockname_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Bind an UDP socket to a remote address, so later sending operation may be faster.
 */

static _CONNECT(connect_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Get the remote address.
 */

static _GETPEERNAME(getpeername_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Send a chunk of data.
 */

static _SEND(send_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Receive a chunk of data.
 */

static _RECV(recv_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDTO(sendto_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Receive some data and get the source address.
 */

static _RECVFROM(recvfrom_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Send a message.
 */

static _SENDMSG(sendmsg_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Receive a message.
 */

static _RECVMSG(recvmsg_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
  return -1;
}

/*
 * Get a socket option value.
 */

static _GETSOCKOPT(getsockopt_udp)
{
  struct socket_udp_pv_s	*pv = (struct socket_udp_pv_s *)fd->pv;
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
    .send = send_udp,
    .recv = recv_udp,
    .sendto = sendto_udp,
    .recvfrom = recvfrom_udp,
    .sendmsg = sendmsg_udp,
    .recvmsg = recvmsg_udp,
    .getsockopt = getsockopt_udp,
    .setsockopt = setsockopt_udp,
    .listen = listen_udp,
    .accept = accept_udp,
    .shutdown = shutdown_udp
  };
