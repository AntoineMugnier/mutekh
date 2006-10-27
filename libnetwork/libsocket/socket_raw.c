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
#include <netinet/socket_raw.h>

#include <hexo/alloc.h>

/*
 * Create a RAW socket. Allocate private data.
 */

static _SOCKET(socket_raw)
{
  struct socket_raw_pv_s	*pv;
  pv = fd->pv = mem_alloc(sizeof (struct socket_raw_pv_s), MEM_SCOPE_NETWORK);

  return fd;
}

/*
 * Set a RAW socket to listen on a given local address.
 */

static _BIND(bind_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Get the socket local address.
 */

static _GETSOCKNAME(getsockname_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Connect a RAW socket to a remote address.
 */

static _CONNECT(connect_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Get the remote address.
 */

static _GETPEERNAME(getpeername_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Send a chunk of data.
 */

static _SEND(send_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Receive a chunk of data.
 */

static _RECV(recv_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Send some data specifiyng explicitely the destination.
 */

static _SENDTO(sendto_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Receive some data and get the source address.
 */

static _RECVFROM(recvfrom_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
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
  return -1;
}

/*
 * Set a socket option value.
 */

static _SETSOCKOPT(setsockopt_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
}

/*
 * Stop dataflow on a socket.
 */

static _SHUTDOWN(shutdown_raw)
{
  struct socket_raw_pv_s	*pv = (struct socket_raw_pv_s *)fd->pv;
  return -1;
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
