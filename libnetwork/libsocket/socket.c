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

#include <netinet/socket.h>

#include <errno.h>

/* Create a new socket of type TYPE in domain DOMAIN, using
   protocol PROTOCOL.  If PROTOCOL is zero, one is chosen automatically.
   Returns a file descriptor for the new socket, or -1 for errors.  */
socket_t			socket(int domain, int type, int protocol)
{
#if defined(CONFIG_NETWORK_SOCKET_HEXO)
  const socket_t		error = NULL;
#elif defined(CONFIG_NETWORK_SOCKET_POSIX)
  const socket_t		error = -1;
#else
# error Neither CONFIG_NETWORK_SOCKET_HEXO nor CONFIG_NETWORK_SOCKET_POSIX are defined.
#endif

  const struct socket_api_s	*api;
  socket_t			sock;

  switch (domain)
    {
      /* Internet sockets */
      case PF_INET:
      case PF_INET6:
	switch (type)
	  {
	    case SOCK_DGRAM:
	      switch (protocol)
		{
#ifdef CONFIG_NETWORK_UDP
		  /* UDP is the default DGRAM protocol */
		  case IPPROTO_UDP:
		  case 0:
		    api = &udp_socket;
		    break;
#endif
		  default:
		    errno = EPROTONOSUPPORT;
		    return error;
		}
	      break;
	    case SOCK_STREAM:
	      switch (protocol)
		{
#ifdef CONFIG_NETWORK_TCP
		  /* UDP is the default STREAM protocol */
		  case IPPROTO_TCP:
		  case 0:
		    api = &tcp_socket;
		    break;
#endif
		  default:
		    errno = EPROTONOSUPPORT;
		    return error;
		}
	      break;
#ifdef CONFIG_NETWORK_SOCKET_RAW
	    /* Raw packets to a given protocol */
	    case SOCK_RAW:
	      api = &raw_socket;
	      break;
#endif
	    default:
	      errno = EPROTONOSUPPORT;
	      return error;
	  }
	break;
#ifdef CONFIG_NETWORK_SOCKET_PACKET
      /* Packet sockets, used to write Layer 2 protocols */
      case PF_PACKET:
	api = &packet_socket;
	break;
#endif
      default:
	errno = EPFNOSUPPORT;
	return error;
    }
  if ((sock = mem_alloc(sizeof (struct socket_s), MEM_SCOPE_NETWORK)) == NULL)
    {
      errno = ENOMEM;
      return -1;
    }
  sock->error = 0;
  sock->f = api;
#if defined(CONFIG_NETWORK_SOCKET_HEXO)
  return api->socket(sock, domain, type, protocol);
#elif defined(CONFIG_NETWORK_SOCKET_POSIX)
  /* XXX */
#else
# error Neither CONFIG_NETWORK_SOCKET_HEXO nor CONFIG_NETWORK_SOCKET_POSIX are defined.
#endif
}

/*
 * Receive a chunk of data.
 */

_RECV(recv)
{
  return recvfrom(fd, buf, n, flags, NULL, NULL);
}

/*
 * Send some data.
 */

_SEND(send)
{
  return sendto(fd, buf, n, flags, NULL, 0);
}

/*
 * Send a message.
 */

_SENDMSG(sendmsg)
{
  ssize_t	ret;
  uint8_t	*buf;
  size_t	n = 0;
  size_t	i;

  /* determine the total size */
  for (i = 0; i < message->msg_iovlen; i++)
    n += message->msg_iov[i].iov_len;

  /* allocate a buffer large enough */
  if ((buf = mem_alloc(n, MEM_SCOPE_SYS)) == NULL)
    {
      fd->error = errno = ENOMEM;
      return -1;
    }

  /* build the buffer */
  for (i = 0, n = 0; i < message->msg_iovlen; i++)
    {
      struct iovec	*v = &message->msg_iov[i];
      memcpy(buf + n, v->iov_base, v->iov_len);
      n += v->iov_len;
    }

  /* send & free */
  ret = sendto(fd, buf, n, flags, message->msg_name, message->msg_namelen);
  mem_free(buf);

  return ret;
}

/*
 * Receive a message.
 */

_RECVMSG(recvmsg)
{
  ssize_t	ret;
  uint8_t	*buf;
  size_t	n = 0;
  size_t	i;

  /* determine the total size */
  for (i = 0; i < message->msg_iovlen; i++)
    n += message->msg_iov[i].iov_len;

  /* allocate a buffer large enough */
  if ((buf = mem_alloc(n, MEM_SCOPE_SYS)) == NULL)
    {
      fd->error = errno = ENOMEM;
      return -1;
    }

  /* receive the data */
  ret = recvfrom(fd, buf, n, flags, message->msg_name, &message->msg_namelen);

  /* pack into multiple vectors */
  for (i = 0, n = 0; i < message->msg_iovlen; i++)
    {
      struct iovec	*v = &message->msg_iov[i];

      if (n + v->iov_len > ret)
	memcpy(v->iov_base, buf + n, ret - n);
      else
	memcpy(v->iov_base, buf + n, v->iov_len);
      n += v->iov_len;
    }

  mem_free(buf);

  return ret;
}
