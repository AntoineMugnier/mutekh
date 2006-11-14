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

/* Create a new socket of type TYPE in domain DOMAIN, using
   protocol PROTOCOL.  If PROTOCOL is zero, one is chosen automatically.
   Returns a file descriptor for the new socket, or NULL for errors.  */
socket_t			socket(int domain, int type, int protocol)
{
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
		    //return -EPROTONOSUPPORT;
		    return NULL;
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
		    //return -EPROTONOSUPPORT;
		    return NULL;
		}
	      break;
#ifdef CONFIG_NETWORK_SOCKET_RAW
	    /* Raw packets to a given protocol */
	    case SOCK_RAW:
	      api = &raw_socket;
	      break;
#endif
	    default:
	      //return -EPROTONOSUPPORT;
	      return NULL;
	  }
	break;
#ifdef CONFIG_NETWORK_SOCKET_PACKET
      /* Packet sockets, used to write Layer 2 protocols */
      case PF_PACKET:
	api = &packet_socket;
	break;
#endif
      default:
	//return -EPFNOSUPPORT;
	return NULL;
    }
  if ((sock = mem_alloc(sizeof (struct socket_s), MEM_SCOPE_NETWORK)) == NULL)
    //return -ENOMEM;
    return NULL;
  /* setup common fields to their defaults */
  sock->error = 0;
  sock->shutdown = -1;
  sock->type = type;
  sock->broadcast = 0;
  sock->keepalive = 0;
  sock->recv_timeout = 0;
  sock->send_timeout = 0;
  sock->f = api;
  api->socket(sock, domain, type, protocol);
  return sock;
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
      fd->error = ENOMEM;
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
  ret = fd->f->sendto(fd, buf, n, flags, message->msg_name, message->msg_namelen, message);
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
      fd->error = ENOMEM;
      return -1;
    }

  /* receive the data */
  ret = fd->f->recvfrom(fd, buf, n, flags, message->msg_name, &message->msg_namelen, message);

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

/*
 * Setsock opt, SOL_SOCKET level.
 */

int setsockopt_socket(socket_t		fd,
		      int		optname,
		      const void	*optval,
		      socklen_t		optlen)
{
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
	  fd->recv_timeout = tv->tv_sec * 1000 + tv->tv_usec / 1000;
	}
	break;
      /* send timeout */
      case SO_SNDTIMEO:
	{
	  const struct timeval	*tv;

	  if (optlen < sizeof (struct timeval))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  tv = optval;
	  fd->send_timeout = tv->tv_sec * 1000 + tv->tv_usec / 1000;
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
	  fd->broadcast = *enable;
	}
	break;
      /* allow keepalive packets */
      case SO_KEEPALIVE:
	{
	  const int		*enable;

	  if (optlen < sizeof (int))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  enable = optval;
	  fd->keepalive = *enable;
	}
	break;
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}

/*
 * Getsock opt, SOL_SOCKET level.
 */

int getsockopt_socket(socket_t	fd,
		      int	optname,
		      void	*optval,
		      socklen_t	*optlen)
{
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
	  tv->tv_usec = (fd->recv_timeout % 1000) * 1000;
	  tv->tv_sec = fd->recv_timeout / 1000;

	  *optlen = sizeof (struct timeval);
	}
	break;
      /* send timeout */
      case SO_SNDTIMEO:
	{
	  struct timeval	*tv;

	  if (*optlen < sizeof (struct timeval))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  tv = optval;
	  tv->tv_usec = (fd->send_timeout % 1000) * 1000;
	  tv->tv_sec = fd->send_timeout / 1000;

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
	  *enabled = fd->broadcast;

	  *optlen = sizeof (int);
	}
	break;
      /* keepalive enabled */
      case SO_KEEPALIVE:
	{
	  int			*enabled;

	  if (*optlen < sizeof (int))
	    {
	      fd->error = EINVAL;
	      return -1;
	    }

	  enabled = optval;
	  *enabled = fd->keepalive;

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
	  *type = fd->type;

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
 * Shutdown
 */

_SHUTDOWN(shutdown_socket)
{
  if (how != SHUT_RDWR && how != SHUT_RD && how != SHUT_WR)
    {
      fd->error = EINVAL;
      return -1;
    }

  /* check combinations */
  if (how == SHUT_RDWR || (fd->shutdown == SHUT_RD && how == SHUT_WR) ||
      (fd->shutdown == SHUT_WR && how == SHUT_RD))
    fd->shutdown = SHUT_RDWR;
  else
    fd->shutdown = how;

  return 0;
}

/*
 * Setsock opt, SOL_IP level.
 */

int setsockopt_inet(socket_t	fd,
		    int		optname,
		    const void	*optval,
		    socklen_t	optlen)
{
  switch (optname)
    {
      /* XXX SOL_IP */
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}

/*
 * Getsock opt, SOL_SOCKET level.
 */

int getsockopt_inet(socket_t	fd,
		    int		optname,
		    void	*optval,
		    socklen_t	*optlen)
{
  switch (optname)
    {
      /* XXX SOL_IP */
      default:
	fd->error = ENOPROTOOPT;
	return -1;
    }

  return 0;
}
