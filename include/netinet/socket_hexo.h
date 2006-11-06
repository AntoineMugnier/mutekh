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

#if !defined(NETINET_SOCKET_H) || defined(SOCKET_HEXO_H)
#error This file can not be included directly
#else

#define SOCKET_HEXO_H

/* a socket is a socket_s structure */
typedef struct socket_s *socket_t;

/* Create a new socket */
socket_t			socket(int domain, int type, int protocol);

/* Give the socket FD the local address ADDR (which is LEN bytes long).  */
#define bind(fd, ...)		(fd)->f->bind(fd, __VA_ARGS__)

/* Put the local address of FD into *ADDR and its length in *LEN.  */
#define getsockname(fd, ...)	(fd)->f->getsockname(fd, __VA_ARGS__)

/* Open a connection on socket FD to peer at ADDR (which LEN bytes long).
   For connectionless socket types, just set the default address to send to
   and the only address from which to accept transmissions.
   Return 0 on success, -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
#define connect(fd, ...)	(fd)->f->connect(fd, __VA_ARGS__)

/* Put the address of the peer connected to socket FD into *ADDR
   (which is *LEN bytes long), and its actual length into *LEN.  */
#define getpeername(fd, ...)	(fd)->f->getpeername(fd, __VA_ARGS__)

/* Send N bytes of BUF on socket FD to peer at address ADDR (which is
   ADDR_LEN bytes long).  Returns the number sent, or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
#define sendto(fd, ...)		(fd)->f->sendto(fd, __VA_ARGS__)

/* Read N bytes into BUF through socket FD.
   If ADDR is not NULL, fill in *ADDR_LEN bytes of it with tha address of
   the sender, and store the actual size of the address in *ADDR_LEN.
   Returns the number of bytes read or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
#define recvfrom(fd, ...)	(fd)->f->recvfrom(fd, __VA_ARGS__)

/* Put the current value for socket FD's option OPTNAME at protocol level LEVEL
   into OPTVAL (which is *OPTLEN bytes long), and set *OPTLEN to the value's
   actual length.  Returns 0 on success, -1 for errors.  */
#define getsockopt(fd, ...)	(fd)->f->getsockopt(fd, __VA_ARGS__)

/* Set socket FD's option OPTNAME at protocol level LEVEL
   to *OPTVAL (which is OPTLEN bytes long).
   Returns 0 on success, -1 for errors.  */
#define setsockopt(fd, ...)	(fd)->f->setsockopt(fd, __VA_ARGS__)

/* Prepare to accept connections on socket FD.
   N connection requests will be queued before further requests are refused.
   Returns 0 on success, -1 for errors.  */
#define listen(fd, ...)		(fd)->f->listen(fd, __VA_ARGS__)

/* Await a connection on socket FD.
   When a connection arrives, open a new socket to communicate with it,
   set *ADDR (which is *ADDR_LEN bytes long) to the address of the connecting
   peer and *ADDR_LEN to the address's actual length, and return the
   new socket's descriptor, or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
#define accept(fd, ...)		(fd)->f->accept(fd, __VA_ARGS__)

/* Shut down all or part of the connection open on socket FD.
   HOW determines what to shut down:
     SHUT_RD   = No more receptions;
     SHUT_WR   = No more transmissions;
     SHUT_RDWR = No more receptions or transmissions.
   Returns 0 on success, -1 for errors.  */
#define shutdown(fd, ...)	(fd)->f->shutdown(fd, __VA_ARGS__)

#endif
