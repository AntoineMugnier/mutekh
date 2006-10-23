/* System-specific socket constants and types.  Linux version.
   Copyright (C) 1991,1992,1994-2001, 2004 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

#ifndef NETINET_SOCKET_H
#define NETINET_SOCKET_H

#include <hexo/types.h>
#include <netinet/sockaddr.h>
#include <netinet/in.h>

/* Types of sockets.  */
#define SOCK_STREAM	1	/* Sequenced, reliable, connection-based
				   byte streams.  */
#define SOCK_DGRAM	2	/* Connectionless, unreliable datagrams
				   of fixed maximum length.  */
#define SOCK_RAW	3	/* Raw protocol interface.  */

/* Protocol families.  */
#define	PF_UNSPEC	0	/* Unspecified.  */
#define	PF_INET		2	/* IP protocol family.  */
#define	PF_INET6	10	/* IP version 6.  */
#define	PF_NETLINK	16
#define	PF_ROUTE	PF_NETLINK /* Alias to emulate 4.4BSD.  */
#define	PF_PACKET	17	/* Packet family.  */
#define	PF_MAX		32	/* For now..  */

/* Address families.  */
#define	AF_UNSPEC	PF_UNSPEC
#define	AF_INET		PF_INET
#define	AF_INET6	PF_INET6
#define	AF_NETLINK	PF_NETLINK
#define	AF_ROUTE	PF_ROUTE
#define	AF_PACKET	PF_PACKET
#define	AF_MAX		PF_MAX

/* Socket level values */
#define SOL_IP		0
#define SOL_SOCKET	1
#define SOL_TCP		6	/* TCP level */
#define SOL_UDP		17      /* sockopt level for UDP */
#define SOL_RAW		255
#define SOL_PACKET	263

/* For setsockopt(2) */
#define SO_DEBUG	1
#define SO_REUSEADDR	2
#define SO_TYPE		3
#define SO_ERROR	4
#define SO_DONTROUTE	5
#define SO_BROADCAST	6
#define SO_SNDBUF	7
#define SO_RCVBUF	8
#define SO_KEEPALIVE	9
#define SO_OOBINLINE	10
#define SO_NO_CHECK	11
#define SO_PRIORITY	12
#define SO_LINGER	13
#define SO_BSDCOMPAT	14
#define SO_REUSEPORT	15
#define SO_PASSCRED	16
#define SO_PEERCRED	17
#define SO_RCVLOWAT	18
#define SO_SNDLOWAT	19
#define SO_RCVTIMEO	20
#define SO_SNDTIMEO	21

/* Security levels - as per NRL IPv6 - don't actually do anything */
#define SO_SECURITY_AUTHENTICATION		22
#define SO_SECURITY_ENCRYPTION_TRANSPORT	23
#define SO_SECURITY_ENCRYPTION_NETWORK		24

#define SO_BINDTODEVICE	25

/* Socket filtering */
#define SO_ATTACH_FILTER        26
#define SO_DETACH_FILTER        27

#define SO_PEERNAME		28
#define SO_TIMESTAMP		29
#define SCM_TIMESTAMP		SO_TIMESTAMP

#define SO_ACCEPTCONN		30

#define SO_PEERSEC		31

/* Maximum queue length specifiable by listen.  */
#define SOMAXCONN	128

/* IP level options (for setsockopt) */
/* Options for use with `getsockopt' and `setsockopt' at the IP level.
   The first word in the comment at the right is the data type used;
   "bool" means a boolean value stored in an `int'.  */
#define IP_OPTIONS		4       /* ip_opts; IP per-packet options.  */
#define IP_HDRINCL		3       /* int; Header is included with data.  */
#define IP_TOS			1       /* int; IP type of service and precedence.  */
#define IP_TTL			2       /* int; IP time to live.  */
#define IP_RECVOPTS		6       /* bool; Receive all IP options w/datagram.  */
/* For BSD compatibility.  */
#define IP_RECVRETOPTS		IP_RETOPTS       /* bool; Receive IP options for response.  */
#define IP_RETOPTS		7       /* ip_opts; Set/get IP per-packet options.  */
#define IP_MULTICAST_IF		32	/* in_addr; set/get IP multicast i/f */
#define IP_MULTICAST_TTL	33	/* u_char; set/get IP multicast ttl */
#define IP_MULTICAST_LOOP	34	/* i_char; set/get IP multicast loopback */
#define IP_ADD_MEMBERSHIP	35	/* ip_mreq; add an IP group membership */
#define IP_DROP_MEMBERSHIP	36	/* ip_mreq; drop an IP group membership */
#define IP_UNBLOCK_SOURCE	37	/* ip_mreq_source: unblock data from source */
#define IP_BLOCK_SOURCE		38	/* ip_mreq_source: block data from source */
#define IP_ADD_SOURCE_MEMBERSHIP	39 /* ip_mreq_source: join source group */
#define IP_DROP_SOURCE_MEMBERSHIP	40 /* ip_mreq_source: leave source group */
#define IP_MSFILTER		41
#define IP_ROUTER_ALERT		5	/* bool */
#define IP_PKTINFO		8	/* bool */
#define IP_PKTOPTIONS		9
#define IP_PMTUDISC		10	/* obsolete name? */
#define IP_MTU_DISCOVER		10	/* int; see below */
#define IP_RECVERR		11	/* bool */
#define IP_RECVTTL		12	/* bool */
#define IP_RECVTOS		13	/* bool */


/* IP_MTU_DISCOVER arguments.  */
#define IP_PMTUDISC_DONT	0	/* Never send DF frames.  */
#define IP_PMTUDISC_WANT	1	/* Use per route hints.  */
#define IP_PMTUDISC_DO		2	/* Always DF.  */

/* IP multicast values */
#define IP_DEFAULT_MULTICAST_TTL	1
#define IP_DEFAULT_MULTICAST_LOOP	1
#define IP_MAX_MEMBERSHIPS		20

/* Structure used to describe IP options for IP_OPTIONS and IP_RETOPTS.
   The `ip_dst' field is used for the first-hop gateway when using a
   source route (this gets put into the header proper).  */
struct ip_opts
  {
    struct in_addr ip_dst;	/* First hop; zero without source route.  */
    char ip_opts[40];		/* Actually variable in size.  */
  };

/* Like `struct ip_mreq' but including interface specification by index.  */
struct ip_mreqn
  {
    struct in_addr imr_multiaddr;	/* IP multicast address of group */
    struct in_addr imr_address;		/* local IP address of interface */
    int	imr_ifindex;			/* Interface index */
  };

/* Structure used for IP_PKTINFO.  */
struct in_pktinfo
  {
    int ipi_ifindex;			/* Interface index  */
    struct in_addr ipi_spec_dst;	/* Routing destination address  */
    struct in_addr ipi_addr;		/* Header destination address  */
  };

/* The following constants should be used for the second parameter of
   `shutdown'.  */
#define SHUT_RD		0	/* No more receptions.  */
#define SHUT_WR		1	/* No more transmissions.  */
#define SHUT_RDWR	2	/* No more receptions or transmissions.  */

/* Bits in the FLAGS argument to `send', `recv', et al.  */
#define MSG_OOB		0x01	/* Process out-of-band data.  */
#define MSG_PEEK	0x02	/* Peek at incoming messages.  */
#define MSG_DONTROUTE	0x04	/* Don't use local routing.  */
#define MSG_CTRUNC	0x08	/* Control data lost before delivery.  */
#define MSG_PROXY	0x10	/* Supply or ask second address.  */
#define MSG_TRUNC	0x20
#define MSG_DONTWAIT	0x40	/* Nonblocking IO.  */
#define MSG_EOR		0x80	/* End of record.  */
#define MSG_WAITALL	0x100	/* Wait for a full request.  */
#define MSG_FIN		0x200
#define MSG_SYN		0x400
#define MSG_CONFIRM	0x800	/* Confirm path validity.  */
#define MSG_RST		0x1000
#define MSG_ERRQUEUE	0x2000	/* Fetch message from error queue.  */
#define MSG_NOSIGNAL	0x4000	/* Do not generate SIGPIPE.  */
#define MSG_MORE	0x8000	/* Sender will send more.  */

/* socklen type */
typedef size_t socklen_t;
struct msghdr; /* XXX */

/* Create a new socket of type TYPE in domain DOMAIN, using
   protocol PROTOCOL.  If PROTOCOL is zero, one is chosen automatically.
   Returns a file descriptor for the new socket, or -1 for errors.  */
int socket (int __domain, int __type, int __protocol);

/* Give the socket FD the local address ADDR (which is LEN bytes long).  */
int bind (int __fd, __CONST_SOCKADDR_ARG __addr, socklen_t __len);

/* Put the local address of FD into *ADDR and its length in *LEN.  */
int getsockname (int __fd, __SOCKADDR_ARG __addr,
			socklen_t *__restrict __len);

/* Open a connection on socket FD to peer at ADDR (which LEN bytes long).
   For connectionless socket types, just set the default address to send to
   and the only address from which to accept transmissions.
   Return 0 on success, -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
int connect (int __fd, __CONST_SOCKADDR_ARG __addr, socklen_t __len);

/* Put the address of the peer connected to socket FD into *ADDR
   (which is *LEN bytes long), and its actual length into *LEN.  */
int getpeername (int __fd, __SOCKADDR_ARG __addr,
			socklen_t *__restrict __len);


/* Send N bytes of BUF to socket FD.  Returns the number sent or -1.

   This function is a cancellation point and therefore not marked with
  .  */
ssize_t send (int __fd, __const void *__buf, size_t __n, int __flags);

/* Read N bytes into BUF from socket FD.
   Returns the number read or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
ssize_t recv (int __fd, void *__buf, size_t __n, int __flags);

/* Send N bytes of BUF on socket FD to peer at address ADDR (which is
   ADDR_LEN bytes long).  Returns the number sent, or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
ssize_t sendto (int __fd, __const void *__buf, size_t __n,
		       int __flags, __CONST_SOCKADDR_ARG __addr,
		       socklen_t __addr_len);

/* Read N bytes into BUF through socket FD.
   If ADDR is not NULL, fill in *ADDR_LEN bytes of it with tha address of
   the sender, and store the actual size of the address in *ADDR_LEN.
   Returns the number of bytes read or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
ssize_t recvfrom (int __fd, void *__restrict __buf, size_t __n,
			 int __flags, __SOCKADDR_ARG __addr,
			 socklen_t *__restrict __addr_len);


/* Send a message described MESSAGE on socket FD.
   Returns the number of bytes sent, or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
ssize_t sendmsg (int __fd, __const struct msghdr *__message,
			int __flags);

/* Receive a message as described by MESSAGE from socket FD.
   Returns the number of bytes read or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
ssize_t recvmsg (int __fd, struct msghdr *__message, int __flags);


/* Put the current value for socket FD's option OPTNAME at protocol level LEVEL
   into OPTVAL (which is *OPTLEN bytes long), and set *OPTLEN to the value's
   actual length.  Returns 0 on success, -1 for errors.  */
int getsockopt (int __fd, int __level, int __optname,
		       void *__restrict __optval,
		       socklen_t *__restrict __optlen);

/* Set socket FD's option OPTNAME at protocol level LEVEL
   to *OPTVAL (which is OPTLEN bytes long).
   Returns 0 on success, -1 for errors.  */
int setsockopt (int __fd, int __level, int __optname,
		       __const void *__optval, socklen_t __optlen);


/* Prepare to accept connections on socket FD.
   N connection requests will be queued before further requests are refused.
   Returns 0 on success, -1 for errors.  */
int listen (int __fd, int __n);

/* Await a connection on socket FD.
   When a connection arrives, open a new socket to communicate with it,
   set *ADDR (which is *ADDR_LEN bytes long) to the address of the connecting
   peer and *ADDR_LEN to the address's actual length, and return the
   new socket's descriptor, or -1 for errors.

   This function is a cancellation point and therefore not marked with
  .  */
int accept (int __fd, __SOCKADDR_ARG __addr,
		   socklen_t *__restrict __addr_len);

/* Shut down all or part of the connection open on socket FD.
   HOW determines what to shut down:
     SHUT_RD   = No more receptions;
     SHUT_WR   = No more transmissions;
     SHUT_RDWR = No more receptions or transmissions.
   Returns 0 on success, -1 for errors.  */
int shutdown (int __fd, int __how);

#endif
