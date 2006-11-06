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
#include <netinet/packet.h>
#include <netinet/sockaddr.h>
#include <netinet/in.h>
#include <hexo/alloc.h>

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

/* socklen type */
typedef size_t socklen_t;

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

/* I/O vector is used for messages */
struct iovec
  {
    void *iov_base;	/* Pointer to data.  */
    size_t iov_len;	/* Length of data.  */
  };

/* Structure describing messages sent by
   `sendmsg' and received by `recvmsg'.  */
struct msghdr
  {
    void *msg_name;		/* Address to send to/receive from.  */
    socklen_t msg_namelen;	/* Length of address data.  */

    struct iovec *msg_iov;	/* Vector of data to send/receive into.  */
    size_t msg_iovlen;		/* Number of elements in the vector.  */

    void *msg_control;		/* Ancillary data (eg BSD filedesc passing). */
    size_t msg_controllen;	/* Ancillary data buffer length.  */

    int msg_flags;		/* Flags on received message.  */
  };

/* Structure used for storage of ancillary data object information.  */
struct cmsghdr
  {
    size_t cmsg_len;		/* Length of data in cmsg_data plus length
				   of cmsghdr structure.  */
    int cmsg_level;		/* Originating protocol.  */
    int cmsg_type;		/* Protocol specific type.  */
  };

#define CMSG_DATA(cmsg) ((unsigned char *) ((struct cmsghdr *) (cmsg) + 1))

/* for PF_PACKET */
struct sockaddr_pkt
{
	unsigned short spkt_family;
	unsigned char spkt_device[14];
	unsigned short spkt_protocol;
};

struct sockaddr_ll
{
	unsigned short	sll_family;
	unsigned short	sll_protocol;
	int		sll_ifindex;
	unsigned short	sll_hatype;
	unsigned char	sll_pkttype;
	unsigned char	sll_halen;
	unsigned char	sll_addr[8];
};

/* Packet types */
#define PACKET_HOST		0		/* To us		*/
#define PACKET_BROADCAST	1		/* To all		*/
#define PACKET_MULTICAST	2		/* To group		*/
#define PACKET_OTHERHOST	3		/* To someone else 	*/
#define PACKET_OUTGOING		4		/* Outgoing of any type */

#define PACKET_ADD_MEMBERSHIP		1
#define PACKET_DROP_MEMBERSHIP		2
#define PACKET_RECV_OUTPUT		3

struct packet_mreq
{
	int		mr_ifindex;
	unsigned short	mr_type;
	unsigned short	mr_alen;
	unsigned char	mr_address[8];
};

#define PACKET_MR_MULTICAST	0
#define PACKET_MR_PROMISC	1
#define PACKET_MR_ALLMULTI	2


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

struct socket_s;
struct socket_api_s;

# if defined(CONFIG_NETWORK_SOCKET_HEXO)
#include "socket_hexo.h"
# elif defined(CONFIG_NETWORK_SOCKET_POSIX)
#include "socket_posix.h"
# else
#  error Neither CONFIG_NETWORK_SOCKET_HEXO nor CONFIG_NETWORK_SOCKET_POSIX are defined.
# endif

/*
 * Socket API prototypes.
 */

#define _SOCKET(f)	socket_t (f)(socket_t	fd,		\
				     int	domain,		\
				     int	type,		\
				     int	protocol)

#define _BIND(f)	int (f)(socket_t	fd,	\
				struct sockaddr	*addr,	\
				socklen_t	len)

#define _GETSOCKNAME(f)	int (f)(socket_t	fd,	\
				struct sockaddr	*addr,	\
				socklen_t	*len)

#define _CONNECT(f)	int (f)(socket_t	fd,	\
				struct sockaddr	*addr,	\
				socklen_t	len)

#define _GETPEERNAME(f)	int (f)(socket_t	fd,	\
				struct sockaddr	*addr,	\
				socklen_t	*len)

#define _SEND(f)	ssize_t (f)(socket_t	fd,	\
				    const void	*buf,	\
				    size_t	n,	\
				    int		flags)

#define _RECV(f)	ssize_t (f)(socket_t	fd,	\
				    void	*buf,	\
				    size_t	n,	\
				    int		flags)
#define _SENDTO(f)	ssize_t (f)(socket_t		fd,		\
				    const void		*buf,		\
				    size_t		n,		\
				    int			flags,		\
				    struct sockaddr	*addr,		\
				    socklen_t		addr_len)

#define _RECVFROM(f)	ssize_t (f)(socket_t		fd,		\
				    void		*buf,		\
				    size_t		n,		\
				    int			flags,		\
				    struct sockaddr	*addr,		\
				    socklen_t		*addr_len)

#define _SENDMSG(f)	ssize_t (f)(socket_t		fd,		\
				    const struct msghdr	*message,	\
				    int			flags)

#define _RECVMSG(f)	ssize_t (f)(socket_t		fd,		\
				    struct msghdr	*message,	\
				    int			flags)

#define _GETSOCKOPT(f)	int (f)(socket_t	fd,		\
				int		level,		\
				int		optname,	\
				void		*optval,	\
				socklen_t	*optlen)

#define _SETSOCKOPT(f)	int (f)(socket_t	fd,		\
				int		level,		\
				int		optname,	\
				const void	*optval,	\
				socklen_t	optlen)

#define _LISTEN(f)	int (f)(socket_t	fd,	\
				int		n)

#define _ACCEPT(f)	int (f)(socket_t	fd,		\
				struct sockaddr	*addr,		\
				socklen_t	*addr_len)

#define _SHUTDOWN(f)	int (f)(socket_t	fd,	\
				int		how)

typedef _SOCKET(_socket_t);
typedef _BIND(_bind_t);
typedef _GETSOCKNAME(_getsockname_t);
typedef _CONNECT(_connect_t);
typedef _GETPEERNAME(_getpeername_t);
typedef _SEND(_send_t);
typedef _RECV(_recv_t);
typedef _SENDTO(_sendto_t);
typedef _RECVFROM(_recvfrom_t);
typedef _SENDMSG(_sendmsg_t);
typedef _RECVMSG(_recvmsg_t);
typedef _GETSOCKOPT(_getsockopt_t);
typedef _SETSOCKOPT(_setsockopt_t);
typedef _LISTEN(_listen_t);
typedef _ACCEPT(_accept_t);
typedef _SHUTDOWN(_shutdown_t);

/*
 * Socket dispatch structure.
 */

struct			socket_api_s
{
  _socket_t		*socket;
  _bind_t		*bind;
  _getsockname_t	*getsockname;
  _connect_t		*connect;
  _getpeername_t	*getpeername;
  _sendto_t		*sendto;
  _recvfrom_t		*recvfrom;
  _getsockopt_t		*getsockopt;
  _setsockopt_t		*setsockopt;
  _listen_t		*listen;
  _accept_t		*accept;
  _shutdown_t		*shutdown;
};

/*
 * Common operations.
 */

_RECV(recv);
_SEND(send);
_RECVMSG(recvmsg);
_SENDMSG(sendmsg);

/*
 * Dispatch structure instances.
 */

#ifdef CONFIG_NETWORK_UDP
extern const struct socket_api_s	udp_socket;
#endif
#ifdef CONFIG_NETWORK_TDP
extern const struct socket_api_s	tcp_socket;
#endif
#ifdef CONFIG_NETWORK_SOCKET_PACKET
extern const struct socket_api_s	packet_socket;
#endif
#ifdef CONFIG_NETWORK_SOCKET_RAW
extern const struct socket_api_s	raw_socket;
#endif

/*
 * A socket.
 */

struct				socket_s
{
  const struct socket_api_s	*f;
  void				*pv;
};

#endif
