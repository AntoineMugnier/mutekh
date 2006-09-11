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

#ifndef NETINET_PROTOS_H_
#define NETINET_PROTOS_H_

#include <hexo/types.h>
#include <netinet/packet.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_hashlist.h>

struct device_s;

/*
 * Container type for protocols list.
 */

CONTAINER_TYPE(net_protos, HASHLIST, struct net_proto_s, NOLOCK, 8, UNSIGNED);

/*
 * Prototype of a push function.
 */

#define NET_PUSHPKT(f)	void (f)(struct device_s	*dev,		\
				 struct net_packet_s	*packet,	\
				 struct	net_proto_s	*protocol,	\
				 net_protos_root_t	*protocols)

typedef NET_PUSHPKT(net_pushpkt_t);

/*
 * Prototype of the function used to prepare a packet.
 */

#define NET_PREPAREPKT(f)	void (f)(struct device_s	*dev,	   \
					 struct net_packet_s	*packet,   \
					 size_t			size)

typedef NET_PREPAREPKT(net_preparepkt_t);

/*
 * Prototype of the function used to initialize private data.
 */

#define NET_INITPROTO(f)	void (f)(struct device_s	*dev,	\
					 struct net_proto_s	*proto,	\
					 va_list		va)

typedef NET_INITPROTO(net_initproto_t);

#include <netinet/ether.h>
#include <netinet/arp.h>
#include <netinet/ip.h>
#include <netinet/icmp.h>
#include <netinet/dummy.h>

typedef uint_fast16_t net_pkt_size_t;
typedef uint_fast16_t net_proto_id_t;

/*
 * This structure defines a protocol.
 */

struct					net_proto_desc_s
{
  const char				*name;	/* the name of the protocol */
  net_proto_id_t			id;	/* protocol identifier */
  net_pushpkt_t				*pushpkt; /* push packet function */
  net_preparepkt_t			*preparepkt; /* prepare packet func */
  net_initproto_t			*initproto; /* init pv data */
  union
  {
    const struct ether_interface_s	*ether;	/* ethernet interface */
    const struct ip_interface_s		*ip;	/* ip protocol interface */
    const struct arp_interface_s	*arp;	/* arp protocol interface */
    const struct rarp_interface_s	*rarp;	/* rarp protocol interface */
    const struct icmp_interface_s	*icmp;	/* icmp protocol interface */
#if 0
    const struct udp_interface_s	*udp;	/* udp protocol interface */
    const struct tcp_interface_s	*tcp;	/* tcp protocol interface */
#endif
    const void				*other;	/* other protocol interface */
  } f;
  size_t				pv_size;
};

struct					net_proto_s
{
  const struct net_proto_desc_s		*desc;	/* protocol descriptor */
  net_protos_entry_t			list_entry;
  net_proto_id_t			id;	/* protocol identifier */
  struct net_proto_pv_s			*pv;	/* private data */
};

/*
 * Container functions.
 */

CONTAINER_FUNC(static inline, net_protos, HASHLIST, net_protos, NOLOCK, list_entry, UNSIGNED, id);

#endif

