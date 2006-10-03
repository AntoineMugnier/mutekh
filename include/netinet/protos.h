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
#include <hexo/alloc.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_hashlist.h>

struct net_if_s;
struct net_packet_s;
struct net_addr_s;
struct net_proto_s;

/*
 * Container type for protocols list.
 */

CONTAINER_TYPE(net_protos, HASHLIST, struct net_proto_s, NOLOCK, 8, UNSIGNED);

/*
 * Prototype of a push function.
 */

#define NET_PUSHPKT(f)	void (f)(struct net_if_s	*interface,	\
				 struct net_packet_s	*packet,	\
				 struct net_proto_s	*addressing,	\
				 struct	net_proto_s	*protocol)

typedef NET_PUSHPKT(net_pushpkt_t);

/*
 * Prototype of the function used to prepare a packet.
 */

#define NET_PREPAREPKT(f)	uint8_t *(f)(struct net_if_s	*interface,\
					 struct net_packet_s	*packet,   \
					 size_t			size,	   \
					 size_t			max_padding)

typedef NET_PREPAREPKT(net_preparepkt_t);

/*
 * Prototype of the function used to initialize private data.
 */

#define NET_INITPROTO(f)	void (f)(struct net_if_s	*interface, \
					 struct net_proto_s	*proto,	    \
					 va_list		va)

typedef NET_INITPROTO(net_initproto_t);

typedef uint_fast16_t net_pkt_size_t;
typedef uint_fast16_t net_proto_id_t;

/*
 * Prototype of the function used to send a packet.
 */

#define NET_SENDPKT(f)		void (f)(struct net_if_s	*interface,  \
					 struct net_packet_s	*packet,     \
					 struct net_proto_s	*protocol,   \
					 net_proto_id_t		proto)

typedef NET_SENDPKT(net_sendpkt_t);

/*
 * Prototype of the function used to match internet addresses.
 */

#define NET_MATCHADDR(f) uint_fast8_t (f)(struct net_proto_s	*protocol,\
					  struct net_addr_s	*a,	  \
					  struct net_addr_s	*b,	  \
					  struct net_addr_s	*mask)

typedef NET_MATCHADDR(net_matchaddr_t);

/*
 * Prototype of the function that computes the pseudo header checksum.
 */

#define NET_PSEUDOHEADER_CHECKSUM(f)					\
  uint16_t	(f)(struct net_packet_s	*packet,			\
		    net_proto_id_t	proto,				\
		    uint_fast16_t	size)

typedef NET_PSEUDOHEADER_CHECKSUM(net_pseudoheader_checksum_t);

/*
 * This structure defines the interface of an addressing protocol.
 */

struct				net_addressing_interface_s
{
  net_sendpkt_t			*sendpkt;
  net_matchaddr_t		*matchaddr;
  net_pseudoheader_checksum_t	*pseudoheader_checksum;
};

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
    const struct net_addressing_interface_s	*addressing;
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

/*
 * Allocate a new protocol node.
 */

static inline struct net_proto_s	*net_alloc_proto(const struct net_proto_desc_s	*desc)
{
  struct net_proto_s	*proto;

  proto = mem_alloc(sizeof (struct net_proto_s) + desc->pv_size,
		    MEM_SCOPE_CONTEXT);

  proto->desc = desc;
  proto->id = desc->id;
  proto->pv = (void*)((uint8_t*)proto + sizeof (struct net_proto_s));

  return proto;
}

#endif

