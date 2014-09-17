/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006
*/

#ifndef NETWORK_IP_H_
#define NETWORK_IP_H_

/**
   @file
   @module{Network library}
   @short IP stack
 */

#include <network/packet.h>
#include <network/protos.h>
#include <network/route.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_chainedhash.h>

#include <device/class/timer.h>

#define IP_DELIVERY_DIRECT	0
#define IP_DELIVERY_INDIRECT	1

/** Reassembly timeout: 10 seconds */
#define IP_REASSEMBLY_TIMEOUT	10000

/** IP pseudo header (for upper layer checksum computation) */
struct ip_pseudoheader_s
{
  uint32_t	source;
  uint32_t	dest;
  uint8_t	zero;
  uint8_t	type;
  uint16_t	size;
} __attribute__((packed));

/*
 * Fragmentation structures.
 */

#define GCT_CONTAINER_LOCK_ip_packet	HEXO_LOCK
#define GCT_CONTAINER_ALGO_ip_packet	CHAINEDHASH

struct					ip_packet_s
{
  uint8_t				id[6];
  uint_fast16_t				size;
  uint_fast16_t				received;
  struct net_proto_s			*addressing;
  struct dev_timer_rq_s			timeout;
  packet_queue_root_t			packets;

  GCT_CONTAINER_ENTRY(ip_packet, list_entry);
};

struct ip_packet_s *fragment_obj_new(struct net_proto_s *addressing,
                                     const uint8_t *id);

void fragment_obj_delete(struct ip_packet_s *obj);

/*
 * Fragments list.
 */

GCT_CONTAINER_TYPES(ip_packet, struct ip_packet_s *, list_entry, 64);
GCT_CONTAINER_KEY_TYPES(ip_packet, PTR, BLOB, id, 6);

/*
 * IP private data.
 */

struct			net_pv_ip_s
{
  dev_timer_delay_t     reassembly_timeout;
  struct net_if_s	*interface;
  struct net_proto_s	*arp;
  struct net_proto_s	*icmp;
  uint_fast32_t		addr;
  uint_fast32_t		mask;
  ip_packet_root_t	fragments;
  uint_fast32_t		id_seq;
};

/*
 * IP functions
 */

NET_INITPROTO(ip_init);
NET_DESTROYPROTO(ip_destroy);
NET_PUSHPKT(ip_pushpkt);
NET_PREPAREPKT(ip_preparepkt);
NET_SENDPKT(ip_send);
NET_MATCHADDR(ip_matchaddr);
NET_PSEUDOHEADER_CHECKSUM(ip_pseudoheader_checksum);

void		ip_route(struct net_packet_s	*packet,
			 struct net_route_s	*route);

extern const struct net_proto_desc_s	ip_protocol;

#endif
