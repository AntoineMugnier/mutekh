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

#ifndef NETINET_IF_H
#define NETINET_IF_H

#include <hexo/device/net.h>
#include <hexo/device.h>
#include <netinet/protos.h>
#include <netinet/packet.h>
#include <netinet/arp.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Misc.
 */

#define IFNAME_MAX_LEN	32

/*
 * Ifconfig actions.
 */

#define IF_SET	0
#define IF_ADD	1
#define IF_DEL	2

#include <netinet/route.h>

/*
 * Interface types.
 */

typedef uint_fast8_t	net_if_type_t;

#define IF_ETHERNET	ARPHRD_ETHER

/*
 * An interface.
 */

struct					net_if_s
{
  char					name[IFNAME_MAX_LEN];
  int_fast32_t				index;
  struct device_s			*dev;
  const uint8_t				*mac;
  uint_fast16_t				mtu;
  net_protos_root_t			protocols;
  uint_fast16_t				type;

  /* statistics */
  uint_fast64_t				rx_bytes;
  uint_fast64_t				tx_bytes;
  uint_fast32_t				rx_packets;
  uint_fast32_t				tx_packets;

  CONTAINER_ENTRY_TYPE(HASHLIST)	list_entry;
};

/*
 * Interface container types.
 */

CONTAINER_TYPE(net_if, HASHLIST, struct net_if_s, NOLOCK, NOOBJ, list_entry, 4);
CONTAINER_KEY_TYPE(net_if, STRING, name);

/*
 * Functions prototypes.
 */

struct net_if_s	*if_register(struct device_s	*dev,
			     net_if_type_t	type,
			     uint8_t		*mac,
			     uint_fast16_t	mtu);
void	if_unregister(struct net_if_s	*interface);

void	if_up(char*	name, ...);
void	if_down(char*	name, ...);
error_t	if_config(int_fast32_t		ifindex,
		  uint_fast8_t		action,
		  struct net_addr_s	*address,
		  struct net_addr_s	*mask);

void	if_register_proto(struct net_if_s	*interface,
			  struct net_proto_s	*proto,
			  ...);
void	if_pushpkt(struct net_if_s	*interface,
		   struct net_packet_s	*packet);
inline uint8_t	*if_preparepkt(struct net_if_s		*interface,
		       struct net_packet_s	*packet,
		       size_t			size,
		       size_t			max_padding);
void	if_sendpkt(struct net_if_s	*interface,
		   struct net_packet_s	*packet,
		   net_proto_id_t	proto);
void	if_stats(const char	*name);
struct net_if_s	*if_get_by_name(const char	*name);
struct net_if_s	*if_get_by_index(int32_t	index);

#ifdef CONFIG_NETWORK_RARP
error_t			rarp_client(const char	*ifname);
#endif

extern net_if_root_t	net_interfaces;

#endif

