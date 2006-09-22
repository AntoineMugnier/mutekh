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

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Misc.
 */

#define IFNAME_MAX_LEN	32

/*
 * Boot methods.
 */

#define IF_BOOT_RARP	0
#define IF_BOOT_DHCP	1

/*
 * Address and mask structures.
 */

enum	net_addr_e
  {
	addr_ipv4
  };

struct			net_addr_s
{
  enum net_addr_e	family;
  union
  {
    uint_fast32_t	ipv4;
  } addr;
};

#define IPV4_ADDR_SET(_addr_,_ip_)					\
  {									\
    (_addr_).family = addr_ipv4;					\
    (_addr_).addr.ipv4 = (_ip_);					\
  }

#define IPV4_ADDR_GET(_addr_)						\
  ({									\
    assert(1 || (_addr_).family == addr_ipv4);				\
    (_addr_).addr.ipv4;							\
  })

#include <netinet/route.h>

/*
 * Interface types.
 */

typedef uint_fast8_t	net_if_type_t;

#define IF_ETHERNET	0

/*
 * Interface container types.
 */

CONTAINER_TYPE(net_if, HASHLIST, struct net_if_s, NOLOCK, 4, STRING);

/*
 * An interface.
 */

struct			net_if_s
{
  char			name[IFNAME_MAX_LEN];
  struct device_s	*dev;
  const uint8_t		*mac;
  net_protos_root_t	protocols;
  route_table_root_t	route_table;

  struct net_proto_s	*ip;

  /* fields for booting */
  union
  {
    struct net_proto_s	*rarp;
    struct net_proto_s	*dhcp;
  } bootproto;
  uint_fast8_t		boottype;

  /* aliasing */
  /* XXX */

  /* statistics */
  uint_fast64_t		rx_bytes;
  uint_fast64_t		tx_bytes;
  uint_fast32_t		rx_packets;
  uint_fast32_t		tx_packets;

  net_if_entry_t	list_entry;
};

/*
 * Functions prototypes.
 */

struct net_if_s	*if_register(struct device_s	*dev,
			     net_if_type_t	type,
			     uint8_t		*mac);
void	if_unregister(struct net_if_s	*interface);

void	if_up(char*	name, ...);
void	if_down(char*	name, ...);

void	if_register_proto(struct net_if_s	*interface,
			  struct net_proto_s	*proto,
			  ...);
void	if_pushpkt(struct net_if_s	*interface,
		   struct net_packet_s	*packet);
uint8_t	*if_preparepkt(struct net_if_s		*interface,
		       struct net_packet_s	*packet,
		       size_t			size,
		       size_t			max_padding);
void	if_sendpkt(struct net_if_s	*interface,
		   struct net_packet_s	*packet,
		   struct net_proto_s	*proto);
void	if_stats(const char	*name);
struct net_if_s	*if_get(const char	*name);

#endif

