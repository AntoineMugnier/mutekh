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

#ifndef NETWORK_IF_H
#define NETWORK_IF_H

/**
   @file
   @module{Network library}
   @short Interface handling
 */

#include <device/class/net.h>
#include <device/device.h>
#include <network/protos.h>
#include <network/packet.h>
#include <netinet/arp.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_chainedhash.h>
#include <gct/refcount.h>

/*
 * Misc.
 */

#define IFNAME_MAX_LEN	32

/*
 * Ifconfig actions.
 */

enum if_action_e {
  IF_SET,
  IF_ADD,
  IF_DEL,
};

/*
 * If state
 */

#define NET_IF_STATE_DOWN	0
#define NET_IF_STATE_UP		1

#include <network/route.h>

/*
 * Interface types.
 */

typedef uint_fast8_t	net_if_type_t;

#define IF_ETHERNET	ARPHRD_ETHER

/*
 * An interface.
 */

#define GCT_CONTAINER_REFCOUNT_net_if		net_if_obj
#define GCT_CONTAINER_LOCK_net_if		HEXO_LOCK
#define GCT_CONTAINER_ALGO_net_if		CHAINEDHASH

struct net_if_s
{
  char					name[IFNAME_MAX_LEN];
  int_fast32_t				index;
  struct device_net_s			dev;
  const uint8_t				*mac;
  size_t				mac_len;
  uint_fast16_t				mtu;
  net_protos_root_t			protocols;
  uint_fast16_t				type;
  uint_fast8_t				state;

  /* statistics */
  uint_fast32_t				rx_bytes;
  uint_fast32_t				tx_bytes;
  uint_fast32_t				rx_packets;
  uint_fast32_t				tx_packets;

  struct net_dispatch_s                 *dispatch;
  GCT_REFCOUNT_ENTRY(obj_entry);
  GCT_CONTAINER_ENTRY(net_if, list_entry);
};

GCT_REFCOUNT(net_if_obj, struct net_if_s *, obj_entry);

struct net_if_s * net_if_obj_new(struct device_s *dev,
                               net_if_type_t type,
                               uint_fast16_t mtu);

void net_if_obj_destroy(struct net_if_s *);


/*
 * Interface container types.
 */

GCT_CONTAINER_TYPES(net_if, struct net_if_s *, list_entry, 4);
GCT_CONTAINER_KEY_TYPES(net_if, PTR, STRING, name);


/**
   @this creates a new interface.

   @param dev Device associated to the interface
   @param type Type of the interface
   @param mac Hardware address of the interface
   @param mtu Interface MTU

   @returns a newly allocated interface status buffer
 */
struct net_if_s	*if_register(struct device_s	*dev,
			     net_if_type_t	type,
			     uint_fast16_t	mtu);

/**
   @this destroys an existing interface. All handling threads and
   received packets should be freed before calling this function.

   @param interface Interface to destroy
 */
void if_unregister(struct net_if_s *interface);

/**
   @this set an interface up by starting a new kernel dispatch thread for this interface.

   @param interface Interface to configure
 */
error_t if_up(struct net_if_s *interface);

/**
   @this configures an interface to be unable to exchange packets

   @param interface Interface to configure
 */
void if_down(struct net_if_s *interface);

/**
   @this configures addresses of an interface. The interface must be up.

   @param interface Interface to configure
   @param action What to do with the address
   @param address Address to modify
   @param mask Mask associated to the address

   @returns 0 when done, or an error
 */
error_t	if_config(struct net_if_s *interface,
                  enum if_action_e action,
                  struct net_addr_s *address,
                  struct net_addr_s *mask);

/**
   @this registers a protocol as handled by this interface.

   @param interface Interface to configure
   @param proto Protocol instance for handling packets

   @returns 0 when done, or an error
 */
error_t	if_register_proto(struct net_if_s	*interface,
                          struct net_proto_s	*proto,
                          ...);

/**
   @this pushes a packet to the protocol stack handled by this
   interface.

   @param interface Interface handling packet
   @param packet Packet to push
 */
void	if_pushpkt(struct net_if_s	*interface,
                   struct net_packet_s	*packet);

inline uint8_t	*if_preparepkt(struct net_if_s		*interface,
		       struct net_packet_s	*packet,
		       size_t			size,
		       size_t			max_padding);


void	if_sendpkt(struct net_if_s	*interface,
		   struct net_packet_s	*packet,
		   net_proto_id_t	proto);

/**
   @This pushes a packet from device irq handler for dispatch by the kernel thread.
 */
void if_dispatch(struct net_if_s *interface, struct net_packet_s *packet);

/**
   @this dumps interface state to console

   @param interface Interface to dump status about

   @returns 0 when done, -EINVAL if interface is down
 */
error_t if_dump(struct net_if_s *interface);

/**
   @this retrieves an interface from its name.

   @param name Interface name

   @returns the interface if it exists, or NULL
 */
struct net_if_s	*if_get_by_name(const char	*name);

/**
   @this retrieves an interface from its index in system.

   @param index Interface index

   @returns the interface if it exists, or NULL
 */
struct net_if_s	*if_get_by_index(int32_t	index);

#ifdef CONFIG_NETWORK_RARP
error_t			rarp_client(const char	*ifname);
#endif

/** Global system interface list */
extern net_if_root_t	net_interfaces;

#endif

