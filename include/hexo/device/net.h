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

#if !defined(DEVICE_H) || defined(DEVICE_NET_H_)
#error This file can not be included directly
#else

#define DEVICE_NET_H_

#include "../types.h"
#include "../error.h"

#include <netinet/packet.h>
#include <netinet/protos.h>

/*
 * packet prepare operation
 */

#define DEVNET_PREPAREPKT(n)	void  (n) (struct device_s *dev, struct net_packet_s *packet, size_t size)

typedef DEVNET_PREPAREPKT(devnet_preparepkt_t);

#define dev_net_preparepkt(dev, ...) (dev)->drv->f.net.f_preparepkt(dev, __VA_ARGS__ )

/*
 * packet send operation
 */

#define DEVNET_SENDPKT(n)	void  (n) (struct device_s *dev, struct net_packet_s *packet, net_proto_id_t proto)

typedef DEVNET_SENDPKT(devnet_sendpkt_t);

#define dev_net_sendpkt(dev, ...) (dev)->drv->f.net.f_sendpkt(dev, __VA_ARGS__ )
/*
 * protocol registration operation
 */

#define DEVNET_REGISTER_PROTO(n)	void  (n) (struct device_s *dev, struct net_proto_s *proto, ...)

typedef DEVNET_REGISTER_PROTO(devnet_register_proto_t);

#define dev_net_register_proto(dev, ...) (dev)->drv->f.net.f_register_proto(dev, __VA_ARGS__ )


/** Net device class methodes */
struct dev_class_net_s
{
  devnet_preparepkt_t		*f_preparepkt;
  devnet_sendpkt_t		*f_sendpkt;
  devnet_register_proto_t	*f_register_proto;
};

#endif

