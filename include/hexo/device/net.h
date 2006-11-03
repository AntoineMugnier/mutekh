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

#ifndef __DEVICE_NET_H__
#define __DEVICE_NET_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/device.h>

struct device_s;
struct driver_s;
struct net_packet_s;

/* network device options */
#define DEV_NET_OPT_PROMISC	1

/*
 * packet prepare operation
 */

#define DEVNET_PREPAREPKT(n)	uint8_t  *(n) (struct device_s *dev, struct net_packet_s *packet, size_t size, size_t max_padding)

typedef DEVNET_PREPAREPKT(devnet_preparepkt_t);

#define dev_net_preparepkt(dev, ...) (dev)->drv->f.net.f_preparepkt(dev, __VA_ARGS__ )

/*
 * packet send operation
 */

#define DEVNET_SENDPKT(n)	void  (n) (struct device_s *dev, struct net_packet_s *packet, uint_fast16_t proto)

typedef DEVNET_SENDPKT(devnet_sendpkt_t);

#define dev_net_sendpkt(dev, ...) (dev)->drv->f.net.f_sendpkt(dev, __VA_ARGS__ )

/*
 * device set option operation
 */

#define DEVNET_SETOPT(n)	error_t (n) (struct device_s *dev, uint_fast32_t option, uintptr_t value)

typedef DEVNET_SETOPT(devnet_setopt_t);

#define dev_net_setopt(dev, ...) (dev)->drv->f.net.f_setopt(dev, __VA_ARGS__ )

/** Net device class methodes */
struct dev_class_net_s
{
  devnet_preparepkt_t		*f_preparepkt;
  devnet_sendpkt_t		*f_sendpkt;
  devnet_setopt_t		*f_setopt;
};

#endif

