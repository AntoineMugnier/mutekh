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

#ifndef DRIVER_NET_NS8390_H_
#define DRIVER_NET_NS8390_H_

#include <hexo/device.h>

/* devices addresses slots */

#define NET_NS8390_ADDR	0

/* net device functions */

DEV_IRQ(net_ns8390_irq);
DEV_INIT(net_ns8390_init);
DEV_CLEANUP(net_ns8390_cleanup);
DEVNET_PREPAREPKT(net_ns8390_preparepkt);
DEVNET_SENDPKT(net_ns8390_sendpkt);
DEVNET_REGISTER_PROTO(net_ns8390_register_proto);


#ifndef CONFIG_STATIC_DRIVERS
extern const struct driver_s	net_ns8390_drv;
#endif

#endif

