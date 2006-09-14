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

#ifndef DRIVER_NET_NE2000PCI_H_
#define DRIVER_NET_NE2000PCI_H_

#include <hexo/device.h>

/* devices addresses slots */

#define NET_NE2000_ADDR		0
#define NET_NE2000_COMMAND	1
#define NET_NE2000_ISR		2

/* net device functions */

DEV_IRQ(net_ne2000pci_irq);
DEV_INIT(net_ne2000pci_init);
DEV_CLEANUP(net_ne2000pci_cleanup);
DEVNET_PREPAREPKT(net_ne2000pci_preparepkt);
DEVNET_SENDPKT(net_ne2000pci_sendpkt);
DEVNET_REGISTER_PROTO(net_ne2000pci_register_proto);


#ifndef CONFIG_STATIC_DRIVERS
extern const struct driver_s	net_ne2000pci_drv;
#endif

#endif

