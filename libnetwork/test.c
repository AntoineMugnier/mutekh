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

#include <hexo/device.h>
#include <hexo/error.h>
#include <hexo/alloc.h>
#include <../drivers/enum-pci/enum-pci.h>

#include <netinet/ip.h>
#include <netinet/arp.h>
#include <netinet/icmp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

CONTAINER_OBJECT_FUNC(static inline, device_list, DLIST, device_list, NOLOCK, device_obj, list_entry);

extern struct device_s enum_pci;

struct device_s	*ne2000;

/*
 * test main.
 */

int_fast8_t		main()
{
  struct enum_id_pci_s	*enum_pv;
  struct net_proto_s	*rarp;
  struct net_proto_s	*arp;
  struct net_proto_s	*ip;
  struct net_proto_s	*icmp;

  /* look for a RTL8029 card */
  CONTAINER_FOREACH(device_list, DLIST, device_list, &enum_pci.children,
  {
    enum_pv = item->enum_pv;
    if (enum_pv->vendor == 0x10ec &&
	enum_pv->devid == 0x8029)
      {
	ne2000 = item;
	/* try to load the NE2000 driver */
	if (!net_ns8390_init(item))
	  goto ok;
      }
  });

 ok:
  /* initialize protocols */
  ip = net_alloc_proto(&ip_protocol);
  arp = net_alloc_proto(&arp_protocol);
  rarp = net_alloc_proto(&rarp_protocol);
  icmp = net_alloc_proto(&icmp_protocol);

  /* register protocols into the driver */
  dev_net_register_proto(ne2000, ip, arp);
  dev_net_register_proto(ne2000, arp, ip);
  dev_net_register_proto(ne2000, rarp, ip);
  dev_net_register_proto(ne2000, icmp, ip);

  /* an RARP request is used to assign us an IP */
  rarp_request(ne2000, rarp, NULL);

  while (1)
    net_ns8390_irq(ne2000); /* XXX replace me by a real IRQ */

  return 0;
}

