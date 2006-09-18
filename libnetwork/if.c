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

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/if.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Functions for the interface container.
 */

CONTAINER_FUNC(static inline, net_if, HASHLIST, net_if, NOLOCK, list_entry, STRING, name);

/*
 * Some local variables.
 */

static net_if_root_t	ifs = CONTAINER_ROOT_INITIALIZER(net_if, HASHLIST, NOLOCK);
static uint_fast8_t	ifid = 0;

/*
 * Register a new interface.
 */

void	if_register(struct device_s	*dev)
{
  struct net_proto_s			*arp;
  struct net_proto_s			*icmp;
  struct net_proto_s			*udp;
  struct net_if_s			*interface;

  /* create new device node */
  interface = mem_alloc(sizeof (struct net_if_s), MEM_SCOPE_SYS);

  /* initialize standard protocols for the device */
  interface->ip = net_alloc_proto(&ip_protocol);
  arp = net_alloc_proto(&arp_protocol);
  interface->bootproto.rarp = net_alloc_proto(&rarp_protocol);
  icmp = net_alloc_proto(&icmp_protocol);
  udp = net_alloc_proto(&udp_protocol);
  dev_net_register_proto(dev, interface->ip, arp);
  dev_net_register_proto(dev, arp, interface->ip);
  dev_net_register_proto(dev, interface->bootproto.rarp, interface->ip);
  dev_net_register_proto(dev, icmp, interface->ip);
  dev_net_register_proto(dev, udp, interface->ip);
  interface->boottype = IF_BOOT_RARP;

  /* name the interface */
  interface->dev = dev;
  sprintf(interface->name, "eth%d", ifid++);

  /* add to the interface list */
  net_if_push(&ifs, interface);
}

/*
 * Unregister a net interface.
 */

void			if_unregister(struct device_s	*dev)
{
  /* XXX */
}

/*
 * Bring an interface up.
 */

void			if_up(const char*	name, ...)
{
  struct device_s	*dev;
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&ifs, name)))
    {
      dev = interface->dev;

      switch (interface->boottype)
	{
	  case IF_BOOT_RARP:
	    rarp_request(dev, interface->bootproto.rarp, NULL);
	    break;
	  case IF_BOOT_DHCP:
	    break;
	  default:
	    /* XXX otherwise, static IP */
	    break;
	}
    }
}

/*
 * Bring an interface down.
 */

void			if_down(const char*	name, ...)
{
  struct device_s	*dev;
  struct net_if_s	*interface;

  if ((interface = net_if_lookup(&ifs, name)))
    {
      dev = interface->dev;

      /* XXX */
    }
}
