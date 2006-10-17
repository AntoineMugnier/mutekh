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

#include <netinet/protos.h>
#include <netinet/route.h>
#include <netinet/ether.h>
#include <netinet/if.h>
#include <netinet/ip.h>

CONTAINER_FUNC(static inline, route_table, DLIST, route_table, NOLOCK);

/*
 * Add a route entry.
 */

void			route_add(struct net_if_s	*interface,
				  struct net_route_s	*route)
{
  struct net_addr_s	*target;
  struct net_addr_s	*mask;

  if (route->type & ROUTETYPE_DIRECT)
    {
      /* direct route: we use the target address to determine which
	 addressing module to use */
      target = &route->target;
      mask = &route->mask;
    }
  else
    {
      /* indirect route: we use the router address to determine which
	 addressing module to use */
      target = &route->router;
      mask = &route->mask;
    }

  /* look for the addressing module to bind to the route */
  CONTAINER_FOREACH(net_protos, HASHLIST, NOLOCK, &route->interface->protocols,
  {
    if (item->id == ETHERTYPE_IP)
      {
	if (item->desc->f.addressing->matchaddr(item, NULL, target, mask))
	  {
	    route->addressing = item;
	    goto ok;
	  }
      }
  });

 ok:
  route_table_push(&interface->route_table, route);
}

/*
 * Get the route to an host.
 */


struct net_route_s	*route_get(struct net_if_s	*interface,
				   struct net_addr_s	*addr)
{
  /* look into the route table XXX must sort it with netmask */
  CONTAINER_FOREACH(route_table, DLIST, NOLOCK, &interface->route_table,
  {
    /* an entry for a single host */
    if (item->type == ROUTETYPE_HOST)
      {
	if (item->addressing->desc->f.addressing->matchaddr(item->addressing, &item->target, addr, NULL))
	  return item;
      }
    else /* an entry for a subnet */
      {
	if (item->addressing->desc->f.addressing->matchaddr(item->addressing, &item->target, addr, &item->mask))
	  return item;
      }
  });

  return NULL;
}

