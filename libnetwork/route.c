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

#include <netinet/route.h>
#include <netinet/if.h>

CONTAINER_FUNC(static inline, route_table, DLIST, route_table, NOLOCK, list_entry);

/*
 * Add a route entry.
 */

void			route_add(struct net_if_s	*interface,
				  struct net_route_s	*route)
{
  route_table_push(&interface->route_table, route);
}

/*
 * Get the route to an host.
 */


struct net_route_s	*route_get(struct net_if_s	*interface,
				   struct net_addr_s	*addr)
{
  uint_fast32_t		target;
  uint_fast32_t		mask;
  uint_fast32_t		addr4;

  /* XXX this is IPv4 code */

  addr4 = IPV4_ADDR_GET(*addr);
  CONTAINER_FOREACH(route_table, DLIST, route_table, &interface->route_table,
  {
    target = IPV4_ADDR_GET(item->target);

    if (item->type == ROUTETYPE_HOST)
      {
	if (target == addr4)
	  return item;
      }
    else
      {
	mask = IPV4_ADDR_GET(item->mask);
	if ((addr4 & mask) == target)
	  return item;
      }
  });

  return NULL;
}

