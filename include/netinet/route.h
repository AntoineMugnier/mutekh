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

#ifndef NETINET_ROUTE_H
#define NETINET_ROUTE_H

#include <hexo/types.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_dlist.h>

/*
 * Specify if a route entry is for a network or a specific host.
 */

#define ROUTETYPE_NET	0
#define ROUTETYPE_HOST	1

/*
 * Route table container.
 */

CONTAINER_TYPE(route_table, DLIST, struct net_route_s, NOLOCK);

#include <netinet/if.h>

/*
 * Structure defining a route.
 */

struct			net_route_s
{
  struct net_if_s	*interface;
  struct net_addr_s	target;
  uint_fast8_t		type;
  struct net_addr_s	mask;
  struct net_addr_s	router;
  uint_fast16_t		flags;

  route_table_entry_t	list_entry;
};

/*
 * Prototypes
 */

void			route_add(struct net_if_s	*interface,
				  struct net_route_s	*route);
struct net_route_s	*route_get(struct net_if_s	*interface,
				   struct net_addr_s	*addr);

#endif
