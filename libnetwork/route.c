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

CONTAINER_FUNC(static inline, route_table, CLIST, route_table, NOLOCK);

static route_table_root_t	route_table = CONTAINER_ROOT_INITIALIZER(route_table, CLIST, NOLOCK);

/*
 * Route object constructor.
 */

OBJECT_CONSTRUCTOR(route_obj)
{
  struct net_route_s	*route;
  struct net_addr_s	*target = (struct net_addr_s *)param;
  struct net_addr_s	*mask = va_arg(ap, struct net_addr_s *);
  struct net_if_s	*interface = va_arg(ap, struct net_if_s *);

  if ((route = mem_alloc(sizeof (struct net_route_s), MEM_SCOPE_NETWORK)) == NULL)
    return NULL;

  route_obj_init(route);
  memcpy(&route->target, target, sizeof (struct net_addr_s));
  memcpy(&route->mask, mask, sizeof (struct net_addr_s));
  net_if_obj_refnew(interface);
  route->interface = interface;
  route->addressing = NULL;
  route->is_routed = 0;
  route->invalidated = 0;

#ifdef CONFIG_NETWORK_PROFILING
  netobj_new[NETWORK_PROFILING_ROUTE]++;
#endif

  return route;
}

/*
 * Route object destructor.
 */

OBJECT_DESTRUCTOR(route_obj)
{
  net_if_obj_refdrop(obj->interface);
  if (obj->addressing != NULL)
    net_proto_obj_refdrop(obj->addressing);
  mem_free(obj);

#ifdef CONFIG_NETWORK_PROFILING
  netobj_del[NETWORK_PROFILING_ROUTE]++;
#endif
}

/*
 * Add a route entry.
 */

error_t			route_add(struct net_route_s	*route)
{
  struct net_addr_s	*target;
  struct net_addr_s	*mask;
  net_proto_id_t	id;
  error_t		err = -1;

  /* re-adding a route is not permitted */
  if (route->invalidated || route->addressing != NULL)
    return -1;

  if (!route->is_routed)
    {
      /* direct route: we use the target address to determine which
	 addressing module to use */
      target = &route->target;
    }
  else
    {
      /* indirect route: we use the router address to determine which
	 addressing module to use */
      target = &route->router;
    }

  mask = &route->mask;
  id = target->family;

  /* look throught all the matching addressing protocols */
  NET_FOREACH_PROTO(&route->interface->protocols, id,
  {
    if (item->desc->f.addressing->matchaddr(item, NULL, target, mask))
      {
	struct net_route_s	*rt;
	struct net_route_s	*prec;

	net_proto_obj_refnew(item);
	route->addressing = item;

	/* push the route into the routing table */
	/* the table is sorted by netmask order descendent */
	for (prec = NULL, rt = route_table_head(&route_table);
	     rt != NULL;
	     prec = rt, rt = route_table_next(&route_table, rt))
	  {
	    if (route->mask.family == addr_ipv4)
	      {
		if (rt->mask.addr.ipv4 <= route->mask.addr.ipv4)
		  break;
	      }
	    else
	      assert(0);
	  }

	if (prec == NULL)
	  err = -(!route_table_push(&route_table, route));
	else
	  err = -(!route_table_insert_post(&route_table, prec, route));

	NET_FOREACH_PROTO_BREAK;
      }
  });

  return err;
}

/*
 * Get the route to an host.
 */

struct net_route_s	*route_get(struct net_addr_s	*addr)
{
  struct net_route_s	*ret = NULL;

  /* look into the route table */
  CONTAINER_FOREACH(route_table, CLIST, NOLOCK, &route_table,
  {
    struct net_proto_s	*addressing = item->addressing;

    if (addressing->desc->f.addressing->matchaddr(addressing, &item->target, addr, &item->mask))
      {
	route_obj_refnew(item);
	ret = item;
	CONTAINER_FOREACH_BREAK;
      }
  });

  return ret;
}

/*
 * Flush routes for a given interface
 */

void			route_flush(struct net_if_s	*interface)
{
  struct net_route_s	*prev = NULL;

  /* look into the route table */
  CONTAINER_FOREACH(route_table, CLIST, NOLOCK, &route_table,
  {
    if (prev != NULL)
      {
	route_del(prev);
	prev = NULL;
      }

    if (item->interface == interface)
      prev = item;
  });

  if (prev != NULL)
    route_del(prev);
}

/*
 * Remove a route.
 */

void			route_del(struct net_route_s	*route)
{
  /* invalidate the route */
  route->invalidated = 1;
  route_table_remove(&route_table, route);
}

static void		spc(uint_fast8_t	i)
{
  for (; i > 0; i--)
    printf(" ");
}

/*
 * Dump the route table.
 */

void			route_dump(void)
{
  uint_fast8_t		i;

  printf("Target            Gateway           Mask              Interface\n");

  /* look into the route table */
  CONTAINER_FOREACH(route_table, CLIST, NOLOCK, &route_table,
  {
    switch (item->target.family)
      {
	case addr_ipv4:
	  if (item->target.addr.ipv4)
	    i = printf("%u.%u.%u.%u", EXTRACT_IPV4(item->target.addr.ipv4));
	  else
	    i = printf("default");
	  spc(18 - i);
	  if (item->is_routed)
	    i = printf("%u.%u.%u.%u", EXTRACT_IPV4(item->router.addr.ipv4));
	  else
	    i = 0;
	  spc(18 - i);
	  i = printf("%u.%u.%u.%u", EXTRACT_IPV4(item->mask.addr.ipv4));
	  spc(18 - i);
	  printf("%s\n", item->interface->name);
	  break;
	default:
	  printf("Entry of unsupported type.\n");
	  break;
      }
  });
}
