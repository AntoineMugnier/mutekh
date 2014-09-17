/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <network/protos.h>
#include <network/route.h>
#include <netinet/ether.h>
#include <network/if.h>
#include <netinet/ip.h>

#include <mutek/printk.h>

GCT_CONTAINER_FCNS(route_table, static inline, route_table,
  init, destroy, head, next, push, insert_next, remove);

static route_table_root_t	route_table = GCT_CONTAINER_ROOT_INITIALIZER(route_table);

/*
 * Route object constructor.
 */

struct net_route_s *route_obj_new(struct net_addr_s *target,
                                  struct net_addr_s *mask,
                                  struct net_if_s *interface)
{
  struct net_route_s *obj = mem_alloc(sizeof(*obj), mem_scope_sys);

  memcpy(&obj->target, target, sizeof (struct net_addr_s));
  memcpy(&obj->mask, mask, sizeof (struct net_addr_s));
  net_if_obj_refinc(interface);
  obj->interface = interface;
  obj->addressing = NULL;
  obj->is_routed = 0;
  obj->invalidated = 0;

#ifdef CONFIG_NETWORK_PROFILING
  netobj_new[NETWORK_PROFILING_ROUTE]++;
#endif

  return 0;
}

/*
 * Route object destructor.
 */

void route_obj_destroy(struct net_route_s * obj)
{
  net_if_obj_refdec(obj->interface);
  if (obj->addressing != NULL)
    net_proto_obj_refdec(obj->addressing);

#ifdef CONFIG_NETWORK_PROFILING
  netobj_del[NETWORK_PROFILING_ROUTE]++;
#endif

  mem_free(obj);
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

	net_proto_obj_refinc(item);
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
	  err = -(!route_table_insert_next(&route_table, prec, route));

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
  GCT_FOREACH(route_table, &route_table, item,
  {
    struct net_proto_s	*addressing = item->addressing;

    if (addressing->desc->f.addressing->matchaddr(addressing, &item->target, addr, &item->mask))
      {
	route_obj_refinc(item);
	ret = item;
	GCT_FOREACH_BREAK;
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
  GCT_FOREACH(route_table, &route_table, item,
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
    printk(" ");
}

/*
 * Dump the route table.
 */

void			route_dump(void)
{
  uint_fast8_t		i;

  printk("Target            Gateway           Mask              Interface\n");

  /* look into the route table */
  GCT_FOREACH(route_table, &route_table, item,
  {
    switch (item->target.family)
      {
	case addr_ipv4:
	  if (item->target.addr.ipv4)
	    i = printk("%u.%u.%u.%u", EXTRACT_IPV4(item->target.addr.ipv4));
	  else
	    i = printk("default");
	  spc(18 - i);
	  if (item->is_routed)
	    i = printk("%u.%u.%u.%u", EXTRACT_IPV4(item->router.addr.ipv4));
	  else
	    i = 0;
	  spc(18 - i);
	  i = printk("%u.%u.%u.%u", EXTRACT_IPV4(item->mask.addr.ipv4));
	  spc(18 - i);
	  printk("%s\n", item->interface->name);
	  break;
	default:
	  printk("Entry of unsupported type.\n");
	  break;
      }
  });
}
