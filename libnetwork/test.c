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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/libudp.h>

#include <stdio.h>
#include <string.h>

void			eval_server();

/*
 * test main.
 */

int_fast8_t		main()
{
  uint_fast32_t i;

  for (i = 0; i < 10000000; i++)
    ;

  if_up("eth0");
  //  if_up("eth1");


#if 0

  struct net_route_s *route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);


  route->interface = if_get("eth0");
  IPV4_ADDR_SET(route->target, 0x0a020200);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth0"), route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get("eth1");
  IPV4_ADDR_SET(route->target, 0x0a020300);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth1"), route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get("eth0");
  IPV4_ADDR_SET(route->target, 0x0a020200);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth1"), route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get("eth1");
  IPV4_ADDR_SET(route->target, 0x0a020300);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth0"), route);

  //arp_hardwire(if_get("eth0"), "\x08\x00\x11\x06\x76\x65", 0x0a020302);

  //  if_up("eth2");
#endif

  eval_server();

  return 0;
}

