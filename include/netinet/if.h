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

#ifndef NETINET_IF_H
#define NETINET_IF_H

#include <hexo/device.h>
#include <netinet/protos.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Misc.
 */

#define IFNAME_MAX_LEN	32

/*
 * Boot methods.
 */

#define IF_BOOT_RARP	1
#define IF_BOOT_DHCP	2

/*
 * Interface container types.
 */

CONTAINER_TYPE(net_if, HASHLIST, struct net_if_s, NOLOCK, 4, STRING);

/*
 * An interface.
 */

struct			net_if_s
{
  char			name[IFNAME_MAX_LEN];
  struct device_s	*dev;
  struct net_proto_s	*ip;
  union
  {
    struct net_proto_s	*rarp;
    struct net_proto_s	*dhcp;
  } bootproto;
  uint_fast8_t		boottype;

  net_if_entry_t	list_entry;
};

/*
 * Functions prototypes.
 */

void	if_register(struct device_s	*dev);
void	if_unregister(struct device_s	*dev);
void	if_up(const char*	name, ...);
void	if_down(const char*	name, ...);



#endif
