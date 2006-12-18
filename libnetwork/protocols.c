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

#include <hexo/types.h>
#include <hexo/alloc.h>

#include <netinet/packet.h>
#include <netinet/protos.h>

/*
 * Protocol instance constructor
 */

OBJECT_CONSTRUCTOR(net_proto_obj)
{
  struct net_proto_s			*proto;
  const struct net_proto_desc_s		*desc = va_arg(ap, const struct net_proto_desc_s *);

  if ((proto = mem_alloc(sizeof (struct net_proto_s) + desc->pv_size, MEM_SCOPE_CONTEXT)) == NULL)
    return NULL;

  net_proto_obj_init(proto);
  proto->desc = desc;
  proto->id = desc->id;
  proto->initialized = 0;
  if (desc->pv_size)
    proto->pv = (void *)(proto + 1);
  else
    proto->pv = NULL;

#ifdef CONFIG_NETWORK_PROFILING
  netobj_new[NETWORK_PROFILING_PROTO]++;
#endif

  return proto;
}

/*
 * Protocol instance destructor
 */

OBJECT_DESTRUCTOR(net_proto_obj)
{
  if (obj->desc->destroyproto != NULL && obj->initialized)
    obj->desc->destroyproto(obj);

  mem_free(obj);

#ifdef CONFIG_NETWORK_PROFILING
  netobj_del[NETWORK_PROFILING_PROTO]++;
#endif
}
