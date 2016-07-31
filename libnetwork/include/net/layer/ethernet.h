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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

#ifndef NET_LAYER_ETHERNET_H_
#define NET_LAYER_ETHERNET_H_

/**
   @file
   @module {Libraries::Abstract network stack}
   @short Network layer definition for Ethernet interface

   @this defines network layer API for ethernet physical layer.  Its
   implementations are device-specific.
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>

/**
   Network layer ID for ethernet.
 */
#define NET_LAYER_ETHERNET CONFIG_NET_ETHERNET_LAYER_FIRST

/**
   @this is a vtable for an ethernet network layer.
*/
struct net_ethernet_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  /**
     @this is called when the link-layer status of the network device
     changes.
   */
  void (*link_changed)(void *delegate, struct net_layer_s *layer, bool_t connected);
};

STRUCT_COMPOSE(net_ethernet_delegate_vtable_s, base);

#endif
