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

#ifndef NET_LAYER_DUMPER_H_
#define NET_LAYER_DUMPER_H_

/**
   @file
   @module {Libraries::Abstract network stack}
   @short Network layer definition for Dumper interface

   @this defines a factory for a pair of layers forwarding packet each
   to the other.
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>

/**
   @this creates a layer pair.

   @returns 0 if completed.
 */
error_t net_pipe_create(struct net_scheduler_s *scheduler,
                        const struct net_addr_s *addr0,
                        void *delegate0,
                        const struct net_layer_delegate_vtable_s *delegate_vtable0,
                        struct net_layer_s **layer0,
                        const struct net_addr_s *addr1,
                        void *delegate1,
                        const struct net_layer_delegate_vtable_s *delegate_vtable1,
                        struct net_layer_s **layer1);

#endif
