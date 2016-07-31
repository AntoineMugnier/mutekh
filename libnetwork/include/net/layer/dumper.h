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

   @this defines network layer factory for a debug dumper layer.

   Dumper layer only prints all incoming packets, nothing else.
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>

/**
   @this creates a dumper layer. Delegate will be informed of layer
   deletion.

   @param scheduler Attached network scheduler
   @param delegate Delegate object
   @param delegate_vtable Delegate vtable
   @param layer Returned layer reference
   @returns 0 if completed.
 */
error_t net_dumper_create(struct net_scheduler_s *scheduler,
                          void *delegate,
                          const struct net_layer_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer);

#endif
