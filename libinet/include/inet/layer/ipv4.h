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

/**
   @file
   @module {Libraries::Internet protocol suite}
*/

#ifndef INET_LAYER_IPV4_H_
#define INET_LAYER_IPV4_H_

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>

error_t inet_ipv4_create(struct net_scheduler_s *scheduler,
                         void *delegate,
                         const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer);

#endif
