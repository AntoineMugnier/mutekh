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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#ifndef NETWORK_H_
#define NETWORK_H_

#include <net/layer.h>
#include <net/scheduler.h>

GCT_CONTAINER_FCNS(net_layer_sched_list, static inline, net_layer_sched_list,
                   init, destroy, pop, pushback, isempty, remove, count);

void net_scheduler_layer_created(struct net_scheduler_s *sched, struct net_layer_s *layer);
void net_scheduler_layer_destroyed(struct net_scheduler_s *scheduler, struct net_layer_s *layer);

#endif
