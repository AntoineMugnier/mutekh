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

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <net/layer/dumper.h>

struct net_dumper_s
{
  struct net_layer_s layer;
};

STRUCT_COMPOSE(net_dumper_s, layer);

static
void net_dumper_destroyed(struct net_layer_s *layer)
{
  struct net_dumper_s *dumper = net_dumper_s_from_layer(layer);

  mem_free(dumper);
}

static
void net_dumper_task_handle(struct net_layer_s *layer,
                            struct net_task_s *task)
{
  struct net_dumper_s *dumper = net_dumper_s_from_layer(layer);
  (void)dumper;

  switch (task->type) {
  default:
    break;

  case NET_TASK_INBOUND:
    printk("> %P\n",
           task->packet.buffer->data + task->packet.buffer->begin,
           task->packet.buffer->end - task->packet.buffer->begin);
    break;
  }

  net_task_destroy(task);
}

static
void net_dumper_dangling(struct net_layer_s *layer)
{
  struct net_dumper_s *dumper = net_dumper_s_from_layer(layer);

  (void)dumper;
}

static const struct net_layer_handler_s dumper_handler = {
  .destroyed = net_dumper_destroyed,
  .task_handle = net_dumper_task_handle,
  .dangling = net_dumper_dangling,
};

error_t net_dumper_create(struct net_scheduler_s *scheduler,
                          void *delegate,
                          const struct net_layer_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer)
{
 struct net_dumper_s *dumper = mem_alloc(sizeof(*dumper), mem_scope_sys);

  if (!dumper)
    return -ENOMEM;

  memset(dumper, 0, sizeof(*dumper));

  error_t err = net_layer_init(&dumper->layer, &dumper_handler, scheduler,
                               delegate, delegate_vtable);
  if (err)
    mem_free(dumper);
  else
    *layer = &dumper->layer;

  return err;
}

