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

#include <net/layer/pipe.h>

struct net_pipe_s;

struct pipe_end_s
{
  struct net_layer_s layer;
  struct net_layer_s *child;
  // Must be set to NULL on layer detruction.
  struct net_pipe_s *pipe;
  struct net_addr_s src_addr;
};

STRUCT_COMPOSE(pipe_end_s, layer);

struct net_pipe_s
{
  struct net_layer_s end[2];
};

static
void net_pipe_layer_destroyed(struct net_layer_s *layer)
{
  struct pipe_end_s *end = pipe_end_s_from_layer(layer);
  struct net_pipe_s *pipe = end->pipe;
  size_t me = end - pipe->end;
  struct pipe_end_s *other = &pipe->end[!me];

  assert(pipe);
  end->pipe = NULL;

  if (me == 1) {
    net_layer_refdec(&other->layer);
  } else {
    assert(!other->pipe);
    mem_free(pipe);
  }
}

static
void net_pipe_layer_task_handle(struct net_layer_s *layer,
                            struct net_task_s *task)
{
  struct pipe_end_s *end = pipe_end_s_from_layer(layer);
  struct net_pipe_s *pipe = end->pipe;
  size_t me = end - pipe->end;
  struct pipe_end_s *other = &pipe->end[!me];
  struct net_task_s *t;

  assert(pipe);

  switch (task->type) {
  default:
    break;

  case NET_TASK_OUTBOUND:
    if (!other->child || !other->pipe)
      break;

    t = net_scheduler_task_alloc(other->layer.scheduler);
    if (!f)
      break;
    net_task_inbound_push(t, other->child, &other->layer,
                          0, &other->src_addr, NULL,
                          task->packet.buffer);
    break;
  }

  net_task_destroy(task);
}

static const struct net_layer_handler_s end_handler = {
  .destroyed = net_pipe_layer_destroyed,
  .task_handle = net_pipe_layer_task_handle,
};

error_t net_pipe_create(struct net_scheduler_s *scheduler,
                        const struct net_addr_s *addr0,
                        void *delegate0,
                        const struct net_layer_delegate_vtable_s *delegate_vtable0,
                        struct net_layer_s **layer0,
                        const struct net_addr_s *addr1,
                        void *delegate1,
                        const struct net_layer_delegate_vtable_s *delegate_vtable1,
                        struct net_layer_s **layer1)
{
  struct net_pipe_s *pipe = mem_alloc(sizeof(*pipe), mem_scope_sys);
  error_t err;

  if (!pipe)
    return -ENOMEM;

  memset(pipe, 0, sizeof(*pipe));

  err = net_layer_init(&pipe->end[0].layer, &end_handler, scheduler,
                       delegate0, delegate_vtable0);
  if (err)
    goto err_free;

  err = net_layer_init(&pipe->end[1].layer, &end_handler, scheduler,
                       delegate1, delegate_vtable1);
  if (err)
    goto err_deinit0;

  // end 0 holds the memory buffer.
  net_layer_refinc(&pipe->end[0].layer);

  pipe->end[0].pipe = pipe;
  pipe->end[1].pipe = pipe;
  pipe->end[0].src_addr = addr0;
  pipe->end[1].src_addr = addr1;

  *layer0 = &pipe->end[0].layer;
  *layer1 = &pipe->end[1].layer;
  
  return 0;

 err_deinit0:
  net_layer_refdec(&pipe->end[0].layer);
  return err;

 err_free:
  mem_free(pipe);
  return err;
}

