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

#include <mutek/printk.h>

#include <net/scheduler.h>
#include <net/layer.h>

#include "network.h"

static
void net_layer_context_update(struct net_layer_s *layer,
                              const struct net_layer_context_s *context)
{
  bool_t changed = 0;

  if (layer->handler->context_updated) {
    changed = layer->handler->context_updated(layer, context);
  } else if (memcmp(&layer->context, context, sizeof(*context))) {
    layer->context = *context;
    changed = 1;
  }

  //printk("Layer %d now %d+%d\n", &layer->handler->type,
  //       layer->context.mtu, layer->context.prefix_size);

  if (changed)
    net_layer_context_changed(layer);
}

void net_layer_context_changed(struct net_layer_s *layer)
{
  GCT_FOREACH(net_layer_list, &layer->children, child,
              net_layer_context_update(child, &layer->context);
              );
}

error_t net_layer_bind(
    struct net_layer_s *layer,
    void *addr,
    struct net_layer_s *child)
{
  assert(!child->parent);

  error_t err = -ENOTSUP;

  if (layer->handler->bound)
    err = layer->handler->bound(layer, addr, child);

  assert(!child->parent);

  if (err)
    return err;

  //printk("Layer %d bound to %d\n", &child->handler->type, &layer->handler->type);

  child->parent = layer;
  net_layer_list_pushback(&layer->children, child);

  net_layer_context_update(child, &layer->context);

  return 0;
}

void net_layer_unbind(
    struct net_layer_s *layer,
    struct net_layer_s *child)
{
  assert(child->parent == layer);

  //printk("Layer %d unbound from %d\n", &child->handler->type, &layer->handler->type);

  if (layer->handler->unbound)
    layer->handler->unbound(layer, child);

  child->parent = NULL;
  net_layer_list_remove(&layer->children, child);
}

error_t net_layer_init(
  struct net_layer_s *layer,
  const struct net_layer_handler_s *handler,
  struct net_scheduler_s *sched,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable)
{
  net_layer_refinit(layer);
  net_layer_list_init(&layer->children);

  layer->handler = handler;
  layer->scheduler = sched;
  layer->parent = NULL;
  layer->delegate = delegate;
  layer->delegate_vtable = delegate_vtable;

  assert((!delegate || delegate_vtable) && "Delegate must come with a vtable");

  net_layer_list_noref_pushback(&sched->layers, layer);

  /* printk("Layer %d init\n", &layer->handler->type); */

  if (layer->handler->use_timer)
    net_scheduler_timer_use(layer->scheduler);

  return 0;
}

void net_layer_destroy(
    struct net_layer_s *layer)
{
  net_layer_list_noref_remove(&layer->scheduler->layers, layer);
  net_scheduler_layer_destroyed(layer->scheduler, layer);
}

void net_layer_destroy_real(
    struct net_layer_s *layer)
{
  struct net_layer_s *child;
  struct net_scheduler_s *sched = layer->scheduler;
  const struct net_layer_handler_s *handler = layer->handler;
  void *delegate = layer->delegate;
  const struct net_layer_delegate_vtable_s *delegate_vtable = layer->delegate_vtable;

  layer->scheduler = NULL;

  /* printk("Layer %d destroy\n", &layer->handler->type); */

  while ((child = net_layer_list_head(&layer->children))) {
    net_layer_unbind(layer, child);
    net_layer_refdec(child);
  }

  net_scheduler_from_layer_cancel(sched, layer);

  if (delegate)
    delegate_vtable->release(delegate, layer);

  layer->delegate = NULL;

  handler->destroyed(layer);

  if (handler->use_timer)
    net_scheduler_timer_release(sched);
}

