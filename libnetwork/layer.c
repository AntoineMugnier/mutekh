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

#define dprintk(...) do{}while(0)

void net_layer_context_changed(struct net_layer_s *layer)
{
  if (net_layer_list_isempty(&layer->children))
    return;

  struct net_layer_context_s cc = layer->context;

  if (layer->vtable->child_context_adjust)
    layer->vtable->child_context_adjust(layer, &cc);

  GCT_FOREACH(net_layer_list, &layer->children, child,
              child->context = cc;
              if (child->vtable->context_changed)
                child->vtable->context_changed(child);
              net_layer_context_changed(child);
              );
}

error_t net_layer_bind(
    struct net_layer_s *layer,
    void *addr,
    struct net_layer_s *child)
{
  assert(!child->parent);

  error_t err = -ENOTSUP;

  if (layer->vtable->bound)
    err = layer->vtable->bound(layer, addr, child);

  assert(!child->parent);

  if (err)
    return err;

  dprintk("Layer %p %p bound to %p %p\n", child, child->vtable,
        layer, layer->vtable);

  child->parent = layer;
  net_layer_list_pushback(&layer->children, child);

  child->context = layer->context;

  if (layer->vtable->child_context_adjust)
    layer->vtable->child_context_adjust(layer, &child->context);

  if (child->vtable->context_changed)
    child->vtable->context_changed(child);

  return 0;
}

void net_layer_unbind(
    struct net_layer_s *layer,
    struct net_layer_s *child)
{
  dprintk("Layer %p %p unbound from %p %p\n", child, child->vtable,
        layer, layer->vtable);

  assert(child->parent == layer);

  net_layer_list_remove(&layer->children, child);
  child->parent = NULL;

  if (child->vtable->dandling)
    child->vtable->dandling(child);

  if (layer->vtable->unbound)
    layer->vtable->unbound(layer, child);
}

void net_layer_unbind_all(struct net_layer_s *layer)
{
  struct net_layer_s *child;

  while ((child = net_layer_list_head(&layer->children))) {
    net_layer_unbind(layer, child);
    net_layer_refdec(child);
  }
}

error_t net_layer_init(
  struct net_layer_s *layer,
  const struct net_layer_vtable_s *vtable,
  struct net_scheduler_s *sched,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable)
{
  net_layer_refinit(layer);
  net_layer_list_init(&layer->children);

  layer->vtable = vtable;
  layer->scheduler = sched;
  layer->parent = NULL;
  layer->delegate = delegate;
  layer->delegate_vtable = delegate_vtable;

  assert((!delegate || delegate_vtable) && "Delegate must come with a vtable");

  net_scheduler_layer_created(sched, layer);
  
  dprintk("Layer %p %p init\n", layer, layer->vtable);

  net_scheduler_timer_use(layer->scheduler);

  return 0;
}

void net_layer_destroy(
    struct net_layer_s *layer)
{
  net_scheduler_layer_destroyed(layer->scheduler, layer);
}

void net_layer_destroy_real(
    struct net_layer_s *layer)
{
  struct net_layer_s *child;
  struct net_scheduler_s *sched = layer->scheduler;
  const struct net_layer_vtable_s *vtable = layer->vtable;
  void *delegate = layer->delegate;
  const struct net_layer_delegate_vtable_s *delegate_vtable = layer->delegate_vtable;

  layer->scheduler = NULL;

  dprintk("Layer %p %p destroy\n", layer, layer->vtable);

  while ((child = net_layer_list_head(&layer->children))) {
    dprintk(" cleaning ref to %p %p\n", layer, layer->vtable);
    net_layer_unbind(layer, child);
    net_layer_refdec(child);
  }

  net_scheduler_from_layer_cancel(sched, layer);

  if (delegate)
    delegate_vtable->release(delegate, layer);

  layer->delegate = NULL;

  vtable->destroyed(layer);

  net_scheduler_timer_release(sched);
}

