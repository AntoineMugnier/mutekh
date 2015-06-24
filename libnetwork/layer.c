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

  //printk("Layer %S now %d+%d\n", &layer->type, 4,
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

  //printk("Layer %S bound to %S\n", &child->type, 4, &layer->type, 4);

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

  //printk("Layer %S unbound from %S\n", &child->type, 4, &layer->type, 4);

  if (layer->handler->unbound)
    layer->handler->unbound(layer, child);

  child->parent = NULL;
  net_layer_list_remove(&layer->children, child);
}

error_t net_layer_init(
    struct net_layer_s *layer,
    const struct net_layer_handler_s *handler,
    struct net_scheduler_s *sched,
    uint32_t type)
{
  net_layer_refinit(layer);
  net_layer_list_init(&layer->children);

  layer->handler = handler;
  layer->scheduler = sched;
  layer->parent = NULL;
  layer->type = type;

  //printk("Layer %S init\n", &layer->type, 4);

  return 0;
}

void net_layer_destroy(
    struct net_layer_s *layer)
{
  struct net_layer_s *child;
  struct net_scheduler_s *sched = layer->scheduler;

  layer->scheduler = NULL;

  //printk("Layer %S destroy\n", &layer->type, 4);

  while ((child = net_layer_list_head(&layer->children))) {
    net_layer_unbind(layer, child);
    net_layer_refdec(child);
  }

  layer->handler->destroyed(layer);

  net_scheduler_from_layer_cancel(sched, layer);
}

