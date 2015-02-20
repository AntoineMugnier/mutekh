#include <mutek/printk.h>

#include <net/scheduler.h>
#include <net/layer.h>

static
bool_t net_layer_context_updated(struct net_layer_s *layer,
                                 const struct net_layer_context_s *context)
{
  if (!memcmp(&layer->context, context, sizeof(*context)))
    return 0;

  layer->context = *context;
  return 1;
}

void net_layer_context_changed(struct net_layer_s *layer)
{
  GCT_FOREACH(net_layer_list, &layer->children, child,
              do {
                bool_t (*updater)(struct net_layer_s *, const struct net_layer_context_s *);
                updater = child->handler->context_updated;
                if (!updater)
                  updater = net_layer_context_updated;

                if (updater(child, &layer->context))
                  net_layer_context_changed(child);
              } while (0);
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
    layer->handler->bound(layer, addr, child);

  assert(!child->parent);

  if (err)
    return err;

  //printk("Layer %S bound to %S\n", &child->type, 4, &layer->type, 4);

  child->parent = layer;
  net_layer_list_pushback(&layer->children, child);

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
    uint32_t type,
    uint8_t header_size)
{
  net_layer_refinit(layer);
  net_layer_list_init(&layer->children);

  layer->handler = handler;
  layer->scheduler = net_scheduler_refinc(sched);;
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
  net_scheduler_refdec(sched);
}

