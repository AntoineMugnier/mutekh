#ifndef FLUKE_CLIENT_H_
#define FLUKE_CLIENT_H_

#include <hexo/decls.h>
#include <hexo/types.h>
#include <net/layer.h>

struct net_layer_delegate_vtable_s;
struct net_scheduler_s;
struct net_layer_s;

struct fluke_measurement_s;

struct fluke_gatt_client_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*discovery_done)(void *delegate, struct net_layer_s *layer,
                         const char *name,
                         const char *serial,
                         const char *fw_version);

  void (*data_changed)(void *delegate, struct net_layer_s *layer,
                       const struct fluke_measurement_s *measurement);
};

STRUCT_COMPOSE(fluke_gatt_client_delegate_vtable_s, base);

struct fluke_gatt_client_handler_s
{
  struct net_layer_handler_s base;
};

STRUCT_COMPOSE(fluke_gatt_client_handler_s, base);

error_t fluke_gatt_client_create(struct net_scheduler_s *scheduler,
                              const void *params,
                              void *delegate,
                              const struct fluke_gatt_client_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer);

#endif
