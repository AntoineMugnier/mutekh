#ifndef OAD_CLIENT_H_
#define OAD_CLIENT_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include "ids.h"

struct net_layer_delegate_vtable_s;
struct net_scheduler_s;
struct net_layer_s;

struct oad_gatt_client_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*discovery_done)(void *delegate, struct net_layer_s *layer,
                         const char *model,
                         const char *sw_version,
                         const char *fw_version,
                         size_t storage_size);

  void (*payload_request)(void *delegate, struct net_layer_s *layer,
                        uint32_t offset);
};

STRUCT_COMPOSE(oad_gatt_client_delegate_vtable_s, base);

struct oad_gatt_client_handler_s
{
  struct net_layer_handler_s base;

  error_t (*payload_set)(struct net_layer_s *layer,
                       uint32_t offset, const void *data);

  error_t (*command_set)(struct net_layer_s *layer,
                         uint8_t cmd, uint32_t arg);
};

STRUCT_COMPOSE(oad_gatt_client_handler_s, base);

error_t oad_gatt_client_create(struct net_scheduler_s *scheduler,
                              const void *params,
                              void *delegate,
                              const struct oad_gatt_client_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer);

static inline
void oad_gatt_client_payload_set(struct net_layer_s *layer,
                               uint32_t offset, const void *data)
{
    const struct oad_gatt_client_handler_s *handler = const_oad_gatt_client_handler_s_from_base(layer->handler);

    handler->payload_set(layer, offset, data);
}

static inline
void oad_gatt_client_command_set(struct net_layer_s *layer,
                                 uint8_t command, uint32_t arg)
{
    const struct oad_gatt_client_handler_s *handler = const_oad_gatt_client_handler_s_from_base(layer->handler);

    handler->command_set(layer, command, arg);
}

#endif
