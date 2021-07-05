#ifndef BLE_PROFILE_FLUKE_CENTRAL_H_
#define BLE_PROFILE_FLUKE_CENTRAL_H_

#include <ble/stack/central.h>
#include <ble/profile/fluke/measurement.h>

struct fluke_central_s;
struct ble_stack_context_s;

struct fluke_central_vtable_s
{
  void (*connected)(struct fluke_central_s *client,
                    const struct ble_addr_s *addr);

  void (*disconnected)(
    struct fluke_central_s *client);

  void (*discovery_done)(
    struct fluke_central_s *client,
    const char *name,
    const char *serial,
    const char *fw_version);

  void (*measurement)(
    struct fluke_central_s *client,
    const struct fluke_measurement_s *measurement);
};

struct fluke_central_s
{
  struct ble_stack_context_s *context;
  struct ble_central_s central;
  struct ble_central_params_s central_params;
  struct net_layer_s *client;
  const struct fluke_central_vtable_s *vtable;
};

STRUCT_COMPOSE(fluke_central_s, central);

error_t fluke_central_init(struct fluke_central_s *client,
                           struct ble_stack_context_s *context,
                           const struct fluke_central_vtable_s *vtable);

error_t fluke_central_disconnect(struct fluke_central_s *client);

#endif
