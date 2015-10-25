#ifndef BEACON_CONFIG_H_
#define BEACON_CONFIG_H_

#include <ble/gatt/db.h>
#include <ble/gatt/service.h>
#include <ble/stack/beacon.h>

struct beacon_config_handler_s;

struct beacon_config_s
{
  struct ble_gatt_db_service_s dbs;
  const struct beacon_config_handler_s *handler;
  struct ble_beacon_config_s config;
};

STRUCT_COMPOSE(beacon_config_s, dbs);

struct beacon_config_handler_s {
  void (*changed)(struct beacon_config_s *config);
};

error_t beacon_config_service_register(struct beacon_config_s *beacon_config,
                                       struct ble_gatt_db_s *db,
                                       const struct beacon_config_handler_s *handler);

#endif
