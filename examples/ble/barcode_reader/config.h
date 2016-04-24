#ifndef CONFIG_H_
#define CONFIG_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/persist.h>
#include <device/class/timer.h>

struct barcode_config_s
{
  char keymap[8];
  char prefix[4];
  char suffix[4];
};

struct config_service_handler_s;

struct config_service_s
{
  struct ble_gattdb_registry_s reg;
  const struct config_service_handler_s *handler;
  struct barcode_config_s config;
  struct device_persist_s persist;
  struct dev_persist_rq_s persist_rq;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  bool_t busy;
};

STRUCT_COMPOSE(config_service_s, reg);

struct config_service_handler_s {
  void (*changed)(struct config_service_s *config);
};

error_t config_service_register(struct config_service_s *config,
                                       struct ble_gattdb_s *db,
                                       const struct config_service_handler_s *handler);

#endif
