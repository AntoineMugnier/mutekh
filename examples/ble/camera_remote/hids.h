#ifndef HIDS_H_
#define HIDS_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/timer.h>

struct hid_service_handler_s;

struct hid_service_s
{
  struct ble_gattdb_registry_s dbs;
  const struct hid_service_handler_s *handler;
  bool_t subscribed;
  bool_t suspended;
  bool_t enabled;
};

STRUCT_COMPOSE(hid_service_s, dbs);

struct hid_service_handler_s
{
  void (*enabled)(struct hid_service_s *hids, bool_t enabled);
};

error_t hid_service_register(struct hid_service_s *hids,
                             const struct hid_service_handler_s *handler,
                             struct ble_gattdb_s *db);

error_t hid_button_set(struct hid_service_s *hids, uint8_t button);

#endif
