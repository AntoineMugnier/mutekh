#ifndef HIDS_H_
#define HIDS_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/timer.h>

#include "keymap_iterator.h"
#include <gct_platform.h>
#include <gct/container_ring.h>

#define GCT_CONTAINER_ALGO_kbd_fifo RING

GCT_CONTAINER_TYPES(kbd_fifo, uint8_t, 64);

struct hid_service_handler_s;

struct hid_service_s
{
  struct ble_gattdb_registry_s dbs;
  const struct hid_service_handler_s *handler;
  const uint16_t *keymap;
  struct keymap_iterator_s ki;
  kbd_fifo_root_t kbd_fifo;
  bool_t subscribed;
  bool_t suspended;
  bool_t enabled;
};

STRUCT_COMPOSE(hid_service_s, dbs);

struct hid_service_handler_s
{
  void (*enabled)(struct hid_service_s *hids, bool_t enabled);
  void (*transmitting)(struct hid_service_s *hids);
};

error_t hid_service_register(struct hid_service_s *hids,
                             const struct hid_service_handler_s *handler,
                             struct ble_gattdb_s *db,
                             const uint16_t *keymap);

error_t hid_service_keymap_set(struct hid_service_s *hids,
                               const uint16_t *keymap);

error_t hid_send_ascii(struct hid_service_s *hids,
                       const void *msg, size_t msg_size);

#endif
