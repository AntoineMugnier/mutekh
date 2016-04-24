#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/descriptor.h>
#include <ble/gatt/hid.h>
#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include "hids.h"
#include "bas.h"

#define dprintk(...) do{}while(0)

GCT_CONTAINER_FCNS(kbd_fifo, static inline, kbd_fifo,
                   init, destroy, isempty, isfull, pop,
                   pop_array, pushback, pushback_array);

static const uint8_t report_map[] = {
#include "report_descriptor.h"
};

enum hid_char_id
{
    CHAR_ID_REPORT_MAP,
    CHAR_ID_HID_INFO,
    CHAR_ID_HID_CONTROL_POINT,
    CHAR_ID_KEYBOARD,
};

static const struct ble_gattdb_service_s *includes[] = {
    &batt_service, NULL,
};

static const uint8_t hid_info[] = {
    0x11, 0x1, 8, 2,
};

static void hids_use_update(struct hid_service_s *hids)
{
  dprintk("HIDS now %ssubscribed and %s\n",
         hids->subscribed ? "" : "un",
         hids->suspended ? "suspended" : "active");
  hids->enabled = hids->subscribed && !hids->suspended;
  hids->handler->enabled(hids, hids->enabled);
}

static
uint8_t on_kbd_subscribe(struct ble_gattdb_registry_s *reg,
                         uint8_t charid,
                         bool_t subscribed)
{
  struct hid_service_s *hids = hid_service_s_from_dbs(reg);

  hids->subscribed = subscribed;
  hids->suspended = 0;
  hids_use_update(hids);
  
  return 0;
}


static
uint8_t on_control_point_write(struct ble_gattdb_client_s *client,
                               struct ble_gattdb_registry_s *reg,
                               uint8_t charid,
                               const void *data, size_t size)
{
  struct hid_service_s *hids = hid_service_s_from_dbs(reg);

  hids->suspended = *(const uint8_t*)data;
  hids_use_update(hids);
  
  return 0;
}

static
uint8_t on_kbd_read(struct ble_gattdb_client_s *client,
                    struct ble_gattdb_registry_s *reg, uint8_t charid,
                    uint16_t offset,
                    void *data, size_t *size)
{
  ((uint8_t*)data)[0] = 0;
  ((uint8_t*)data)[1] = 0;
  *size = 2;

  return 0;
}

static
error_t on_kbd_stream(struct ble_gattdb_client_s *client,
                      struct ble_gattdb_registry_s *reg,
                      uint8_t charid,
                      struct buffer_s *buffer)
{
  struct hid_service_s *hids = hid_service_s_from_dbs(reg);
  error_t err;
  
  for (;;) {
    err = keymap_iterator_next(&hids->ki,
                               buffer->data + buffer->begin,
                               buffer->data + buffer->begin + 1);
    if (!err) {
      dprintk("%s %02x %02x\n", __FUNCTION__,
             buffer->data[buffer->begin],
             buffer->data[buffer->begin + 1]);

      buffer->end = buffer->begin + 2;
      hids->handler->transmitting(hids);
      return 0;
    }
    
    uint8_t tmp;
    size_t count = kbd_fifo_pop_array(&hids->kbd_fifo, &tmp, 1);
    if (!count)
      return -EAGAIN;

    keymap_iterator_set(&hids->ki, tmp);
  }
}

BLE_GATTDB_SERVICE_DECL(
  hid_service,
  BLE_GATTDB_SERVICE_PRIMARY | BLE_GATTDB_SERVICE_ADVERTISED,
  BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_HUMAN_INTERFACE_DEVICE),
  includes,

  [CHAR_ID_REPORT_MAP] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_REPORT_MAP),
      BLE_GATTDB_PERM_OTHER_READ,
      BLE_GATTDB_CHAR_DATA_CONSTANT(report_map, sizeof(report_map)),
      BLE_GATTDB_DESCRIPTORS(BLE_HID_EXTERNAL_REPORT_REFERENCE(BLE_GATT_CHAR_BATTERY_LEVEL))),

  [CHAR_ID_HID_INFO] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_HID_INFORMATION),
      BLE_GATTDB_PERM_OTHER_READ,
      BLE_GATTDB_CHAR_DATA_CONSTANT(hid_info, sizeof(hid_info))),

  [CHAR_ID_HID_CONTROL_POINT] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_HID_CONTROL_POINT),
      BLE_GATTDB_PERM_ENC_WRITE | BLE_GATTDB_PERM_ENC_READ,
      BLE_GATTDB_CHAR_DATA_DYNAMIC(NULL, on_control_point_write, NULL)),

  [CHAR_ID_KEYBOARD] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_REPORT),
      BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_PERM_ENC_READ,
      BLE_GATTDB_CHAR_DATA_STREAM(on_kbd_read, on_kbd_stream, on_kbd_subscribe),
      BLE_GATTDB_DESCRIPTORS(BLE_HID_INPUT_REPORT(1))),
  );

error_t hid_service_register(struct hid_service_s *hids,
                             const struct hid_service_handler_s *handler,
                             struct ble_gattdb_s *db,
                             const uint16_t *keymap)
{
  error_t err;

  err = ble_gattdb_service_register(&hids->dbs, db, &hid_service);
  if (err)
    return err;

  hids->subscribed = 0;
  hids->suspended = 0;
  hids->enabled = 0;
  hids->handler = handler;
  kbd_fifo_init(&hids->kbd_fifo);
  keymap_iterator_init(&hids->ki, keymap);
  
  return 0;
}

error_t hid_send_ascii(struct hid_service_s *hids,
                       const void *msg, size_t msg_size)
{
  dprintk("%s service %s: '%S'\n",
         __FUNCTION__,
         hids->enabled ? "enabled" : "disabled",
         msg, msg_size);

  kbd_fifo_pushback_array(&hids->kbd_fifo, (uint8_t*)msg, msg_size);

  ble_gattdb_char_stream_resume(&hids->dbs, CHAR_ID_KEYBOARD);

  return 0;
}

error_t hid_service_keymap_set(struct hid_service_s *hids,
                               const uint16_t *keymap)
{
  keymap_iterator_init(&hids->ki, keymap);
  return 0;
}
