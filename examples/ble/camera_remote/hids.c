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

uint8_t report_data = 0;

static const struct ble_gattdb_service_s *includes[] = {
    &batt_service, NULL,
};

static const uint8_t hid_info[] = {
    0x11, 0x01, 0x00, 0x01,
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
uint8_t hids_on_subscribe(struct ble_gattdb_registry_s *reg,
                         uint8_t charid,
                         bool_t subscribed)
{
  struct hid_service_s *hids = hid_service_s_from_dbs(reg);

  hids->subscribed = subscribed;
  hids->suspended = 0;
  hids_use_update(hids);
  
  return 0;
}

error_t hid_button_set(struct hid_service_s *hids, uint8_t button)
{
  report_data = button;

  ble_gattdb_char_changed(&hids->dbs, CHAR_ID_KEYBOARD, 1,
                          &report_data, sizeof(report_data));

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

BLE_GATTDB_SERVICE_DECL(
  hid_service,
  BLE_GATTDB_SERVICE_PRIMARY | BLE_GATTDB_SERVICE_ADVERTISED,
  BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_HUMAN_INTERFACE_DEVICE),
  includes,

  [CHAR_ID_REPORT_MAP] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_REPORT_MAP),
      BLE_GATTDB_PERM_OTHER_READ,
      BLE_GATTDB_CHAR_DATA_CONSTANT(report_map, sizeof(report_map)),
                    BLE_GATTDB_DESCRIPTORS(BLE_HID_EXTERNAL_REPORT_REFERENCE(BLE_GATT_CHAR_BATTERY_LEVEL))
                    ),

  [CHAR_ID_HID_INFO] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_HID_INFORMATION),
      BLE_GATTDB_PERM_OTHER_READ,
      BLE_GATTDB_CHAR_DATA_CONSTANT(hid_info, sizeof(hid_info))),

  [CHAR_ID_HID_CONTROL_POINT] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_HID_CONTROL_POINT),
      BLE_GATTDB_PERM_ENC_WRITE,
      BLE_GATTDB_CHAR_DATA_DYNAMIC(NULL, on_control_point_write, NULL)),

  [CHAR_ID_KEYBOARD] =
    BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_REPORT),
      BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_PERM_ENC_READ,
      BLE_GATTDB_CHAR_DATA_PLAIN(&report_data, sizeof(report_data), hids_on_subscribe, NULL),
      BLE_GATTDB_DESCRIPTORS(BLE_HID_INPUT_REPORT(1))),
  );

error_t hid_service_register(struct hid_service_s *hids,
                             const struct hid_service_handler_s *handler,
                             struct ble_gattdb_s *db)
{
  error_t err;

  err = ble_gattdb_service_register(&hids->dbs, db, &hid_service);
  if (err)
    return err;

  hids->subscribed = 0;
  hids->suspended = 0;
  hids->enabled = 0;
  hids->handler = handler;
  
  return 0;
}
