#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>
#include <ble/gatt/dis.h>

#include "dis.h"

static const struct ble_dis_pnp_id_s pnp_id = {
    .id_source = 2,
    .vendor_id = 0x10eb,
    .product_id = 0x27,
    .version = 0x100,
};

BLE_GATTDB_SERVICE_DECL(
  dis_service, BLE_GATTDB_SERVICE_PRIMARY,
  BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_DEVICE_INFORMATION),
  NULL,
  BLE_GATTDB_CHAR_CONSTANT_STRING(
    BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MANUFACTURER_NAME_STRING),
    "MutekH"),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
    BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MODEL_NUMBER_STRING),
    "MCR12-Adapter"),
  BLE_GATTDB_CHAR_CONSTANT_BLOB(
      BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_PNP_ID),
      &pnp_id, sizeof(pnp_id)),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
      BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_HARDWARE_REVISION_STRING),
      "1.0"),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
      BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_FIRMWARE_REVISION_STRING),
      "-git"),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
      BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_SERIAL_NUMBER_STRING),
      "01234"),
  );

error_t dis_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db)
{
  return ble_gattdb_service_register(dbs, db, &dis_service);
}
