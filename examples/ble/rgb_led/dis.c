#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>

#include "dis.h"

#if 0
/* anchor long_service_decl */
// A Characteristic array
static const
struct ble_gattdb_characteristic_s dis_service_characteristic_array[] =
{
  {
    .type = BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MANUFACTURER_NAME_STRING),
    .permissions = BLE_GATTDB_PERM_OTHER_READ,
    .mode = BLE_GATTDB_CHARACTERISTIC_CONSTANT,
    .data.constant.data = "MutekH",
    .data.constant.size = 6,
  }, {
    .type = BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MODEL_NUMBER_STRING),
    .permissions = BLE_GATTDB_PERM_OTHER_READ,
    .mode = BLE_GATTDB_CHARACTERISTIC_CONSTANT,
    .data.constant.data = "BLE RGB Led",
    .data.constant.size = 11,
  },
};

// Service definition object
const struct ble_gattdb_service_s dis_service = {
  .flags = BLE_GATTDB_SERVICE_PRIMARY,
  .type = BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_DEVICE_INFORMATION),
  .include = NULL,
  .characteristic = dis_service_characteristic_array,
  .characteristic_count = 2,
};
/* anchor end */
#else
/* anchor service_decl */
BLE_GATTDB_SERVICE_DECL(
  // Object name
  dis_service,
  // Flags
  BLE_GATTDB_SERVICE_PRIMARY,
  // Type
  BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_DEVICE_INFORMATION),
  // Inclusion list
  NULL,
  // All remaining are characteristics
  BLE_GATTDB_CHAR_CONSTANT_STRING(
    BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MANUFACTURER_NAME_STRING),
    "MutekH"),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
    BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MODEL_NUMBER_STRING),
    "BLE RGB Led"),
  );
/* anchor end */
#endif

error_t dis_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db)
{
    return ble_gattdb_service_register(dbs, db, &dis_service);
}
