#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>

#include "dis.h"

BLE_GATTDB_SERVICE_DECL(dis_service,
                        BLE_GATTDB_SERVICE_PRIMARY,
                        BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_DEVICE_INFORMATION),
                        NULL,
                        BLE_GATTDB_CHAR_CONSTANT_STRING(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MANUFACTURER_NAME_STRING),
                                                        "MutekH"),
                        BLE_GATTDB_CHAR_CONSTANT_STRING(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MODEL_NUMBER_STRING),
                                                        "BLE Midi device"),
                        );

error_t dis_service_register(struct ble_gattdb_registry_s *reg,
                             struct ble_gattdb_s *db)
{
  return ble_gattdb_service_register(reg, db, &dis_service);
}
