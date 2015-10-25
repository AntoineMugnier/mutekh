#include <ble/gatt/db.h>
#include <ble/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>
#include <ble/protocol/gatt/service.h>

#include "dis.h"

BLE_GATT_SERVICE_DECL(dis_service,
                      BLE_GATT_SERVICE_PRIMARY,
                      BLE_UUID_SHORT_P(BLE_UUID_DEVICE_INFORMATION_SERVICE),
                      NULL,
                      BLE_GATT_CHAR_CONSTANT_STRING(
                          BLE_UUID_SHORT_P(BLE_UUID_MANUFACTURER_NAME_STRING_CHAR),
                          "MutekH"),
                      BLE_GATT_CHAR_CONSTANT_STRING(
                          BLE_UUID_SHORT_P(BLE_UUID_MODEL_NUMBER_STRING_CHAR),
                          "654321"),
                      );

error_t dis_service_register(struct ble_gatt_db_service_s *dbs,
                             struct ble_gatt_db_s *db)
{
    return ble_gatt_db_service_register(dbs, db, &dis_service);
}
