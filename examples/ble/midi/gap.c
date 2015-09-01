#include <ble/gatt/db.h>
#include <ble/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>
#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gap.h>

#include "gap.h"

static const uint16_t appearance = BLE_GAP_APPEARANCE_UNKNOWN;

BLE_GATT_SERVICE_DECL(gap_service,
                      BLE_GATT_SERVICE_PRIMARY,
                      BLE_UUID_SHORT_P(BLE_UUID_GENERIC_ACCESS_SERVICE),
                      NULL,
                      BLE_GATT_CHAR_CONSTANT_STRING(
                          BLE_UUID_SHORT_P(BLE_UUID_GAP_DEVICE_NAME_CHAR),
                          "Midi keyboard"),
                      BLE_GATT_CHAR_CONSTANT_BLOB(
                          BLE_UUID_SHORT_P(BLE_UUID_GAP_APPEARANCE_CHAR),
                          &appearance, sizeof(appearance)),
                      );

error_t gap_service_register(struct ble_gatt_db_service_s *dbs,
                             struct ble_gatt_db_s *db)
{
    return ble_gatt_db_service_register(dbs, db, &gap_service);
}
