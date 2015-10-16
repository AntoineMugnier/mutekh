#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>
#include <ble/protocol/gap.h>

#include "gap.h"

static const struct ble_gap_preferred_conn_params_s params = {
    .interval_min = 8,
    .interval_max = 20,
    .latency = 0,
    .timeout = 200,
};

static const uint16_t appearance = BLE_GAP_APPEARANCE_UNKNOWN;

BLE_GATTDB_SERVICE_DECL(gap_service,
                      BLE_GATTDB_SERVICE_PRIMARY,
                      BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_GENERIC_ACCESS),
                      NULL,
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_GAP_DEVICE_NAME),
                          "RGB Led"),
                      BLE_GATTDB_CHAR_CONSTANT_BLOB(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_GAP_APPEARANCE),
                          &appearance, sizeof(appearance)),
                      BLE_GATTDB_CHAR_CONSTANT_BLOB(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS),
                          &params, sizeof(params)),
                      );

error_t gap_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db)
{
    return ble_gattdb_service_register(dbs, db, &gap_service);
}
