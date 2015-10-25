#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/protocol/gatt/characteristic.h>
#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gap.h>

#include "gap.h"

struct gap_preferren_conn_params_s {
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t latency;
    uint16_t timeout;
};

static const struct gap_preferren_conn_params_s params = {
    .interval_min = 20,
    .interval_max = 40,
    .latency = 19,
    .timeout = 200,
};

static const uint16_t appearance = BLE_GAP_APPEARANCE_UNKNOWN;

BLE_GATTDB_SERVICE_DECL(gap_service,
                      BLE_GATTDB_SERVICE_PRIMARY,
                      BLE_UUID_SHORT_P(BLE_UUID_GENERIC_ACCESS_SERVICE),
                      NULL,
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_SHORT_P(BLE_UUID_GAP_DEVICE_NAME_CHAR),
                          "Beacon config"),
                      BLE_GATTDB_CHAR_CONSTANT_BLOB(
                          BLE_UUID_SHORT_P(BLE_UUID_GAP_APPEARANCE_CHAR),
                          &appearance, sizeof(appearance)),
                      BLE_GATTDB_CHAR_CONSTANT_BLOB(
                          BLE_UUID_SHORT_P(BLE_UUID_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_CHAR),
                          &params, sizeof(params)),
                      );

error_t gap_service_register(struct ble_gattdb_registry_s *reg,
                             struct ble_gattdb_s *db)
{
    return ble_gattdb_service_register(reg, db, &gap_service);
}
