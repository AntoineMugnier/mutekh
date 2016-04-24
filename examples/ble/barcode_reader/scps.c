#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>

#include "scps.h"

struct scan_parameters_s {
    uint16_t interval;
    uint16_t window;
};

static struct scan_parameters_s scan_parameters = {
    1600, // 1s
    160, // 100ms
};

BLE_GATTDB_SERVICE_DECL(scps_service,
                      BLE_GATTDB_SERVICE_PRIMARY,
                      BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_SCAN_PARAMETERS),
                      NULL,
                      BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_SCAN_INTERVAL_WINDOW),
                                    BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_PERM_AUTH_READ,
                                    BLE_GATTDB_CHAR_DATA_PLAIN(&scan_parameters,
                                                             sizeof(scan_parameters),
                                                             NULL, NULL)));

error_t scps_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db)
{
    return ble_gattdb_service_register(dbs, db, &scps_service);
}
