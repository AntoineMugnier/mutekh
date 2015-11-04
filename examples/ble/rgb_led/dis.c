#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/protocol/gatt/characteristic.h>
#include <ble/protocol/gatt/service.h>

#include "dis.h"

struct pnp_id_s {
    uint8_t id_source;
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
} __attribute__((packed));

BLE_GATTDB_SERVICE_DECL(dis_service,
                      BLE_GATTDB_SERVICE_PRIMARY,
                      BLE_UUID_SHORT_P(BLE_UUID_DEVICE_INFORMATION_SERVICE),
                      NULL,
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_SHORT_P(BLE_UUID_MANUFACTURER_NAME_STRING_CHAR),
                          "MutekH"),
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_SHORT_P(BLE_UUID_MODEL_NUMBER_STRING_CHAR),
                          "BLE RGB Led"),
                      );

error_t dis_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db)
{
    return ble_gattdb_service_register(dbs, db, &dis_service);
}
