#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>

#include "dis.h"

struct pnp_id_s {
    uint8_t id_source;
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
} __attribute__((packed));

static const struct pnp_id_s pnp_id = {
    .id_source = 2,
    .vendor_id = 0x10eb,
    .product_id = 0x29,
    .version = 0x100,
};

BLE_GATTDB_SERVICE_DECL(dis_service,
                      BLE_GATTDB_SERVICE_PRIMARY,
                      BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_DEVICE_INFORMATION),
                      NULL,
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MANUFACTURER_NAME_STRING),
                          "MutekH"),
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_MODEL_NUMBER_STRING),
                          "v1"),
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_FIRMWARE_REVISION_STRING),
                          "LOL"),
                      BLE_GATTDB_CHAR_CONSTANT_STRING(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_SOFTWARE_REVISION_STRING),
                          "42"),
                      BLE_GATTDB_CHAR_CONSTANT_BLOB(
                          BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_PNP_ID),
                          &pnp_id, sizeof(pnp_id)),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
      BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_SERIAL_NUMBER_STRING),
      "01234"),
                      );

error_t dis_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db)
{
    return ble_gattdb_service_register(dbs, db, &dis_service);
}
