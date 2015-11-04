#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/protocol/gatt/characteristic.h>
#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gatt/descriptor.h>
#include <ble/protocol/att.h>
#include <mutek/printk.h>
#include "beacon_config.h"

static
uint8_t on_beacon_major_write(struct ble_gattdb_client_s *client,
                              struct ble_gattdb_registry_s *service,
                              uint8_t charid,
                              const void *data, size_t size)
{
    struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

    if (size != 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    
    cfg->config.major = endian_le16_na_load(data);

    cfg->handler->changed(cfg);
    
    return 0;
}

static
uint8_t on_beacon_minor_write(struct ble_gattdb_client_s *client,
                              struct ble_gattdb_registry_s *service,
                              uint8_t charid,
                              const void *data, size_t size)
{
    struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

    if (size != 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    
    cfg->config.minor = endian_le16_na_load(data);

    cfg->handler->changed(cfg);

    return 0;
}

static
uint8_t on_beacon_interval_write(struct ble_gattdb_client_s *client,
                              struct ble_gattdb_registry_s *service,
                              uint8_t charid,
                              const void *data, size_t size)
{
    struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

    if (size != 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    
    cfg->config.interval_ms = endian_le16_na_load(data);

    cfg->handler->changed(cfg);

    return 0;
}

static
uint8_t on_beacon_group_id_write(struct ble_gattdb_client_s *client,
                           struct ble_gattdb_registry_s *service,
                           uint8_t charid,
                           const void *data, size_t size)
{
    struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

    if (size != 16)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    
    memcpy(&cfg->config.group_uuid, data, 16);

    cfg->handler->changed(cfg);

    return 0;
}

static
uint8_t on_beacon_rssi_write(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *service,
                             uint8_t charid,
                             const void *data, size_t size)
{
    struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

    if (size != 1)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    
    cfg->config.one_meter_rssi = *(int8_t *)data;

    cfg->handler->changed(cfg);

    return 0;
}

static
uint8_t on_beacon_major_read(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *service, uint8_t charid,
                             uint16_t offset,
                             void *data, size_t *size)
{
  struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

  endian_le16_na_store(data, cfg->config.major);
  *size = 2;

  return 0;
}

static
uint8_t on_beacon_minor_read(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *service, uint8_t charid,
                             uint16_t offset,
                             void *data, size_t *size)
{
  struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

  endian_le16_na_store(data, cfg->config.minor);
  *size = 2;

  return 0;
}

static
uint8_t on_beacon_interval_read(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *service, uint8_t charid,
                             uint16_t offset,
                             void *data, size_t *size)
{
  struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

  endian_le16_na_store(data, cfg->config.interval_ms);
  *size = 2;

  return 0;
}

static
uint8_t on_beacon_group_id_read(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *service, uint8_t charid,
                             uint16_t offset,
                             void *data, size_t *size)
{
  struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

  memcpy(data, &cfg->config.group_uuid, 16);
  *size = 16;

  return 0;
}

static
uint8_t on_beacon_rssi_read(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *service, uint8_t charid,
                             uint16_t offset,
                             void *data, size_t *size)
{
  struct beacon_config_s *cfg = beacon_config_s_from_reg(service);

  *(int8_t*)data = cfg->config.one_meter_rssi;
  *size = 1;

  return 0;
}

BLE_GATTDB_SERVICE_DECL(beacon_config_service,
                      BLE_GATTDB_SERVICE_PRIMARY | BLE_GATTDB_SERVICE_ADVERTISED,
                      BLE_UUID_P(0x729a0a6d, 0xf667, 0x4ad4, 0x9805, 0xa1e5d0475707ULL),
                      NULL,
                      BLE_GATTDB_CHAR(BLE_UUID_P(0x6421b265, 0xf3ec, 0x47ec, 0x9f76, 0x02e9a3623664ULL),
                                    BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_beacon_major_read, on_beacon_major_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Major"))),
                      BLE_GATTDB_CHAR(BLE_UUID_P(0xfdcfbddf, 0x4e64, 0x4c8d, 0xbdc3, 0x0bc6a3abace4ULL),
                                    BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_beacon_minor_read, on_beacon_minor_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Minor"))),
                      BLE_GATTDB_CHAR(BLE_UUID_P(0x217c13c2, 0x1845, 0x4e0c, 0xb790, 0xcabe4dc7eb14ULL),
                                    BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_beacon_group_id_read, on_beacon_group_id_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Group UUID"))),
                      BLE_GATTDB_CHAR(BLE_UUID_P(0xe7346771, 0xfa3a, 0x4034, 0xb363, 0xe2482f0bc318ULL),
                                    BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_beacon_rssi_read, on_beacon_rssi_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("RSSI @1m"))),
                      BLE_GATTDB_CHAR(BLE_UUID_P(0x67c768df, 0x986f, 0x4779, 0x87d7, 0x7b95f4f151e0ULL),
                                    BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_beacon_interval_read, on_beacon_interval_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Interval"))),
                      );

error_t beacon_config_service_register(struct beacon_config_s *cfg,
                                       struct ble_gattdb_s *db,
                                       const struct beacon_config_handler_s *handler)
{
    cfg->handler = handler;

    return ble_gattdb_service_register(&cfg->reg, db, &beacon_config_service);
}
