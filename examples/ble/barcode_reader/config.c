#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>
#include <ble/gatt/descriptor.h>
#include <ble/protocol/att.h>
#include <mutek/printk.h>
#include "config.h"
#include "keymap.h"

static const struct dev_persist_descriptor_s barcode_config_blob = {
  .uid = 0x7700,
  .type = DEV_PERSIST_BLOB,
  .size = sizeof(struct barcode_config_s),
};

static void config_updated(struct config_service_s *cfg)
{
  cfg->handler->changed(cfg);

  printk("New config:\n");
  printk("  keymap %s\n", cfg->config.keymap);
  printk("  prefix %P\n", cfg->config.prefix, strlen(cfg->config.prefix));
  printk("  suffix %P\n", cfg->config.suffix, strlen(cfg->config.suffix));
  
  DEVICE_OP(&cfg->timer, cancel, &cfg->timer_rq);
  DEVICE_OP(&cfg->timer, request, &cfg->timer_rq);
}

static
uint8_t on_keymap_write(struct ble_gattdb_client_s *client,
                        struct ble_gattdb_registry_s *service,
                        uint8_t charid,
                        const void *data, size_t size)
{
  struct config_service_s *cfg = config_service_s_from_reg(service);
  char tmp[sizeof(cfg->config.keymap)];

  if (size >= sizeof(cfg->config.keymap) - 1)
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

  memcpy(tmp, data, size);
  tmp[size] = '\0';
    
  if (!keymap_get(tmp))
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    
  memcpy(cfg->config.keymap, tmp, size + 1);
  config_updated(cfg);

  return 0;
}

static
uint8_t on_keymap_read(struct ble_gattdb_client_s *client,
                       struct ble_gattdb_registry_s *service, uint8_t charid,
                       uint16_t offset,
                       void *data, size_t *size)
{
  struct config_service_s *cfg = config_service_s_from_reg(service);

  *size = strlen(cfg->config.keymap);
  memcpy(data, cfg->config.keymap, *size);
  
  return 0;
}

static
uint8_t on_prefix_read(struct ble_gattdb_client_s *client,
                       struct ble_gattdb_registry_s *service, uint8_t charid,
                       uint16_t offset,
                       void *data, size_t *size)
{
  struct config_service_s *cfg = config_service_s_from_reg(service);

  *size = strlen(cfg->config.prefix);
  memcpy(data, cfg->config.prefix, *size);
  
  return 0;
}

static
uint8_t on_suffix_read(struct ble_gattdb_client_s *client,
                       struct ble_gattdb_registry_s *service, uint8_t charid,
                       uint16_t offset,
                       void *data, size_t *size)
{
  struct config_service_s *cfg = config_service_s_from_reg(service);

  *size = strlen(cfg->config.suffix);
  memcpy(data, cfg->config.suffix, *size);
  
  return 0;
}

static
uint8_t on_prefix_write(struct ble_gattdb_client_s *client,
                        struct ble_gattdb_registry_s *service,
                        uint8_t charid,
                        const void *data, size_t size)
{
  struct config_service_s *cfg = config_service_s_from_reg(service);

  if (size >= sizeof(cfg->config.prefix) - 1)
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

  memcpy(cfg->config.prefix, data, size);
  cfg->config.prefix[size] = '\0';
  config_updated(cfg);

  return 0;
}

static
uint8_t on_suffix_write(struct ble_gattdb_client_s *client,
                        struct ble_gattdb_registry_s *service,
                        uint8_t charid,
                        const void *data, size_t size)
{
  struct config_service_s *cfg = config_service_s_from_reg(service);

  if (size >= sizeof(cfg->config.suffix) - 1)
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

  memcpy(cfg->config.suffix, data, size);
  cfg->config.suffix[size] = '\0';
  config_updated(cfg);

  return 0;
}

BLE_GATTDB_SERVICE_DECL(config_service,
                      BLE_GATTDB_SERVICE_PRIMARY | BLE_GATTDB_SERVICE_ADVERTISED,
                      BLE_UUID_P(0x92547a69, 0x303b, 0x489c, 0x8bbf, 0x547ff4a512e6ULL),
                      NULL,
                      BLE_GATTDB_CHAR(BLE_UUID_P(0x4767874a, 0x86db, 0x4d69, 0x9f5c, 0xd27fcf59111cULL),
                                    BLE_GATTDB_PERM_ENC_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_keymap_read, on_keymap_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Keymap"))),
                      BLE_GATTDB_CHAR(BLE_UUID_P(0xd6355b7d, 0x929a, 0x4345, 0xacf8, 0xb661405b761dULL),
                                    BLE_GATTDB_PERM_ENC_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_prefix_read, on_prefix_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Prefix"))),
                      BLE_GATTDB_CHAR(BLE_UUID_P(0x42449b9f, 0xf679, 0x4140, 0x9db0, 0x2e4395220b5fULL),
                                    BLE_GATTDB_PERM_ENC_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                    BLE_GATTDB_CHAR_DATA_DYNAMIC(on_suffix_read, on_suffix_write, NULL),
                                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Suffix"))),
                      );

static KROUTINE_EXEC(config_committed)
{
  struct config_service_s *cfg = KROUTINE_CONTAINER(kr, *cfg, persist_rq.rq.kr);

  cfg->busy = 0;
}

static KROUTINE_EXEC(config_commit)
{
  struct config_service_s *cfg = KROUTINE_CONTAINER(kr, *cfg, timer_rq.rq.kr);

  if (cfg->busy)
    return;

  dev_persist_wait_write(&cfg->persist, &barcode_config_blob, 0, &cfg->config);
}

error_t config_service_register(struct config_service_s *cfg,
                                struct ble_gattdb_s *db,
                                const struct config_service_handler_s *handler)
{
  const struct barcode_config_s *config;
  error_t err;
  
  memset(cfg, 0, sizeof(*cfg));

  err = device_get_accessor_by_path(&cfg->persist.base, NULL, "/nvmc", DRIVER_CLASS_PERSIST);
  if (err)
    return err;

  err = device_get_accessor_by_path(&cfg->timer.base, NULL, "/rtc*", DRIVER_CLASS_TIMER);
  if (err)
    return err;

  dev_timer_init_sec(&cfg->timer, &cfg->timer_rq.delay, NULL, 5, 0);
  kroutine_init_sched_switch(&cfg->timer_rq.rq.kr, config_commit);
  kroutine_init_sched_switch(&cfg->persist_rq.rq.kr, config_committed);
  cfg->persist_rq.descriptor = &barcode_config_blob;
  cfg->persist_rq.op = DEV_PERSIST_WRITE;
  
  cfg->handler = handler;

  err = dev_persist_wait_read(&cfg->persist,
                              &barcode_config_blob,
                              0, (const void**)&config);
  if (!err) {
    memcpy(&cfg->config, config, sizeof(*config));
  } else {
    strcpy(cfg->config.keymap, "fr");
    cfg->config.prefix[0] = '\0';
    cfg->config.suffix[0] = '\n';
    cfg->config.suffix[1] = '\0';
  }

  return ble_gattdb_service_register(&cfg->reg, db, &config_service);
}
