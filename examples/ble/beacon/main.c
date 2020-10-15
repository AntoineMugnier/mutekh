#include <stdlib.h>

#include <mutek/thread.h>
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>

#include <device/device.h>
#include <device/driver.h>

#if defined(CONFIG_BLE_SECURITY_DB)
  #include <persist/persist.h>
#endif



#include <hexo/power.h>

#include <ble/stack/context.h>
#include <ble/stack/peripheral.h>

#include "dis.h"
#include "gap.h"
#include "led.h"
#include "beacon_config.h"

#define dprintk(...) do{}while(0)

static const struct persist_descriptor_s beacon_config_blob = {
  .uid = 0x7700,
  .type = PERSIST_BLOB,
  .size = sizeof(struct ble_beacon_config_s),
};

struct app_s
{
  struct ble_stack_context_s context;
  struct ble_peripheral_s peripheral;
  struct ble_gattdb_registry_s gap;
  struct ble_gattdb_registry_s dis;
  struct beacon_config_s beacon_config;
  struct led_s led[3];
  struct net_layer_s *beaconer;
#if defined(CONFIG_BLE_SECURITY_DB)
  struct persist_context_s persist;
  bool_t config_changed;
#endif
};

STRUCT_COMPOSE(app_s, beacon_config);
STRUCT_COMPOSE(app_s, peripheral);

#if defined(CONFIG_BLE_CRYPTO)
static
void peri_pairing_requested(struct ble_stack_connection_s *conn, bool_t bonding)
{
  ble_stack_connection_pairing_accept(conn, 0, 0, 0);
}

static
void peri_pairing_failed(struct ble_stack_connection_s *conn, enum sm_reason reason)
{
}

static
void peri_bonding_success(struct ble_stack_connection_s *conn)
{
}
#endif

static
bool_t peri_connection_requested(struct ble_peripheral_s *peri, const struct ble_addr_s *addr)
{
  return 1;
}

static
void peri_connection_closed(struct ble_stack_connection_s *conn, uint8_t reason)
{
}

static void config_save(struct app_s *app)
{
#if defined(CONFIG_DEVICE_PERSIST)
  persist_wait_write(&app->persist,
                     &beacon_config_blob,
                     0, &app->beacon_config.config);

  app->config_changed = 0;
#endif
}

static void config_load(struct app_s *app)
{
#if defined(CONFIG_DEVICE_PERSIST)
  error_t err;
  const struct ble_beacon_config_s *config;

  err = persist_wait_read(&app->persist,
                          &beacon_config_blob,
                          0, (const void**)&config);

  if (!err) {
    memcpy(&app->beacon_config.config, config, sizeof(*config));
    return;
  }
#endif

  app->beacon_config.config.phy = BLE_PHY_1M;
  memcpy(&app->beacon_config.config.group_uuid,
         BLE_UUID_P(0x431a3f31, 0x6c68, 0x45e8, 0x9e4c, 0xfcb992494632ULL), 16);
  app->beacon_config.config.major = 42;
  app->beacon_config.config.minor = 1;
  app->beacon_config.config.one_meter_rssi = -65;
  app->beacon_config.config.interval_ms = 900;
}

static
void peri_state_changed(struct ble_peripheral_s *peri, enum ble_peripheral_state_e state)
{
  struct app_s *app = app_s_from_peripheral(peri);

#if defined(CONFIG_DEVICE_PERSIST)
  if (app->config_changed) {
    config_save(app);
  }
#endif

  switch (state) {
  case BLE_PERIPHERAL_IDLE:
    led_blink(&app->led[2], 0, 100, 1);
    break;

  case BLE_PERIPHERAL_RECONNECTING:
  case BLE_PERIPHERAL_ADVERTISING:
    break;

  case BLE_PERIPHERAL_CONNECTED:
    led_blink(&app->led[2], 50, 1950, 5000);
    break;
  }
}

static const struct ble_peripheral_handler_s peri_handler =
{
#if defined(CONFIG_BLE_CRYPTO)
  .base.pairing_requested = peri_pairing_requested,
  .base.pairing_failed = peri_pairing_failed,
  .base.bonding_success = peri_bonding_success,
#endif
  .connection_requested = peri_connection_requested,
  .base.connection_closed = peri_connection_closed,
  .state_changed = peri_state_changed,
};

static void config_changed(struct beacon_config_s *config)
{
  struct app_s *app = app_s_from_beacon_config(config);

  app->config_changed = 1;
  ble_beacon_update(app->beaconer, &config->config);
}

static const struct beacon_config_handler_s config_handler = {
  .changed = config_changed,
};

static const struct ble_peripheral_params_s peri_params = {
  .phy = BLE_PHY_1M,
  .adv_interval_ms = 1000,
};

static const struct persist_config persist_config = {
  .dev_addr = CONFIG_LOAD_ROM_RO_SIZE - 4096,
  .dev_size = 4096,
  .page_size = 2048,
};

static CONTEXT_ENTRY(main)
{
  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;
  struct ble_addr_s addr;

  printk("Beacon\n");

  memset(app, 0, sizeof(*app));

  err = ble_stack_context_init(&app->context, "/ble", "/rtc1", "/rng", "/aes", &persist_config);
  ensure(!err && "stack context failed");

  ble_stack_context_local_address_get(&app->context, &addr);
  printk("BLE address: "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&addr));

#if defined(CONFIG_DEVICE_PERSIST)
  err = device_get_accessor_by_path(&app->persist.base, NULL, "/nvmc", DRIVER_CLASS_PERSIST);
  ensure(!err && "persist device not found");
#endif

  err = ble_peripheral_init(&app->peripheral, &peri_params, &peri_handler, &app->context);
  ensure(!err && "peripheral init failed");

  err = dis_service_register(&app->dis, &app->context.gattdb);
  ensure(!err && "DIS init failed");
  err = gap_service_register(&app->gap, &app->context.gattdb);
  ensure(!err && "GAPs init failed");
  err = beacon_config_service_register(&app->beacon_config, &app->context.gattdb, &config_handler);
  ensure(!err && "Beacon config service init failed");

  err = led_init(&app->led[0], 16, 0);
  ensure(!err && "LED failed");
  err = led_init(&app->led[1], 12, 0);
  ensure(!err && "LED failed");
  err = led_init(&app->led[2], 15, 0);
  ensure(!err && "LED failed");

  led_blink(&app->led[1], 25, 4975, 5000);

  ble_peripheral_mode_set(&app->peripheral, 0
                          | BLE_PERIPHERAL_DISCOVERABLE
                          | BLE_PERIPHERAL_PAIRABLE
                          | BLE_PERIPHERAL_CONNECTABLE);

  config_load(app);
  ble_stack_context_address_non_resolvable_generate(
         &app->context, &app->beacon_config.config.local_addr);

  err = ble_beacon_create(&app->context, &app->beacon_config.config, &app->beaconer);
  ensure(!err && "beaconer create failed");
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 0x800,
  };

  thread_create(main, 0, &attr);
}
