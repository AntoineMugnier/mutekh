#include <assert.h>

#include <stdio.h>
#include <enums.h>

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>

#include <ble/stack/context.h>
#include <ble/stack/central.h>
#include <ble/net/att.h>
#include <ble/net/llcp.h>
#include <ble/protocol/error.h>
#include <ble/protocol/gap.h>
#include <ble/protocol/company.h>
#include <ble/gatt/service.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/net.h>
#include <device/class/timer.h>
#if defined(HAS_KEYBOARD)
#include <device/class/valio.h>
#include <device/valio/keyboard.h>
#endif

#include <hexo/bit.h>
#include <hexo/power.h>

#include <ble/profile/hid/client.h>
#include "led.h"

enum led_id_e
{
    LED_GREEN,
    LED_BLUE,
    LED_RED,
    LED_COUNT,
};

struct app_s {
  struct ble_stack_context_s context;
  struct ble_central_s central;
  struct persist_config persist;
  struct net_layer_s *client;
#if defined(HAS_KEYBOARD)
  struct device_valio_s button;
  struct dev_valio_rq_s button_rq;
  uint8_t button_state;
  dev_timer_value_t last_press;
#endif
  struct device_timer_s timer;
  struct led_s led[LED_COUNT];
};

STRUCT_COMPOSE(app_s, context);
STRUCT_COMPOSE(app_s, central);

static void client_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct app_s *app = delegate;

  printk("Client layer released\n");
  if (layer != app->client)
    return;

  app->client = NULL;
}

static
void app_descriptor_discovered(void *delegate, struct net_layer_s *layer, 
                                const struct hid_client_descriptor_s *desc)
{
  struct app_s *app = delegate;

  printk("%s\n", __FUNCTION__);

  led_blink(&app->led[LED_BLUE], 250, 0, 0);

  printk("%s descriptor %d %P\n", __FUNCTION__, desc->descriptor_size, desc->descriptor, desc->descriptor_size);
}

static
void app_discovery_error(void *delegate, struct net_layer_s *layer)
{
  struct app_s *app = delegate;

  printk("%s\n", __FUNCTION__);

  led_blink(&app->led[LED_BLUE], 250, 0, 0);
  led_blink(&app->led[LED_RED], 250, 250, 50);
}

static
void app_should_encrypt(void *delegate, struct net_layer_s *layer)
{
  printk("%s\n", __FUNCTION__);
}

static
void app_report_changed(void *delegate, struct net_layer_s *layer,
                        uint8_t report_id,
                        const void *data, size_t size)
{
  struct app_s *app = delegate;

  printk("%s %d %P\n", __FUNCTION__, report_id, data, size);

  led_blink(&app->led[LED_GREEN], 10, 0, 0);
}

static
void app_report_read_done(void *delegate, struct net_layer_s *layer,
                          const void *data, size_t size)
{
  struct app_s *app = delegate;

  (void)app;
  
  printk("%s %P\n", __FUNCTION__, data, size);
}

static
void app_report_read_error(void *delegate, struct net_layer_s *layer)
{
  struct app_s *app = delegate;

  (void)app;

  printk("%s\n", __FUNCTION__);
}

static const struct hid_client_delegate_vtable_s client_app_vtable =
{
  .base.release = client_destroyed,
  .discovery_error = app_discovery_error,
  .should_encrypt = app_should_encrypt,
  .descriptor_discovered = app_descriptor_discovered,
  .report_changed = app_report_changed,
  .report_read_done = app_report_read_done,
  .report_read_error = app_report_read_error,
};

static
void app_pairing_requested(struct ble_stack_connection_s *conn,
                           bool_t bonding)
{
  ble_stack_connection_pairing_accept(conn, 0, 0, 0);
}

static
void app_pairing_failed(struct ble_stack_connection_s *conn,
			enum sm_reason reason)
{
  struct ble_central_s *ctrl = ble_central_s_from_conn(conn);
  struct app_s *app = app_s_from_central(ctrl);
  ble_stack_connection_drop(&app->central.conn);
}

static void app_client_create(struct app_s *app)
{
  error_t err;

  err = hid_client_create(&app->context.scheduler, NULL, app, &client_app_vtable.base, &app->client);
  if (err) {
    printk("Unable to create remote client layer: %d\n", err);
    return;
  }

  uint16_t proto = BLE_ATT_CLIENT;
  err = net_layer_bind(app->central.conn.att, &proto, app->client);
  if (err) {
    printk("error while binding remote client to att: %d\n", err);
    net_layer_refdec(app->client);
    app->client = NULL;
    return;
  }

  net_layer_refdec(app->client);
}

static
void app_bonding_success(struct ble_stack_connection_s *conn)
{
  struct ble_central_s *ctrl = ble_central_s_from_conn(conn);
  struct app_s *app = app_s_from_central(ctrl);

  printk("Bonding success, creating remote client layer\n");

  led_blink(&app->led[LED_GREEN], 250, 250, 3);

  ble_llcp_encryption_restart(app->central.conn.llcp);

  app_client_create(app);
}

static
void app_connection_closed(struct ble_stack_connection_s *conn,
                           uint8_t reason)
{
  struct ble_central_s *ctrl = ble_central_s_from_conn(conn);
  struct app_s *app = app_s_from_central(ctrl);

  ble_central_mode_set(&app->central, BLE_CENTRAL_CONNECTABLE);
}

static
void app_connection_opened(struct ble_central_s *ctrl,
                           const struct ble_addr_s *addr)
{
  struct app_s *app = app_s_from_central(ctrl);

  (void)app;
  printk("Central connection opened to "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(addr));
}

static
void app_state_changed(struct ble_central_s *ctrl,
                       enum ble_central_state_e state)
{
  struct app_s *app = app_s_from_central(ctrl);

  printk("App %p ctrl state now %d\n", app, state);

  led_blink(&app->led[LED_RED], 0, 0, 0);

  switch (state) {
  case BLE_CENTRAL_IDLE:
    led_blink(&app->led[LED_BLUE], 50, 50, 0);
    break;

  case BLE_CENTRAL_SCANNING:
    led_blink(&app->led[LED_BLUE], 10, 1990, -1);
    break;

  case BLE_CENTRAL_PAIRING:
    led_blink(&app->led[LED_BLUE], 500, 1500, -1);
    break;

  case BLE_CENTRAL_CONNECTED:
    led_blink(&app->led[LED_BLUE], 50, 50, 0);

    if (app->central.mode & BLE_CENTRAL_PAIRABLE) {
        printk("We are pairing...\n");
        ble_stack_connection_pairing_request(&app->central.conn, 0, 0);
    } else {
        printk("We are bonded already...\n");
        app_client_create(app);
    }
    break;
  }
}

static
enum ble_scan_filter_policy_e app_device_policy(struct ble_central_s *ctrl,
                                                const struct ble_scan_filter_device_s *device)
{
  struct app_s *app = app_s_from_central(ctrl);

  printk("From random "BLE_ADDR_FMT" known %d RSSI %d\n", BLE_ADDR_ARG(&device->addr), device->known, device->rssi / 8);

  if (device->known && app->central.last_state != BLE_CENTRAL_PAIRING)
    return BLE_SCAN_FILTER_CONNECT;

  if (app->central.last_state != BLE_CENTRAL_PAIRING)
    return BLE_SCAN_FILTER_MONITOR;

  if (device->rssi < -70 * 8)
    return BLE_SCAN_FILTER_MONITOR;

  for (const uint8_t *ad = device->ad; ad < device->ad + device->ad_len; ad += *ad + 1) {
    const uint8_t *data = ad + 2;
    const uint8_t data_size = ad[0] - 1;

    switch (ad[1]) {
    case BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE:
    case BLE_GAP_UUID16_SERVICE_LIST_COMPLETE:
      printk(" srv");
      for (uint8_t i = 0; i < data_size; i += 2) {
        struct ble_uuid_s uuid;
        ble_uuid_bluetooth_based(&uuid, endian_le16_na_load(ad + i));
        printk(" "BLE_UUID_FMT, BLE_UUID_ARG(&uuid));

        if (endian_le16_na_load(data + i) == BLE_GATT_SERVICE_HUMAN_INTERFACE_DEVICE) {
          printk("Found HID device...\n");
          return BLE_SCAN_FILTER_CONNECT;
        }
      }

      break;
    }
  }

  if (device->scanned)
    return BLE_SCAN_FILTER_MONITOR;
  else
    return BLE_SCAN_FILTER_SCAN;
}

static const struct ble_central_handler_s app_central_handler =
{
  .base.pairing_requested = app_pairing_requested,
  .base.pairing_failed = app_pairing_failed,
  .base.bonding_success = app_bonding_success,
  .base.connection_closed = app_connection_closed,
  .state_changed = app_state_changed,
  .connection_opened = app_connection_opened,
  .device_policy = app_device_policy,
};

#if defined(HAS_KEYBOARD)
static KROUTINE_EXEC(button_changed)
{
  struct app_s *app = KROUTINE_CONTAINER(kr, *app, button_rq.base.kr);

  printk("Button changed, state %02x\n", app->button_state);

  if (app->button_state) {
    led_blink(&app->led[LED_RED], 50, 950, 2);
    DEVICE_OP(&app->timer, get_value, &app->last_press, 0);
  } else {
    dev_timer_value_t now;
    dev_timer_delay_t duration, sec;

    DEVICE_OP(&app->timer, get_value, &now, 0);
    duration = now - app->last_press;
    dev_timer_init_sec(&app->timer, &sec, NULL, 1, 1);

    if (duration > sec) {
      led_blink(&app->led[LED_RED], 50, 50, 5);
      ble_central_mode_set(&app->central, BLE_CENTRAL_PAIRABLE | BLE_CENTRAL_CONNECTABLE);
    }

    printk("Dropping connection\n");
    ble_stack_connection_drop(&app->central.conn);
  }

  DEVICE_OP(&app->button, request, &app->button_rq);
}
#endif

static const struct persist_config persist_config = {
  .dev_addr = CONFIG_LOAD_ROM_RO_SIZE - 4096,
  .dev_size = 4096,
  .page_size = 2048,
};


static CONTEXT_ENTRY(main)
{
  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;
  struct ble_central_params_s central_params = {
    .scan_interval_ms = 200,
    .scan_duration_ms = 190,
    .conn.interval_min = 8,
    .conn.interval_max = 12,
    .conn.latency = 99,
    .conn.timeout = 400,
  };

  ensure(app);

  memset(app, 0, sizeof(*app));

  err = ble_stack_context_init(&app->context, "/ble", "/rtc1", "/rng", "/aes", &persist_config);
  assert(!err && "ble_stack_context_init failed");

  err = ble_central_init(&app->central, &central_params, &app_central_handler, &app->context);
  assert(!err && "ble_central_init failed");

#if defined(HAS_KEYBOARD)
  err = device_get_accessor_by_path(&app->button.base, NULL,
                                    "keyboard", DRIVER_CLASS_VALIO);
  ensure(err == 0);
#endif
  
  err = device_get_accessor_by_path(&app->timer.base, NULL,
                                    "/rtc*", DRIVER_CLASS_TIMER);
  ensure(err == 0);

#if defined(HAS_KEYBOARD)
  app->button_rq.attribute = VALIO_KEYBOARD_MAP;
  app->button_rq.type = DEVICE_VALIO_WAIT_EVENT;
  app->button_rq.data = &app->button_state;
  dev_valio_rq_init(&app->button_rq, button_changed);
  DEVICE_OP(&app->button, request, &app->button_rq);
#endif
  
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    err = led_init(&app->led[i], i, 0);
    if (err) {
      printk("LED %d failed\n", i);
      assert(0);
    }
  }

  ble_central_mode_set(&app->central, BLE_CENTRAL_CONNECTABLE);

  printk("App %p Scanning started\n", app);
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 1024,
  };

  thread_create(main, 0, &attr);
}
