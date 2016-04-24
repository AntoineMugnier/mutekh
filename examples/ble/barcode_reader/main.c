#include <stdlib.h>

#include <mutek/thread.h>
#include <mutek/startup.h>
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>

#include <ble/stack/context.h>
#include <ble/stack/peripheral.h>

#include "dis.h"
#include "gap.h"
#include "scps.h"
#include "hids.h"
#include "bas.h"
#include "led.h"
#include "mcr12.h"
#include "keymap.h"
#include "poweroff.h"
#include "config.h"

#define dprintk(...) do{}while(0)

enum led_e {
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  LED_COUNT,
};

static const uint8_t led_pin[LED_COUNT] = {9, 10, 11};

struct app_s
{
  // BLE Stack and services
  struct ble_stack_context_s context;
  struct ble_peripheral_s peripheral;
  struct ble_gattdb_registry_s gap;
  struct ble_gattdb_registry_s dis;
  struct ble_gattdb_registry_s scps;
  struct config_service_s config;
  struct hid_service_s hids;
  struct batt_s bas;

  // Barcode reader
  struct mcr12_s barcode_reader;

  // User Interface
  struct led_s led[LED_COUNT];
  struct device_valio_s button;
  struct dev_valio_rq_s button_rq;
  uint8_t button_state, button_last_state;

  // Timers
  struct device_timer_s timer;
  struct dev_timer_rq_s timeout_rq;

  dev_timer_value_t long_press_end;
  dev_timer_value_t pairing_end;
  dev_timer_value_t adv_end;
  dev_timer_value_t conn_end;

  dev_timer_delay_t long_press_time;
  dev_timer_delay_t adv_time;
  dev_timer_delay_t idle_time;
  dev_timer_delay_t second;

  bool_t off_already;
};

STRUCT_COMPOSE(app_s, barcode_reader);
STRUCT_COMPOSE(app_s, hids);
STRUCT_COMPOSE(app_s, config);
STRUCT_COMPOSE(app_s, peripheral);

static void app_timeout_reschedule(struct app_s *app);

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
  struct app_s *app = app_s_from_peripheral(ble_peripheral_s_from_conn(conn));

  app->pairing_end = 0;
  led_blink(&app->led[LED_GREEN], 100, 50, 5);

  app_timeout_reschedule(app);
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
  struct app_s *app = app_s_from_peripheral(ble_peripheral_s_from_conn(conn));

  led_blink(&app->led[LED_RED], 500, 500, 3);
}

static
void peri_state_changed(struct ble_peripheral_s *peri, enum ble_peripheral_state_e state)
{
  struct app_s *app = app_s_from_peripheral(peri);

  printk("%s %d\n", __FUNCTION__, state);
  
  switch (state) {
  case BLE_PERIPHERAL_CONNECTED:
    led_blink(&app->led[LED_BLUE], 100, 100, 5);
    break;

  case BLE_PERIPHERAL_RECONNECTING:
  case BLE_PERIPHERAL_ADVERTISING:
    led_blink(&app->led[LED_BLUE], 300, 1700, -1);
    break;

  case BLE_PERIPHERAL_PAIRING:
    mcr12_power(&app->barcode_reader, 0);
    led_blink(&app->led[LED_BLUE], 500, 500, -1);
    break;

  default:
    mcr12_power(&app->barcode_reader, 0);
    led_blink(&app->led[LED_BLUE], 200, 0, 1);
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

static void app_timeout_reschedule(struct app_s *app)
{
  dev_timer_value_t now;
  dev_timer_delay_t timeout;
  const dev_timer_delay_t timeout_max = app->second * 3600;
  uint8_t mode = 0;
  error_t err;

  DEVICE_OP(&app->timer, cancel, &app->timeout_rq);

 again:
  
  DEVICE_OP(&app->timer, get_value, &now, 0);
  timeout = timeout_max;

  dprintk("%s pairing %lld %lld\n", __FUNCTION__, now, app->pairing_end);
  dprintk("%s conn    %lld %lld\n", __FUNCTION__, now, app->conn_end);
  dprintk("%s adv     %lld %lld\n", __FUNCTION__, now, app->adv_end);

  if (now < app->pairing_end) {
    mode |= BLE_PERIPHERAL_PAIRABLE
      | BLE_PERIPHERAL_CONNECTABLE
      | BLE_PERIPHERAL_DISCOVERABLE;
    timeout = __MIN(timeout, app->pairing_end - now + 1);
  }

  if (now < app->conn_end) {
    mode |= BLE_PERIPHERAL_CONNECTABLE;
    timeout = __MIN(timeout, app->conn_end - now + 1);
  }

  if (now < app->adv_end) {
    mode |= BLE_PERIPHERAL_DISCOVERABLE;
    timeout = __MIN(timeout, app->adv_end - now + 1);
  }

  if (now < app->long_press_end) {
    timeout = __MIN(timeout, app->long_press_end - now + 1);
  }
  
  if (timeout == timeout_max)
    timeout = app->second * 5;

  dprintk("%s %lld mode %d timeout %d\n", __FUNCTION__,
         now, mode, timeout);

  if (app->peripheral.last_state == BLE_PERIPHERAL_IDLE
      && !mode) {
    if (app->off_already) {
      poweroff();
      return;
    } else {
      app->off_already = 1;
    }
  } else {
    app->off_already = 0;
  }
  
  ble_peripheral_mode_set(&app->peripheral, mode);

  app->timeout_rq.delay = timeout;
  err = DEVICE_OP(&app->timer, request, &app->timeout_rq);
  if (err)
    goto again;
}

static KROUTINE_EXEC(app_timeout_handle)
{
  struct app_s *app = KROUTINE_CONTAINER(kr, *app, timeout_rq.rq.kr);
  dev_timer_value_t now;

  DEVICE_OP(&app->timer, get_value, &now, 0);

  if (app->long_press_end && now > app->long_press_end)
    app->pairing_end = app->long_press_end + app->adv_time;

  app_timeout_reschedule(app);
}

static void code_received(struct mcr12_s *mcr, const uint8_t *code, size_t size)
{
  struct app_s *app = app_s_from_barcode_reader(mcr);
  dev_timer_value_t now;

  led_blink(&app->led[LED_GREEN], 50, 50, 3);

  DEVICE_OP(&app->timer, get_value, &now, 0);

  app->conn_end = now + app->idle_time;
  app_timeout_reschedule(app);

  printk("Code received: '%S'\n", code, size);

  hid_send_ascii(&app->hids, app->config.config.prefix, strlen(app->config.config.prefix));
  hid_send_ascii(&app->hids, code, size);
  hid_send_ascii(&app->hids, app->config.config.suffix, strlen(app->config.config.suffix));
}

static void hid_report_subscribed(struct hid_service_s *hids, bool_t enabled)
{
  struct app_s *app = app_s_from_hids(hids);

  mcr12_power(&app->barcode_reader, enabled);
}

static void hid_transmitting(struct hid_service_s *hids)
{
  struct app_s *app = app_s_from_hids(hids);

  led_blink(&app->led[LED_BLUE], 50, 50, 1);
}

static const struct ble_peripheral_params_s peri_params = {
  .adv_interval_ms = 50,
};

static const struct mcr12_handler_s barcode_reader_handler = {
  .code_received = code_received,
};

static const struct hid_service_handler_s hid_handler = {
  .enabled = hid_report_subscribed,
  .transmitting = hid_transmitting,
};

static KROUTINE_EXEC(button_changed)
{
  struct app_s *app = KROUTINE_CONTAINER(kr, *app, button_rq.base.kr);
  dev_timer_value_t now;

  DEVICE_OP(&app->timer, get_value, &now, 0);

  if (app->button_state != app->button_last_state) {
    app->button_last_state = app->button_state;

    dprintk("Button: %d\n", app->button_state);

    if (app->button_state) {
      if (app->hids.enabled) {
        led_blink(&app->led[LED_GREEN], 50, 50, 2);

        mcr12_trigger(&app->barcode_reader);
      } else {
        led_blink(&app->led[LED_RED], 50, 50, 2);

        app->adv_end = now + app->adv_time;
        if (app->peripheral.last_state != BLE_PERIPHERAL_CONNECTED)
          app->long_press_end = now + app->long_press_time;
      }
    } else {
      app->long_press_end = 0;
    }
  }

  DEVICE_OP(&app->button, request, &app->button_rq);

  app->conn_end = now + app->idle_time;
  app_timeout_reschedule(app);
}

static void config_changed(struct config_service_s *config)
{
  struct app_s *app = app_s_from_config(config);

  hid_service_keymap_set(&app->hids, keymap_get(app->config.config.keymap));
}

static const struct config_service_handler_s config_handler = {
  .changed = config_changed,
};

static CONTEXT_ENTRY(main)
{
  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;

  printk("Barcode reader\n");

  memset(app, 0, sizeof(*app));

  err = ble_stack_context_init(&app->context, "/ble", "/rtc1", "/rng", "/aes", "/nvmc");
  ensure(!err && "stack context failed");

  err = ble_peripheral_init(&app->peripheral, &peri_params, &peri_handler, &app->context);
  ensure(!err && "peripheral init failed");

  ensure(dis_service_register(&app->dis, &app->context.gattdb) == 0);
  ensure(gap_service_register(&app->gap, &app->context.gattdb) == 0);
  ensure(scps_service_register(&app->scps, &app->context.gattdb) == 0);
  ensure(config_service_register(&app->config, &app->context.gattdb, &config_handler) == 0);
  ensure(hid_service_register(&app->hids, &hid_handler, &app->context.gattdb,
                              keymap_get(app->config.config.keymap)) == 0);
  ensure(bas_register(&app->bas, &app->context.gattdb) == 0);

  for (uint_fast8_t i = 0; i < LED_COUNT; ++i)
    led_init(&app->led[i], led_pin[i], 0);
  mcr12_init(&app->barcode_reader, &barcode_reader_handler, 38400);

  ensure(device_get_accessor_by_path(&app->button.base, NULL,
                                     "keyboard", DRIVER_CLASS_VALIO) == 0);

  ensure(device_get_accessor_by_path(&app->timer.base, NULL,
                                     "/rtc*", DRIVER_CLASS_TIMER) == 0);
  
  app->button_rq.attribute = VALIO_KEYBOARD_MAP;
  app->button_rq.type = DEVICE_VALIO_WAIT_EVENT;
  app->button_rq.data = &app->button_state;
  kroutine_init_sched_switch(&app->button_rq.base.kr, button_changed);
  DEVICE_OP(&app->button, request, &app->button_rq);

  dev_timer_value_t now;

  DEVICE_OP(&app->timer, get_value, &now, 0);

  kroutine_init_deferred(&app->timeout_rq.rq.kr, app_timeout_handle);
  dev_timer_init_sec(&app->timer, &app->second, NULL, 1, 1);
  app->adv_time = app->second * 30;
  app->idle_time = app->second * 120;
  app->long_press_time = app->second * 5;
  app->conn_end = 0;
  app->pairing_end = 0;
  app_timeout_reschedule(app);
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 0x1000,
  };

  thread_create(main, 0, &attr);
}
