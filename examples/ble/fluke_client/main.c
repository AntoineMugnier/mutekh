#define LOGK_MODULE_ID "main"

#include <assert.h>

#include <stdio.h>
#include <enums.h>

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <hexo/power.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/timer.h>

#include <ble/stack/context.h>
#include <ble/profile/fluke/central.h>
#include <ble/profile/fluke/measurement.h>

struct app_s {
  struct ble_stack_context_s context;
  struct fluke_central_s fluke;
  struct device_timer_s timer;
  dev_timer_delay_t sec;
};

STRUCT_COMPOSE(app_s, context);
STRUCT_COMPOSE(app_s, fluke);

static
void fluke_connected(struct fluke_central_s *client,
                     const struct ble_addr_s *addr)
{
  logk("Fluke connected to "BLE_ADDR_FMT"", BLE_ADDR_ARG(addr));
}

static
void fluke_disconnected(struct fluke_central_s *client)
{
  logk("Fluke disconnected");

  power_reboot();
}

static
void fluke_discovery_done(struct fluke_central_s *client,
                       const char *name,
                       const char *serial,
                       const char *fw_version)
{
  logk("Fluke discovery done");
}

static
void fluke_measurement(struct fluke_central_s *client,
                    const struct fluke_measurement_s *measurement)
{
  struct app_s *app = app_s_from_fluke(client);

  char meas_str[32];
  dev_timer_value_t ts;
  uint64_t ms;

  fluke_measurement_value_to_string(measurement, meas_str);

  DEVICE_OP(&app->timer, get_value, &ts, 0);

  ms = ts * 1000 / app->sec;

  printk("@%lld: %s\n", ms, meas_str);
}

static const struct fluke_central_vtable_s fluke_vtable =
{
  .connected = fluke_connected,
  .disconnected = fluke_disconnected,
  .discovery_done = fluke_discovery_done,
  .measurement = fluke_measurement,
};

static CONTEXT_ENTRY(main)
{
  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;

  ensure(app);
  memset(app, 0, sizeof(*app));

  err = ble_stack_context_init(&app->context,
                               "/ble", "/rtc1", "/rng", "/aes",
                               NULL);
  ensure(!err);

  err = device_get_accessor_by_path(&app->timer.base, NULL, "rtc* timer*", DRIVER_CLASS_TIMER);
  logk("timer: %d");
  ensure(!err);

  device_start(&app->timer.base);
  dev_timer_init_sec(&app->timer, &app->sec, NULL, 1, 1);
  logk("sec: %d", app->sec);
  
  err = fluke_central_init(&app->fluke, &app->context, &fluke_vtable);
  ensure(!err);

  logk("Fluke client started");
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 1024,
  };
  
  thread_create(main, 0, &attr);
}
