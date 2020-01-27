#define LOGK_MODULE_ID "main"

#include <assert.h>

#include <stdio.h>
#include <enums.h>

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>

#include <device/driver.h>
#include <device/device.h>

#include <ble/stack/context.h>
#include <ble/profile/fluke/central.h>
#include <ble/profile/fluke/measurement.h>

struct app_s {
  struct ble_stack_context_s context;
  struct fluke_central_s fluke;
};

STRUCT_COMPOSE(fluke_central_s, context);
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
  char meas_str[32];

  fluke_measurement_value_to_string(measurement, meas_str);
  
  logk("%s\n", meas_str);
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
