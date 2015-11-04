#include <stdlib.h>

#include <mutek/thread.h>
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>

#include <device/device.h>
#include <device/driver.h>

#include <hexo/power.h>

#include <ble/stack/peripheral.h>

#include "dis.h"
#include "gap.h"
#include "led.h"

#define dprintk(...) do{}while(0)

struct app_s
{
    struct ble_stack_context_s context;
    struct ble_peripheral_s peripheral;
    struct ble_gattdb_registry_s gap;
    struct ble_gattdb_registry_s dis;
    struct led_s led;
};

static
void peri_pairing_requested(struct ble_peripheral_s *peri, bool_t bonding)
{
  ble_peripheral_pairing_accept(peri, 0, 0, 0);
}

static
void peri_pairing_failed(struct ble_peripheral_s *peri, enum sm_reason reason)
{
}

static
void peri_pairing_success(struct ble_peripheral_s *peri)
{
}

static
bool_t peri_connection_requested(struct ble_peripheral_s *peri, const struct ble_addr_s *addr)
{
  return 1;
}

static
void peri_connection_closed(struct ble_peripheral_s *peri, uint8_t reason)
{
}

static
void peri_state_changed(struct ble_peripheral_s *peri, enum ble_peripheral_state_e state)
{
}

static const struct ble_peripheral_handler_s peri_handler =
{
  .pairing_requested = peri_pairing_requested,
  .pairing_failed = peri_pairing_failed,
  .pairing_success = peri_pairing_success,
  .connection_requested = peri_connection_requested,
  .connection_closed = peri_connection_closed,
  .state_changed = peri_state_changed,
};

static const struct ble_peripheral_params_s peri_params = {
  .adv_interval_ms = 50,
};

static CONTEXT_ENTRY(main)
{
    struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
    error_t err;

    printk("RGB Led\n");

    memset(app, 0, sizeof(*app));

    err = ble_stack_context_init(&app->context, "/ble", "/rtc1", "/rng", "/aes", "/nvmc");
    ensure(!err && "stack context failed");

    err = ble_peripheral_init(&app->peripheral, &peri_params, &peri_handler, &app->context);
    ensure(!err && "peripheral init failed");

    dis_service_register(&app->dis, &app->context.gattdb);
    gap_service_register(&app->gap, &app->context.gattdb);
    led_service_register(&app->led, &app->context.gattdb);

    ble_peripheral_mode_set(&app->peripheral, 0
                            | BLE_PERIPHERAL_DISCOVERABLE
                            | BLE_PERIPHERAL_PAIRABLE
                            | BLE_PERIPHERAL_CONNECTABLE);
}

void app_start(void)
{
    struct thread_attr_s attr = {
        .stack_size = 0x1000,
    };

    thread_create(main, 0, &attr);
}
