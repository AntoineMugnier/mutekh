#include <assert.h>

#include <stdio.h>
#include <enums.h>

#include <mutek/console.h>
#include <mutek/shell.h>
#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>

#include <ble/stack/context.h>
#include <ble/protocol/error.h>
#include <ble/protocol/gap.h>
#include <ble/protocol/company.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/net.h>

#include <ble/net/scanner.h>
#include <ble/net/scan_filter.h>
#include <ble/net/layer_id.h>
#include <ble/net/generic.h>

struct scanner_s {
  struct ble_stack_context_s context;
  struct net_layer_s *scan;
  struct net_layer_s *scan_filter;
};

static
bool_t scanner_connection_requested(void *delegate, struct net_layer_s *layer,
                                    const struct ble_adv_connect_s *conn,
                                    dev_timer_value_t anchor)
{
  return 1;
}

static void scanner_scan_destroyed(void *delegate, struct net_layer_s *layer)
{
}

static
enum ble_scan_filter_policy_e scanner_device_updated(void *delegate, struct net_layer_s *layer,
                                                     const struct ble_scan_filter_device_s *device)
{
  struct scanner_s *scanner = delegate;

  printk(" % 3d "BLE_ADDR_FMT" for %ds,%s%s%s%s\n",
         device->id,
         BLE_ADDR_ARG(&device->addr),
         (uint32_t)(device->last_seen - device->first_seen) / 32768,
         device->active ? " active" : "",
         device->connectable ? " connectable" : "",
         device->scannable ? " scannable" : "",
         device->scanned ? " scanned" : "");

  for (const uint8_t *ad = device->ad; ad < device->ad + device->ad_len; ad += *ad + 1) {
    switch (ad[1]) {
    case BLE_GAP_FLAGS:
      printk(" flags 0x%x\n", ad[2]);
      break;

    case BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE:
      printk(" srv");
      for (uint8_t i = 0; i < ad[0] - 1; i += 2) {
        struct ble_uuid_s uuid;
        ble_uuid_bluetooth_based(&uuid, endian_le16_na_load(ad + i));
        printk(" "BLE_UUID_FMT, BLE_UUID_ARG(&uuid));
      }
      printk(" ...\n");
      break;

    case BLE_GAP_UUID16_SERVICE_LIST_COMPLETE:
      printk(" srv");
      for (uint8_t i = 2; i < ad[0] + 1; i += 2) {
        struct ble_uuid_s uuid;
        ble_uuid_bluetooth_based(&uuid, endian_le16_na_load(ad + i));
        printk(" "BLE_UUID_FMT, BLE_UUID_ARG(&uuid));
      }
      printk("\n");
      break;

    case BLE_GAP_UUID128_SERVICE_LIST_INCOMPLETE:
      printk(" srv");
      for (uint8_t i = 2; i < ad[0] + 1; i += 16) {
        struct ble_uuid_s uuid;
        memrevcpy(&uuid, ad + i, 16);
        printk(" "BLE_UUID_FMT, BLE_UUID_ARG(&uuid));
      }
      printk(" ...\n");
      break;

    case BLE_GAP_UUID128_SERVICE_LIST_COMPLETE:
      printk(" srv");
      for (uint8_t i = 2; i < ad[0] + 1; i += 16) {
        struct ble_uuid_s uuid;
        memrevcpy(&uuid, ad + i, 16);
        printk(" "BLE_UUID_FMT, BLE_UUID_ARG(&uuid));
      }
      printk("\n");
      break;

    case BLE_GAP_SHORTENED_LOCAL_NAME:
      printk(" \"%S...\"\n", ad + 2, ad[0] - 1);
      break;

    case BLE_GAP_SLAVE_CONNECTION_INTERVAL_RANGE:
      printk(" connInt %d-%d\n",
             endian_le16_na_load(ad + 2),
             endian_le16_na_load(ad + 4));
      break;

    case BLE_GAP_COMPLETE_LOCAL_NAME:
      printk(" \"%S\"\n", ad + 2, ad[0] - 1);
      break;

    case BLE_GAP_APPEARANCE:
      printk(" appear 0x%x\n", endian_le16_na_load(ad + 2));
      break;

    case BLE_GAP_MANUFACTURER_SPECIFIC_DATA:
      printk(" %s-specific (%d) [%P]\n",
             ble_company_name(endian_le16_na_load(ad + 2)),
             endian_le16_na_load(ad + 2),
             ad + 4, ad[0] - 3);
      break;

    default:
      printk(" %d: [%P]\n", ad[1], ad + 2, ad[0] - 1);
      break;
    }
  }

  if (device->connectable)
    return BLE_SCAN_FILTER_SCAN;
  return BLE_SCAN_FILTER_IGNORE;
}

static const struct ble_scan_filter_delegate_vtable_s scanner_scan_filter_vtable =
{
  .base.release = scanner_scan_destroyed,
  .device_updated = scanner_device_updated,
};

static const struct ble_scanner_delegate_vtable_s scanner_scan_vtable =
{
  .base.release = scanner_scan_destroyed,
  .connection_requested = scanner_connection_requested,
};

static CONTEXT_ENTRY(main)
{
  struct scanner_s *scanner = mem_alloc(sizeof(*scanner), mem_scope_sys);
  error_t err;
  struct ble_scanner_param_s scan_params = {
    .interval_ms = 800,
    .duration_ms = 500,
  };

  ensure(scanner);

  memset(scanner, 0, sizeof(*scanner));

  err = ble_stack_context_init(&scanner->context, "/ble", "/rtc1", "/rng", "/aes", "/nvmc");
  assert(!err && "ble_stack_context_init failed");

  ble_stack_context_local_address_get(&scanner->context, &scan_params.local_addr);

  err = DEVICE_OP(&scanner->context.ble, layer_create,
                  &scanner->context.scheduler,
                  BLE_NET_LAYER_SCANNER,
                  &scan_params,
                  scanner, &scanner_scan_vtable.base,
                  &scanner->scan);
  if (err) {
    printk("Scanning start failed: %d\n", err);
    return;
  }

  err = ble_scan_filter_create(&scanner->context.scheduler, &scan_params, scanner, &scanner_scan_filter_vtable.base, &scanner->scan_filter);
  if (err) {
    printk("Scanning start failed: %d\n", err);
    return;
  }

  err = net_layer_bind(scanner->scan, NULL, scanner->scan_filter);

  printk("Scanning... %d\n", err);
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 1024,
  };

  thread_create(main, 0, &attr);
}
