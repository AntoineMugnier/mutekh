#define LOGK_MODULE_ID "fctr"

#include <assert.h>

#include <stdio.h>
#include <enums.h>

#include <mutek/printk.h>

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

#include <ble/profile/fluke/measurement.h>
#include <ble/profile/fluke/client.h>
#include <ble/profile/fluke/central.h>

static void client_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct fluke_central_s *fc = delegate;

  logk_trace("Client layer released");
  if (layer != fc->client)
    return;

  fc->client = NULL;
}

static void fluke_central_discovery_done(void *delegate,
                                struct net_layer_s *layer,
                                const char *name,
                                const char *serial,
                                const char *fw_version)
{
  struct fluke_central_s *fc = delegate;

  fc->vtable->discovery_done(fc, name, serial, fw_version);
}

static void fluke_central_data_changed(void *delegate,
                             struct net_layer_s *layer,
                             const struct fluke_measurement_s *measurement)
{
  struct fluke_central_s *fc = delegate;

  fc->vtable->measurement(fc, measurement);
}

static const struct fluke_gatt_client_delegate_vtable_s client_fluke_central_vtable =
{
  .base.release = client_destroyed,
  .discovery_done = fluke_central_discovery_done,
  .data_changed = fluke_central_data_changed,
};

static
void fluke_central_client_create(struct fluke_central_s *fc)
{
  error_t err;

  err = fluke_gatt_client_create(&fc->context->scheduler, NULL,
                            fc, &client_fluke_central_vtable, &fc->client);
  if (err) {
    logk_error("Unable to create device client layer: %d", err);
    return;
  }

  uint16_t proto = BLE_ATT_CLIENT;
  err = net_layer_bind(fc->central.conn.att, &proto, fc->client);
  if (err) {
    logk_error("error while binding device client to att: %d", err);
    net_layer_refdec(fc->client);
    fc->client = NULL;
    return;
  }

  net_layer_refdec(fc->client);
}

static
void fluke_central_connection_closed(struct ble_stack_connection_s *conn,
                                     uint8_t reason)
{
  struct ble_central_s *ctr = ble_central_s_from_conn(conn);
  struct fluke_central_s *fc = fluke_central_s_from_central(ctr);

  fc->vtable->disconnected(fc);
}

static
void fluke_central_connection_opened(struct ble_central_s *ctrl,
                                     const struct ble_addr_s *addr)
{
  struct fluke_central_s *fc = fluke_central_s_from_central(ctrl);

  logk_trace("Central connection opened to "BLE_ADDR_FMT"",
         BLE_ADDR_ARG(addr));

  fc->vtable->connected(fc, addr);
}

static
void fluke_central_state_changed(struct ble_central_s *ctrl,
                        enum ble_central_state_e state)
{
  struct fluke_central_s *fc = fluke_central_s_from_central(ctrl);

  /* logk_trace("Central state now %d", state); */

  switch (state) {
  case BLE_CENTRAL_IDLE:
    logk("Central idle");
    break;
    
  case BLE_CENTRAL_SCANNING:
    logk("Central scanning");
    break;
    
  case BLE_CENTRAL_PAIRING:
    logk("Central pairing");
    break;

  case BLE_CENTRAL_CONNECTED:
    logk("Central connected");
    fluke_central_client_create(fc);
    break;
  }
}

static
enum ble_scan_filter_policy_e fluke_central_device_policy(struct ble_central_s *ctrl,
                                                 const struct ble_scan_filter_device_s *device)
{
  static const struct ble_uuid_s gatt_fluke_service_type
    = BLE_UUID(0xb6981800, 0x7562, 0x11e2, 0xb50d, 0x00163e46f8feULL);
  struct fluke_central_s *fc = fluke_central_s_from_central(ctrl);
  bool_t found = 0;

  (void)fc;

  /* logk_trace(" % 3d "BLE_ADDR_FMT" for %ds,%s%s%s%s", */
  /*        device->id, */
  /*        BLE_ADDR_ARG(&device->addr), */
  /*        (uint32_t)(device->last_seen - device->first_seen) / 32768, */
  /*        device->active ? " active" : "", */
  /*        device->connectable ? " connectable" : "", */
  /*        device->scannable ? " scannable" : "", */
  /*        device->scanned ? " scanned" : ""); */

  for (const uint8_t *ad = device->ad;
       ad < device->ad + device->ad_len;
       ad += *ad + 1) {
    const uint8_t *data = ad + 2;
    const uint8_t data_size = ad[0] - 1;
    (void)data_size;

    /* logk_trace("  %02x '%P'", ad[1], data, data_size); */

    switch (ad[1]) {
    case BLE_GAP_UUID128_SERVICE_LIST_INCOMPLETE:
    case BLE_GAP_UUID128_SERVICE_LIST_COMPLETE:
      if (!memcmp(data, &gatt_fluke_service_type, 16)) {
        /* logk_trace("  is Fluke"); */
        found = 1;
      }
      
      break;
    }
  }

  if (found)
    return BLE_SCAN_FILTER_CONNECT;

  return device->scanned ? BLE_SCAN_FILTER_MONITOR : BLE_SCAN_FILTER_SCAN;
}

static const struct ble_central_handler_s fluke_central_central_handler =
{
  .base.connection_closed = fluke_central_connection_closed,
  .state_changed = fluke_central_state_changed,
  .connection_opened = fluke_central_connection_opened,
  .device_policy = fluke_central_device_policy,
};

error_t fluke_central_init(struct fluke_central_s *fc,
                           struct ble_stack_context_s *context,
                           const struct fluke_central_vtable_s *vtable)
{
  error_t err;
  
  memset(fc, 0, sizeof(*fc));

  fc->central_params.scan_interval_ms = 800;
  fc->central_params.scan_duration_ms = 500;
  fc->central_params.conn.interval_min = 24;
  fc->central_params.conn.interval_max = 24;
  fc->central_params.conn.latency = 0;
  fc->central_params.conn.timeout = 200;

  fc->vtable = vtable;
  fc->context = context;

  err = ble_central_init(&fc->central, &fc->central_params,
                         &fluke_central_central_handler, fc->context);
  if (err)
    return err;

  ble_central_mode_set(&fc->central,
                       BLE_CENTRAL_CONNECTABLE | BLE_CENTRAL_PAIRABLE);

  return 0;
}

error_t fluke_central_disconnect(struct fluke_central_s *client)
{
  return -EINVAL;
}
