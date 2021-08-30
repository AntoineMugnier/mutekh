/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#define LOGK_MODULE_ID "bcen"

#include <ble/protocol/l2cap.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>

#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>

#include <ble/stack/central.h>
#include <ble/stack/context.h>

#include <ble/net/scanner.h>
#include <ble/net/phy.h>
#include <ble/net/gatt.h>
#include <ble/net/sm.h>
#include <ble/net/gap.h>
#include <ble/net/att.h>
#include <ble/net/link.h>
#include <ble/net/llcp.h>

#include <ble/net/layer_id.h>

#include <mutek/printk.h>

#include <ble/net/generic.h>

static const struct ble_scanner_delegate_vtable_s ctr_scan_vtable;
static const struct ble_scan_filter_delegate_vtable_s central_scan_filter_vtable;

static void ble_central_target_add(struct ble_central_s *ctrl,
                                   const struct ble_addr_s *addr,
                                   enum ble_scanner_policy_e policy);

struct dev_rng_s;

static error_t scan_start(struct ble_central_s *ctr);

static void ctr_state_update(struct ble_central_s *ctr)
{
  kroutine_exec(&ctr->updater);
}

static KROUTINE_EXEC(ble_central_state_update)
{
  struct ble_central_s *ctr = KROUTINE_CONTAINER(kr, *ctr, updater);
  enum ble_central_state_e state = BLE_CENTRAL_IDLE;

  if (ctr->conn.phy) {
    state = BLE_CENTRAL_CONNECTED;
  } else if (ctr->scan) {
    if (ctr->mode & BLE_CENTRAL_PAIRABLE)
      state = BLE_CENTRAL_PAIRING;
    else
      state = BLE_CENTRAL_SCANNING;
  }

  if (state == BLE_CENTRAL_IDLE && ctr->mode & BLE_CENTRAL_CONNECTABLE)
    scan_start(ctr);

  if (state == ctr->last_state)
    return;

  if (ctr->mode == 0 && state == BLE_CENTRAL_IDLE)
    ble_stack_context_release(ctr->context);

  ctr->last_state = state;

  logk_trace("Central state now %d", state);

  ctr->handler->state_changed(ctr, state);
}

static void ctr_conn_state_changed(struct ble_stack_connection_s *conn,
                                    bool_t connected)
{
  struct ble_central_s *ctr = ble_central_s_from_conn(conn);

  if (!connected && ctr->mode & BLE_CENTRAL_CONNECTABLE)
    scan_start(ctr);

#ifdef CONFIG_BLE_CRYPTO
  if (connected
      && !(ctr->mode & BLE_CENTRAL_PAIRABLE)
      && conn->peer.paired
      && conn->peer.ltk_present)
    ble_llcp_encryption_enable(conn->llcp);
#endif

  ctr_state_update(ctr);
}

static
void ctr_conn_pairing_success(struct ble_stack_connection_s *conn)
{
  struct ble_central_s *ctr = ble_central_s_from_conn(conn);

  logk_trace("Central pairing success");

  if (!ctr->conn.llcp)
    return;

  error_t err = ble_llcp_encryption_enable(ctr->conn.llcp);

  if (err) {
    logk_error("Encryption enable error: %d", err);
    ble_stack_connection_drop(&ctr->conn);
  }
}

static const struct ble_stack_context_handler_s ctr_conn_handler =
{
  .state_changed = ctr_conn_state_changed,
  .pairing_success = ctr_conn_pairing_success,
};

static
bool_t ctr_connection_requested(void *delegate, struct net_layer_s *layer,
                                 const struct ble_adv_connect_s *conn,
                                 dev_timer_value_t anchor)
{
  struct ble_central_s *ctr = delegate;
  error_t err;
  struct ble_phy_params_s phy_params;

  phy_params.connect_packet_timestamp = anchor;
  phy_params.phy = ctr->params.phy;
  phy_params.conn_req = *conn;

  logk_trace("Connection request from "BLE_ADDR_FMT"...", BLE_ADDR_ARG(&conn->master));

#if defined(CONFIG_BLE_CRYPTO)
  if (ctr->conn.sm) {
    logk_error("Cannot accept connection, still have sm");
    return 1;
  }
#endif

  if (ctr->conn.phy) {
    logk_error("Cannot accept connection, still have master");
    return 1;
  }

  ctr->handler->connection_opened(ctr, &conn->slave);

  err = ble_stack_connection_create(&ctr->conn, ctr->context,
                                    &ctr->handler->base,
                                    &ctr_conn_handler,
                                    1, &phy_params,
                                    &ctr->conn_params);

  if (!err) {
    net_layer_refdec(ctr->scan);
    ctr->scan = NULL;
  }

  ctr_state_update(ctr);

  return err ? 1 : 0;
}

static error_t scan_start(struct ble_central_s *ctr)
{
  error_t err;

  if (ctr->conn.phy) {
    logk_error("Cannot scan: has master");
    return 0;
  }

  if (ctr->scan) {
    logk_error("Scan already active %p", ctr->scan);
    return 0;
  }

  ctr->params.access_address = ble_stack_access_address_generate(ctr->context);
  dev_rng_wait_read(&ctr->context->rng, &ctr->params.crc_init, 4);
  // 6.B.2.3.3.1: It shall have a random value in the range 5 to 16.
  ctr->params.hop = ((ctr->params.crc_init >> 24) & 0xf) + 5;
  ctr->params.crc_init &= 0xffffff;
  ctr->params.channel_map = (1ull << 37) - 1;

  logk("Using access address %08x, crcinit %06x, hop %d",
       ctr->params.access_address, ctr->params.crc_init, ctr->params.hop);

  ble_stack_context_local_address_get(ctr->context, &ctr->params.local_addr);

  ctr->params.default_policy = BLE_SCANNER_IGNORE;
  ctr->params.target_count = 0;

  ctr->params.timing = ctr->conn_params;

  logk_trace("Central scanning starting");

  err = DEVICE_OP(&ctr->context->ble, layer_create,
                  &ctr->context->scheduler,
                  BLE_NET_LAYER_SCANNER,
                  &ctr->params,
                  ctr, &ctr_scan_vtable.base,
                  &ctr->scan);
  if (err) {
    logk_error("Scanning start failed: %d", err);
    ctr->scan = NULL;
    goto out;
  }

  struct ble_scan_filter_param_s filter_params = {
#ifdef CONFIG_BLE_CRYPTO
    .peerdb = &ctr->context->security_db,
#else
    .peerdb = NULL,
#endif
    .scan_params = ctr->params,
  };

  err = ble_scan_filter_create(&ctr->context->scheduler,
                               &filter_params, ctr, &central_scan_filter_vtable.base,
                               &ctr->scan_filter);
  if (err) {
    logk_error("Scanning filter creation failed: %d", err);
    goto scan_forget;
  }

  err = net_layer_bind(ctr->scan, NULL, ctr->scan_filter);

  net_layer_refdec(ctr->scan_filter);

  if (!err)
    goto out;

  logk_error("Scan filter binding failed: %d", err);

 scan_forget:
  net_layer_refdec(ctr->scan);
  ctr->scan = NULL;

 out:
  ctr_state_update(ctr);

  return err;
}

error_t ble_central_init(
  struct ble_central_s *ctr,
  const struct ble_central_params_s *params,
  const struct ble_central_handler_s *handler,
  struct ble_stack_context_s *context)
{
  memset(ctr, 0, sizeof(*ctr));

  ctr->params.phy = params->phy;
  ctr->params.interval_ms = params->scan_interval_ms;
  ctr->params.duration_ms = params->scan_duration_ms;
  ctr->params.target_count = 0;
  ctr->params.default_policy = BLE_SCANNER_IGNORE;

  ctr->conn_params = params->conn;

  ctr->handler = handler;
  ctr->context = context;
  ctr->mode = 0;

  ble_stack_context_local_address_get(context, &ctr->addr);
  kroutine_init_sched_switch(&ctr->updater, ble_central_state_update);
  
  return 0;
}

static void ctr_scan_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_central_s *ctr = delegate;

  if (layer == ctr->scan)
    ctr->scan = NULL;
  
  ctr_state_update(ctr);
}

static void ctr_scan_filter_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_central_s *ctr = delegate;

  logk_debug("Scan filter layer %p ctr->scan_filter %p destroyed", layer, ctr->scan_filter);
  
  if (layer == ctr->scan_filter)
    ctr->scan_filter = NULL;
  
  ctr_state_update(ctr);
}

static const struct ble_scanner_delegate_vtable_s ctr_scan_vtable =
{
  .base.release = ctr_scan_destroyed,
  .connection_requested = ctr_connection_requested,
};

void ble_central_mode_set(struct ble_central_s *ctr, uint8_t mode)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  if (ble_security_db_count(&ctr->context->security_db) == 0
      && mode & BLE_CENTRAL_CONNECTABLE) {
    logk_trace("No peer in sec db, pairing mode forced");
    mode |= BLE_CENTRAL_PAIRABLE;
  }
#else
  mode |= BLE_CENTRAL_PAIRABLE;
#endif

  if (mode == ctr->mode)
    return;

  if (ctr->mode == 0)
    ble_stack_context_use(ctr->context);

  ctr->mode = mode;

  if (!(mode & BLE_CENTRAL_CONNECTABLE))
    ble_stack_connection_drop(&ctr->conn);

  if (ctr->scan) {
    net_layer_refdec(ctr->scan);
    ctr->scan = NULL;
  }

  if (mode & BLE_CENTRAL_CONNECTABLE && !ctr->conn.phy)
    scan_start(ctr);
}

static
enum ble_scan_filter_policy_e central_device_updated(void *delegate, struct net_layer_s *layer,
                                                     const struct ble_scan_filter_device_s *device)
{
  struct ble_central_s *central = delegate;

  return central->handler->device_policy(central, device);
}

static const struct ble_scan_filter_delegate_vtable_s central_scan_filter_vtable =
{
  .base.release = ctr_scan_filter_destroyed,
  .device_updated = central_device_updated,
};

static void ble_central_target_add(struct ble_central_s *ctrl,
                                   const struct ble_addr_s *addr,
                                   enum ble_scanner_policy_e policy)
{
  if (ctrl->params.target_count < BLE_SCANNER_TARGET_MAXCOUNT) {
    ctrl->params.target_count++;
  } else {
    memmove(ctrl->params.target, ctrl->params.target + 1,
            sizeof(struct ble_scanner_target_s) * (BLE_SCANNER_TARGET_MAXCOUNT - 1));
  }

  ctrl->params.target[ctrl->params.target_count - 1].addr = *addr;
  ctrl->params.target[ctrl->params.target_count - 1].policy = policy;

  if (ctrl->scan)
    ble_scanner_params_update(ctrl->scan, &ctrl->params);
}

void ble_central_connect(struct ble_central_s *ctrl, const struct ble_addr_s *addr)
{
  ble_central_target_add(ctrl, addr, BLE_SCANNER_CONNECT);
}
