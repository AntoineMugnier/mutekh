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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <ble/stack/raw_connection.h>
#include <ble/stack/context.h>
#include <ble/protocol/address.h>
#include <ble/protocol/error.h>
#include <ble/protocol/advertise.h>
#include <ble/net/layer_id.h>
#include <ble/net/adv.h>
#include <ble/net/scanner.h>
#include <ble/net/phy.h>

error_t ble_stack_raw_conn_init(struct ble_stack_raw_conn_s *conn,
                                struct ble_stack_context_s *context,
                                const struct ble_stack_raw_conn_handler_s *handler)
{
  conn->context = context;
  conn->handler = handler;
  conn->adv = NULL;
  conn->phy = NULL;

  return 0;
}

static void conn_layer_destroyed(void *delegate, struct net_layer_s *layer)
{
}

static void conn_close(struct ble_stack_raw_conn_s *conn, uint8_t reason)
{
  if (!conn->phy)
    return;

  net_layer_refdec(conn->phy);
  conn->phy = NULL;

  conn->handler->closed(conn, reason);
}

static void conn_adv_stop(struct ble_stack_raw_conn_s *conn)
{
  if (!conn->adv)
    return;

  net_layer_refdec(conn->adv);
  conn->adv = NULL;
}

static void phy_conn_lost(void *delegate, struct net_layer_s *layer,
                          uint8_t reason)
{
  conn_close(delegate, reason);
}

static const struct ble_phy_delegate_vtable_s conn_phy_vtable =
{
  .base.release = conn_layer_destroyed,
  .connection_lost = phy_conn_lost,
};

static
bool_t conn_requested(struct ble_stack_raw_conn_s *conn,
                      const struct ble_adv_connect_s *conn_params,
                      dev_timer_value_t anchor,
                      uint32_t layer_type)
{
  struct ble_phy_params_s phy_params = {
    .connect_packet_timestamp = anchor,
    .conn_req = *conn_params,
  };
  error_t err;

  err = DEVICE_OP(&conn->context->ble, layer_create,
                  &conn->context->scheduler,
                  layer_type,
                  &phy_params,
                  conn, &conn_phy_vtable.base,
                  &conn->phy);
  if (err) {
    conn->phy = NULL;
    return 1;
  } else {
    conn_adv_stop(conn);
    conn->handler->opened(conn);
    return 0;
  }
}

static
bool_t slave_conn_requested(void *delegate, struct net_layer_s *layer,
                            const struct ble_adv_connect_s *conn_params,
                            dev_timer_value_t anchor)
{
  return conn_requested(delegate, conn_params, anchor, BLE_NET_LAYER_SLAVE);
}

static
bool_t master_conn_requested(void *delegate, struct net_layer_s *layer,
                             const struct ble_adv_connect_s *conn_params,
                             dev_timer_value_t anchor)
{
  return conn_requested(delegate, conn_params, anchor, BLE_NET_LAYER_MASTER);
}

static const struct ble_advertiser_delegate_vtable_s conn_adv_vtable =
{
  .base.release = conn_layer_destroyed,
  .connection_requested = slave_conn_requested,
};

static const struct ble_scanner_delegate_vtable_s conn_scanner_vtable =
{
  .base.release = conn_layer_destroyed,
  .connection_requested = master_conn_requested,
};

error_t ble_stack_raw_conn_advertise(struct ble_stack_raw_conn_s *conn)
{
  error_t err;
  struct ble_advertiser_param_s adv_params = {
    .interval_ms = 50,
    .delay_max_ms = 5,
    .connectable = 1,
    .ad = NULL,
    .ad_len = 0,
  };

  ble_stack_context_local_address_get(conn->context, &adv_params.local_addr);

  conn_close(conn, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
  conn_adv_stop(conn);

  err = DEVICE_OP(&conn->context->ble, layer_create,
                  &conn->context->scheduler,
                  BLE_NET_LAYER_ADV,
                  &adv_params,
                  conn, &conn_adv_vtable.base,
                  &conn->adv);

  if (err)
    conn->adv = NULL;

  return err;
}

error_t ble_stack_raw_conn_connect(struct ble_stack_raw_conn_s *conn, const struct ble_addr_s *addr)
{
  error_t err;

  conn_close(conn, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
  conn_adv_stop(conn);

  struct ble_scanner_param_s scan_params = {
    .interval_ms = 800,
    .duration_ms = 500,
    .target_count = 1,
    .default_policy = BLE_SCANNER_IGNORE,
    .access_address = ble_stack_access_address_generate(conn->context),
    .crc_init = ble_stack_access_address_generate(conn->context) & 0xffffff,
  };

  scan_params.target[0].addr = *addr;
  scan_params.target[0].policy = BLE_SCANNER_CONNECT;

  ble_stack_context_local_address_get(conn->context, &scan_params.local_addr);

  err = DEVICE_OP(&conn->context->ble, layer_create,
                  &conn->context->scheduler,
                  BLE_NET_LAYER_SCANNER,
                  &scan_params,
                  conn, &conn_scanner_vtable.base,
                  &conn->adv);

  if (err)
    conn->adv = NULL;

  return err;
}

error_t ble_stack_raw_conn_close(struct ble_stack_raw_conn_s *conn)
{
  conn_close(conn, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
  conn_adv_stop(conn);
  return 0;
}

