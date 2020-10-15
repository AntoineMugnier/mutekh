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

#ifndef BLE_SCANNER_H_
#define BLE_SCANNER_H_

#include <hexo/types.h>
#include <net/task.h>
#include <net/layer.h>

#include <ble/protocol/radio.h>
#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

enum ble_scanner_policy_e
{
  BLE_SCANNER_IGNORE = 0,
  BLE_SCANNER_SCAN = 1,
  BLE_SCANNER_CONNECT = 2,
};

struct ble_scanner_target_s
{
  struct ble_addr_s addr;
  uint8_t policy;
};

#define BLE_SCANNER_TARGET_MAXCOUNT 8

struct ble_scanner_param_s
{
  uint32_t interval_ms;
  uint32_t duration_ms;
  struct ble_addr_s local_addr;
  size_t target_count;
  struct ble_scanner_target_s target[BLE_SCANNER_TARGET_MAXCOUNT];
  uint8_t default_policy;

  uint32_t access_address;
  uint32_t crc_init;
  enum ble_phy_mode_e phy;

  struct ble_gap_preferred_conn_params_s timing;
};

struct ble_scanner_handler_s
{
  struct net_layer_handler_s base;

  error_t (*params_update)(struct net_layer_s *layer,
                           const struct ble_scanner_param_s *params);
};

STRUCT_COMPOSE(ble_scanner_handler_s, base);

struct ble_scanner_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  bool_t (*connection_requested)(void *delegate, struct net_layer_s *layer,
                                 const struct ble_adv_connect_s *conn,
                                 dev_timer_value_t anchor);
};

STRUCT_COMPOSE(ble_scanner_delegate_vtable_s, base);

ALWAYS_INLINE
error_t ble_scanner_params_update(struct net_layer_s *layer,
                                  const struct ble_scanner_param_s *params)
{
  const struct ble_scanner_handler_s *handler
    = const_ble_scanner_handler_s_from_base(layer->handler);

  return handler->params_update(layer, params);
}

#endif
