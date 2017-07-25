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

#ifndef BLE_SCAN_FILTER_H_
#define BLE_SCAN_FILTER_H_

#include <hexo/types.h>
#include <net/task.h>
#include <net/layer.h>

#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/net/scanner.h>

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

struct ble_scan_filter_device_s
{
  struct ble_addr_s addr;
  uint32_t id;
  dev_timer_value_t first_seen;
  dev_timer_value_t last_seen;
  enum ble_scanner_policy_e policy;
  uint8_t ad[62];
  uint8_t ad_len;
  int16_t rssi;
  bool_t active : 1;
  bool_t connectable : 1;
  bool_t scannable : 1;
  bool_t scanned : 1;
  bool_t known : 1;
};

struct ble_scan_filter_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  enum ble_scanner_policy_e (*device_updated)(void *delegate, struct net_layer_s *layer,
                                              const struct ble_scan_filter_device_s *device);
};

STRUCT_COMPOSE(ble_scan_filter_delegate_vtable_s, base);

struct ble_scan_filter_param_s
{
  struct ble_security_db_s *peerdb;
  struct ble_scanner_param_s scan_params;
};

#endif
