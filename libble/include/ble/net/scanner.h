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

#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

struct ble_scanner_param_s
{
  dev_timer_delay_t interval;
  dev_timer_delay_t duration;
  struct ble_addr_s local_addr;
};

struct ble_scanner_device_s
{
  struct ble_addr_s addr;
  dev_timer_value_t first_seen;
  dev_timer_value_t last_seen;
  uint8_t ad[62];
  uint8_t adv_ad_len;
  uint8_t scan_ad_len;
  bool_t connectable;
};

struct ble_scanner_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*device_added)(void *delegate, struct net_layer_s *layer, struct ble_scanner_device_s *dev);
  void (*device_updated)(void *delegate, struct net_layer_s *layer, struct ble_scanner_device_s *dev);
  void (*device_removed)(void *delegate, struct net_layer_s *layer, struct ble_scanner_device_s *dev);
};

STRUCT_COMPOSE(ble_scanner_delegate_vtable_s, base);

#endif
