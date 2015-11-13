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

#include <gct/container_avl_p.h>

#define GCT_CONTAINER_ALGO_ble_scanner_device_set AVL_P

GCT_CONTAINER_TYPES    (ble_scanner_device_set, struct ble_scanner_device_s *, hash_entry);
GCT_CONTAINER_KEY_TYPES(ble_scanner_device_set, PTR, BLOB, addr, sizeof(struct ble_addr_s));
GCT_CONTAINER_KEY_FCNS (ble_scanner_device_set, ASC, ALWAYS_INLINE, ble_scanner_device_set, addr,
                        init, destroy, push, lookup, remove);

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

struct ble_scanner_param_s
{
  uint32_t interval_ms;
  uint32_t duration_ms;
  struct ble_addr_s local_addr;
};

struct ble_scanner_initiation_s
{
  struct ble_conn_timing_param_s timing;
  uint64_t channel_map;
  uint32_t access_address;
  uint32_t crc_init;
  uint16_t win_period_tk;
  uint16_t win_offset_tk;
  uint8_t hop;
  uint8_t sca;
  uint8_t win_size;
};

struct ble_scanner_handler_s
{
  struct net_layer_handler_s base;

  error_t (*params_update)(struct net_layer_s *layer, const struct ble_scanner_param_s *params);
  void (*scan_device)(struct net_layer_s *layer, const struct ble_addr_s *addr);
  void (*connect_device)(struct net_layer_s *layer,
                         const struct ble_addr_s *addr,
                         );
};

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
