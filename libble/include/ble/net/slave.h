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

#ifndef BLE_NET_SLAVE_H_
#define BLE_NET_SLAVE_H_

/**
   @file
   @module{BLE library}
   @short Slave connection layer

   @section {Description}

   This handles a connection from the slave side.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>
#include <ble/protocol/error.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>

#include <ble/util/channel_mapper.h>
#include <ble/util/timing_mapper.h>

#include <device/class/ble_radio.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>

struct buffer_s;
struct ble_slave_handler_s;
struct ble_peer_s;

enum ble_slave_layer_e
{
  BLE_SLAVE_LAYER_L2CAP,
  BLE_SLAVE_LAYER_GAP,
};

#define BLE_CONN_PARAMS_UPDATE 0x43757064

struct ble_conn_params_update_task_s
{
  struct net_task_s task;

  uint16_t interval_min, interval_max;
  uint16_t slave_latency;
  uint16_t timeout;
};

STRUCT_COMPOSE(ble_conn_params_update_task_s, task);

struct ble_slave_handler_s
{
  struct net_layer_handler_s base;

  void (*connection_close)(struct net_layer_s *layer);
};

STRUCT_COMPOSE(ble_slave_handler_s, base);

struct ble_slave_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*connection_closed)(void *delegate, struct net_layer_s *layer, uint8_t reason);
};

STRUCT_COMPOSE(ble_slave_delegate_vtable_s, base);

struct ble_slave_param_s
{
  struct ble_adv_connect_s conn_req;
  dev_timer_value_t connect_packet_timestamp;
  struct ble_peer_s *peer;
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s *rng;
#endif
};

ALWAYS_INLINE
void ble_slave_connection_close(struct net_layer_s *layer)
{
  const struct ble_slave_handler_s *handler
    = const_ble_slave_handler_s_from_base(layer->handler);
  handler->connection_close(layer);
}

#endif
