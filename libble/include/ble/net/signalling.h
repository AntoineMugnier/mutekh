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

#ifndef BLE_SIGNALLING_H_
#define BLE_SIGNALLING_H_

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/layer.h>

#define BLE_LAYER_TYPE_SIGNALLING NET_LAYER_TYPE('S', 'i', 'g', 'n')

struct ble_signalling_handler_s;

/**
 BLE L2CAP signalling layer.

 Handles optional/advanced features of BLE connections:
 - Disconnection requests,
 - Connection parameter update requests to master,
 - Credit-based connection flow control,
 - Ping.
 */
struct ble_signalling_s
{
  struct net_layer_s layer;
  const struct ble_signalling_handler_s *handler;
  struct net_task_s *pending_conn_params;
  uint8_t pending_conn_params_identifier;
  uint8_t identifier;
};

STRUCT_COMPOSE(ble_signalling_s, layer);

struct ble_signalling_handler_s
{
  void (*destroyed)(struct ble_signalling_s *sig);
};

#define BLE_SIG_CONN_PARAMS_UPDATE NET_LAYER_TYPE('C','U','p','d')

struct ble_signalling_conn_params_update_task_s
{
  struct net_task_s task;

  uint16_t interval_min, interval_max;
  uint16_t slave_latency;
  uint16_t timeout;
};

STRUCT_COMPOSE(ble_signalling_conn_params_update_task_s, task);

error_t ble_signalling_init(
  struct ble_signalling_s *signalling,
  const struct ble_signalling_handler_s *handler,
  struct net_scheduler_s *scheduler);

void ble_signalling_cleanup(
  struct ble_signalling_s *signalling);

#endif
