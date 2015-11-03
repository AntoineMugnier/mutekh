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

#ifndef BLE_LLCP_H_
#define BLE_LLCP_H_

/**
   @file
   @module{BLE library}
   @short Link-Layer Control Protocol network layer

   @section {Description}

   This implements LE-C channel handling of a connection.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/task.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>
#include <ble/protocol/data.h>

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

enum ble_llcp_child_e
{
  BLE_LLCP_CHILD_GAP,
};

struct ble_llcp_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*connection_closed)(void *delegate, struct net_layer_s *layer, uint8_t reason);
};

STRUCT_COMPOSE(ble_llcp_delegate_vtable_s, base);

enum ble_llcp_task_id_e
{
  BLE_LLCP_CONNECTION_PARAMETERS_UPDATE = 0x11c20000,
  BLE_LLCP_CHANNEL_MAP_UPDATE,
  BLE_LLCP_CONNECTION_PARAMETERS_REQ,
  BLE_LLCP_ENCRYPTION_SETUP,
  BLE_LLCP_LENGTH_UPDATE,
  BLE_LLCP_PING,
};

struct ble_llcp_connection_parameters_update_s
{
  struct net_task_s task;

  struct ble_conn_params_update update;
};

STRUCT_COMPOSE(ble_llcp_connection_parameters_update_s, task);

struct ble_llcp_channel_map_update_s
{
  struct net_task_s task;

  uint64_t channel_map;
  uint16_t instant;
};

STRUCT_COMPOSE(ble_llcp_channel_map_update_s, task);

struct ble_llcp_encryption_setup_s
{
  struct net_task_s task;

  uint8_t sk[16];
  uint8_t iv[8];
};

STRUCT_COMPOSE(ble_llcp_encryption_setup_s, task);

struct ble_llcp_length_update_s
{
  struct net_task_s task;

  uint16_t rx_mtu;
};

STRUCT_COMPOSE(ble_llcp_length_update_s, task);


struct ble_llcp_handler_s
{
  struct net_layer_handler_s base;

  void (*connection_close)(struct net_layer_s *layer);
};

STRUCT_COMPOSE(ble_llcp_handler_s, base);

ALWAYS_INLINE void ble_llcp_connection_close(struct net_layer_s *layer)
{
  const struct ble_llcp_handler_s *handler = const_ble_llcp_handler_s_from_base(layer->handler);
  handler->connection_close(layer);
}

struct ble_llcp_params_s
{
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s *rng;
  struct ble_peer_s *peer;
#endif
};

#endif
