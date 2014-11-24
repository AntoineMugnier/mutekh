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
   @module {Bluetooth Low Energy library}
   @short Network layer definition for Link-Layer Control Protocol (LLCP)

   This header defines Network Layer API for Link-Layer Control
   Protocol.

   LLCP takes care of most aspects of the out-of-band communication of
   BLE:
   @list
   @item connection parameters negociation and updates,
   @item channel mapping,
   @item encryption setup,
   @item link termination.
   @end list

   To this extent, LLCP layer needs to communicate with:
   @list
   @item physical layer (for updating connection parameters and channels),
   @item link layer (for enabling / disabling cryptography),
   @item Security DB (for key lookup),
   @end list

   LLCP is expected to be bound on a @ref {@ble/net/link.h}
   {link layer}.

   There is a generic implementation of this layer in the library that
   can be created through @ref ble_llcp_create.
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/task.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>
#include <ble/protocol/llcp.h>
#include <ble/protocol/gap.h>

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
  bool_t authenticated;
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
  error_t (*encryption_enable)(struct net_layer_s *layer);
};

STRUCT_COMPOSE(ble_llcp_handler_s, base);

ALWAYS_INLINE void ble_llcp_connection_close(struct net_layer_s *layer)
{
  const struct ble_llcp_handler_s *handler = const_ble_llcp_handler_s_from_base(layer->handler);
  handler->connection_close(layer);
}

ALWAYS_INLINE error_t ble_llcp_encryption_enable(struct net_layer_s *layer)
{
  const struct ble_llcp_handler_s *handler = const_ble_llcp_handler_s_from_base(layer->handler);
  return handler->encryption_enable(layer);
}

struct ble_llcp_params_s
{
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s *rng;
  struct ble_peer_s *peer;
#endif
  struct ble_conn_timing_param_s conn_timing;
  struct ble_gap_preferred_conn_params_s wanted_timing;
};

#endif
