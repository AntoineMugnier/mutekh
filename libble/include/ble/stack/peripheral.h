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

/**
   @file
   @module{BLE library}
   @short BLE peripheral stack

   @section {Description}

   This utility component creates a full peripheral stack for handling
   a connection.

   It is created from a connection request packet and a GATT database.

   @end section
*/

#ifndef BLE_STACK_PERIPHERAL_H
#define BLE_STACK_PERIPHERAL_H

#include <hexo/types.h>
#include <errno.h>
#include <ble/net/phy.h>
#include <ble/net/l2cap.h>
#include <ble/net/signalling.h>
#include <ble/net/gatt.h>
#include <ble/net/gap.h>
#include <net/scheduler.h>

#include "context.h"

struct ble_peripheral_handler_s;
struct dev_rng_s;
struct ble_peripheral_s;

enum ble_peripheral_state_e {
  BLE_PERIPHERAL_IDLE,
  BLE_PERIPHERAL_RECONNECTING,
  BLE_PERIPHERAL_ADVERTISING,
  BLE_PERIPHERAL_PAIRING,
  BLE_PERIPHERAL_CONNECTED,
};

enum ble_peripheral_mode_e {
  BLE_PERIPHERAL_DISCOVERABLE = 1,
  BLE_PERIPHERAL_PAIRABLE = 2,
  BLE_PERIPHERAL_CONNECTABLE = 4,
};

struct ble_peripheral_handler_s
{
  struct ble_stack_connection_handler_s base;
  bool_t (*connection_requested)(struct ble_peripheral_s *peri,
                                 const struct ble_addr_s *addr);
  void (*state_changed)(struct ble_peripheral_s *peri, enum ble_peripheral_state_e state);
};

STRUCT_COMPOSE(ble_peripheral_handler_s, base);

struct ble_peripheral_params_s
{
  uint32_t adv_interval_ms;
};

struct ble_peripheral_s
{
  struct ble_stack_context_s *context;
  const struct ble_peripheral_handler_s *handler;
  struct ble_stack_connection_s conn;
  struct net_layer_s *adv;
  struct ble_addr_s addr;
  enum ble_peripheral_state_e last_state : 8;
  uint8_t mode;
  struct ble_peripheral_params_s params;
};

STRUCT_COMPOSE(ble_peripheral_s, conn);

error_t ble_peripheral_init(
  struct ble_peripheral_s *peri,
  const struct ble_peripheral_params_s *params,
  const struct ble_peripheral_handler_s *handler,
  struct ble_stack_context_s *context);

void ble_peripheral_mode_set(struct ble_peripheral_s *peri, uint8_t mode);

#endif
