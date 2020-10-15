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
   @module {Bluetooth Low Energy library}
   @short BLE stack Peripheral context

   @this contains definitions for a @ref {ble_peripheral_s}
   {Peripheral context stack utility}.
*/

#ifndef BLE_STACK_PERIPHERAL_H
#define BLE_STACK_PERIPHERAL_H

#include <hexo/types.h>
#include <ble/net/scanner.h>
#include <ble/net/scan_filter.h>
#include <ble/protocol/address.h>
#include <ble/protocol/radio.h>
#include <ble/stack/connection.h>

struct ble_peripheral_handler_s;
struct ble_stack_context_s;
struct dev_rng_s;
struct ble_peripheral_s;

/**
   @this is a list of states a peripheral may be in.
 */
enum ble_peripheral_state_e {
  /** Peripheral is not connected, not advertising */
  BLE_PERIPHERAL_IDLE,
  /** Peripheral is not connected, fast advertising */
  BLE_PERIPHERAL_RECONNECTING,
  /** Peripheral is not connected, normally advertising */
  BLE_PERIPHERAL_ADVERTISING,
  /** Peripheral is connected to a non-paired device */
  BLE_PERIPHERAL_PAIRING,
  /** Peripheral is connected to a paired device */
  BLE_PERIPHERAL_CONNECTED,
};

/**
   @this is a or-mask of device supported actions.  Mode should be
   updated through calls to @ref ble_peripheral_mode_set.
 */
enum ble_peripheral_mode_e {
  /** Peripheral will advertise */
  BLE_PERIPHERAL_DISCOVERABLE = 1,
  /** Accept connections from any host, even not in whitelist */
  BLE_PERIPHERAL_PAIRABLE = 2,
  /** Peripheral will accept connections */
  BLE_PERIPHERAL_CONNECTABLE = 4,
};

/**
   @this defines the set of functions a peripheral context can call
   back.  They allow the peripheral context to meet caller's
   expectations.
 */
struct ble_peripheral_handler_s
{
  /**
     Connection-specific callbacks
   */
  struct ble_stack_connection_handler_s base;

  /**
     @this asks the handler whether connection request from address
     @tt addr should be accepted.
   */
  bool_t (*connection_requested)(struct ble_peripheral_s *peri,
                                 const struct ble_addr_s *addr);

  /**
     @this notifies the handler of new peripheral state.  This will
     typically change on accepted connection request, connection drop,
     or mode change.
   */
  void (*state_changed)(struct ble_peripheral_s *peri,
                        enum ble_peripheral_state_e state);
};

STRUCT_COMPOSE(ble_peripheral_handler_s, base);

/**
   @this is peripheral parameters.
 */
struct ble_peripheral_params_s
{
  enum ble_phy_mode_e phy;
  /** Interval in milliseconds between two advertising events. */
  uint32_t adv_interval_ms;
};

/**
   @this is an utility for easy declaration of a typical Peripheral
   role.  This utility uses a @ref {ble_stack_context_s} {stack
   context} as data source, advertises for device properties, and
   waits for a connection request.

   Upon connection request, if @ref {ble_peripheral_handler_s}
   {handler} accepts the connection, advertising stops util connection
   closes.  To this extent, this utility handles only one concurrent
   connection.
 */
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

/**
   @this initializes a peripheral stack context.
 */
error_t ble_peripheral_init(
  struct ble_peripheral_s *peri,
  const struct ble_peripheral_params_s *params,
  const struct ble_peripheral_handler_s *handler,
  struct ble_stack_context_s *context);

/**
   @this sets peripheral mode.  If passed mode implies connection is
   not possible, current connection gets dropped, if any.
 */
void ble_peripheral_mode_set(struct ble_peripheral_s *peri, uint8_t mode);

void ble_peripheral_cleanup(struct ble_peripheral_s *peri);

#endif
