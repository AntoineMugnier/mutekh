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
   @short BLE stack Central context

   @this contains definitions for a @ref {ble_central_s}
   {Central context stack utility}.
*/

#ifndef BLE_STACK_CENTRAL_H
#define BLE_STACK_CENTRAL_H

#include <hexo/types.h>
#include <mutek/kroutine.h>
#include <ble/net/scanner.h>
#include <ble/net/scan_filter.h>
#include <ble/protocol/address.h>
#include <ble/protocol/gap.h>
#include <ble/stack/connection.h>

struct ble_central_handler_s;
struct ble_central_s;
struct ble_stack_context_s;
struct net_layer_s;
struct dev_rng_s;

/**
   @this is a list of states a central may be in.
 */
enum ble_central_state_e {
  /** Central is not connected, not scanning */
  BLE_CENTRAL_IDLE,
  /** Central is not connected, scanning */
  BLE_CENTRAL_SCANNING,
  /** Central is connected to a non-paired device */
  BLE_CENTRAL_PAIRING,
  /** Central is connected to a paired device */
  BLE_CENTRAL_CONNECTED,
};

/**
   @this is a or-mask of device supported actions.  Mode should be
   updated through calls to @ref ble_central_mode_set.
 */
enum ble_central_mode_e {
  /** Central will scan */
  BLE_CENTRAL_CONNECTABLE = 1,
  /** Central will allow pairing requests */
  BLE_CENTRAL_PAIRABLE = 2,
};

/**
   @this defines the set of functions a central context can call back.
   They allow the central context to meet caller's expectations.
 */
struct ble_central_handler_s
{
  /**
     Connection-specific callbacks
   */
  struct ble_stack_connection_handler_s base;

  /**
     @this notifies the handler a connection got created to a given
     device.
  */
  void (*connection_opened)(struct ble_central_s *ctrl,
                            const struct ble_addr_s *addr);

  /**
     @this asks handler about what behavior to adopt with a given
     device that got scanned.
   */
  enum ble_scan_filter_policy_e (*device_policy)(struct ble_central_s *ctrl,
                                                 const struct ble_scan_filter_device_s *device);

  /**
     @this notifies the handler of new central state.  This will
     typically change on successful connection, connection drop, or
     mode change.
   */
  void (*state_changed)(struct ble_central_s *ctrl, enum ble_central_state_e state);
};

STRUCT_COMPOSE(ble_central_handler_s, base);

/**
   @this is the central parameter structure to pass to central state
   initialization.
 */
struct ble_central_params_s
{
  enum ble_phy_mode_e phy;
  /** Time between two scan window starts */
  uint32_t scan_interval_ms;
  /** Time to scan for */
  uint32_t scan_duration_ms;
  /** Initial connection parameters */
  struct ble_gap_preferred_conn_params_s conn;
};

/**
   @this is an utility for easy declaration of a typical Central role.
   This utility uses a @ref {ble_stack_context_s} {stack context} as
   data source, scans for advertising devices, and may connect to them
   if wanted.

   After connection creation, scanning stops util connection closes.
   To this extent, this utility handles only one concurrent
   connection.
 */
struct ble_central_s
{
  struct ble_stack_context_s *context;
  const struct ble_central_handler_s *handler;
  struct ble_stack_connection_s conn;
  struct net_layer_s *scan;
  struct net_layer_s *scan_filter;
  struct ble_addr_s addr;
  struct kroutine_s updater;

  enum ble_central_state_e last_state : 8;
  uint8_t mode;

  struct ble_gap_preferred_conn_params_s conn_params;
  struct ble_scanner_param_s params;
};

/**
   @this initializes a central stack context.
 */
error_t ble_central_init(
  struct ble_central_s *ctrl,
  const struct ble_central_params_s *params,
  const struct ble_central_handler_s *handler,
  struct ble_stack_context_s *context);

STRUCT_COMPOSE(ble_central_s, conn);

/**
   @this sets central mode.  If passed mode implies connection is not
   possible, current connection gets dropped, if any.
 */
void ble_central_mode_set(struct ble_central_s *ctrl, uint8_t mode);

/**
   @this asks the central context to connection to given device as
   soon as possible.
 */
void ble_central_connect(struct ble_central_s *ctrl,
                         const struct ble_addr_s *addr);

/**
   @this tell link-layer to start link encryption if keys are
   available.
 */
error_t ble_central_encryption_enable(struct ble_central_s *ctrl);

#endif
