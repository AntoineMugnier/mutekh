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

#ifndef BLE_GAP_H_
#define BLE_GAP_H_

#include <hexo/types.h>

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Network layer definition for Generic Access Profile

   This layer is not a layer per-se, but implements parts of the GAP
   behavior for devices.

   In particular, this layer tries to enforce connection parameters if
   they are declared in the attached GATT DB.

   This layer is expected to be bound to LLCP in the stack.

   There is a generic implementation of this layer in the library that
   can be created through @ref ble_gap_create.
*/

struct net_scheduler_s;
struct net_layer_s;

/**
   @this defines parameters for instanciating a GAP layer.
 */
struct ble_gap_params_s
{
  /** A pointer to GATT DB exposed by device. */
  struct ble_gattdb_s *db;
  /** A pointer to L2CAP signaling layer. */
  struct net_layer_s *sig;
};

/**
   Network request type for updating connectino parameters.
 */
#define BLE_GAP_CONN_PARAMS_UPDATE 0x43757064

/**
   @this defines a task the GAP layer may send to LLCP or L2CAP
   Signalling Layers for asking peer device to update its connection
   parameters.  This is done whatever the side the connection we are
   on.  LLCP and Signalling layers are responsible for taking relevant
   action.
*/
struct ble_gap_conn_params_update_s
{
  struct net_task_s task;

  uint16_t interval_min, interval_max;
  uint16_t slave_latency;
  uint16_t timeout;
};

STRUCT_COMPOSE(ble_gap_conn_params_update_s, task);

#endif
