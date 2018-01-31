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

#ifndef BLE_GATT_H_
#define BLE_GATT_H_

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short Network layer definition for GATT Server

   This header defines Network Layer API for a GATT server.  This
   layer mostly handles @ref {ble_att_transaction_s} {requests defined
   by the ATT layer}.

   GATT Server layer is backed by an attribute database and peer
   information that needs to be passed as construction parameters.

   There is a generic implementation of this layer in the library that
   can be created through @ref ble_gatt_create.
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/scheduler.h>

#include <ble/gattdb/client.h>
#include <ble/protocol/att.h>
#include <ble/peer.h>

/**
   @this defines instanciation parameters for a GATT server layer.
 */
struct ble_gatt_params_s
{
  /**
     Pointer to a peer structure filled with available information.
   */
  struct ble_peer_s *peer;

  /**
     Pointer to a GATT database.
   */
  struct ble_gattdb_s *db;
};

#endif
