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
   @module{BLE library}
   @short GATT protocol layer

   @section {Description}

   This implements the Attribute protocol for LE links backed by a
   GATT database.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/scheduler.h>

#include <ble/gattdb/client.h>
#include <ble/protocol/att.h>
#include <ble/peer.h>

struct ble_gatt_params_s
{
  struct ble_peer_s *peer;
  struct ble_gattdb_s *db;
};

#endif
