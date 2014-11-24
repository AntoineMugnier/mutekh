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

#include <ble/gatt/client.h>

struct ble_gatt_handler_s;

/**
 BLE GATT protocol layer.  This actually implements an ATT layer
 backed to a GATT Db.
 */
struct ble_gatt_s
{
  struct net_layer_s layer;
  struct ble_gatt_client_s client;
  const struct ble_gatt_handler_s *handler;

  uint16_t server_mtu;
};

STRUCT_COMPOSE(ble_gatt_s, layer);
STRUCT_COMPOSE(ble_gatt_s, client);

struct ble_gatt_pairing_task_s
{
  struct net_task_s task;
};

struct ble_gatt_handler_s
{
  void (*destroyed)(struct ble_gatt_s *gatt);
};

error_t ble_gatt_init(
  struct ble_gatt_s *gatt,
  const struct ble_gatt_handler_s *handler,
  struct net_scheduler_s *scheduler,
  struct ble_gatt_db_s *db);

void ble_gatt_cleanup(
  struct ble_gatt_s *gatt);

#endif
