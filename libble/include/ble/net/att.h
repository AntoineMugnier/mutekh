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

#ifndef BLE_ATT_H_
#define BLE_ATT_H_

/**
   @file
   @module{BLE library}
   @short Attribute protocol network layer

   @section {Description}

   This implements the Attribute protocol for LE links.  This layer
   actually is a demo for typical Att handling.  Usual ATT
   implementation using a GATT DB backend is in @ref {gatt.h}.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/scheduler.h>

#define BLE_LAYER_TYPE_ATT NET_LAYER_TYPE('A', 't', 't', '_')

struct ble_att_handler_s;

/**
 BLE Attribute protocol layer.
 */
struct ble_att_s
{
  struct net_layer_s layer;
  const struct ble_att_handler_s *handler;

  uint16_t server_mtu;
};

STRUCT_COMPOSE(ble_att_s, layer);

struct ble_att_pairing_task_s
{
  struct net_task_s task;
};

struct ble_att_handler_s
{
  void (*destroyed)(struct ble_att_s *att);
};

error_t ble_att_init(
  struct ble_att_s *att,
  const struct ble_att_handler_s *handler,
  struct net_scheduler_s *scheduler);

void ble_att_cleanup(
  struct ble_att_s *att);

#endif
