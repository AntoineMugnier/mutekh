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
#include <hexo/decls.h>
#include <net/scheduler.h>

/**
   @file
   @module{BLE library}
   @short Generic Access Profile network layer

   @section {Description}

   This layer is not a layer per-se, but implement GAP behavior
   (advertising, pairing/bonding) for devices.

   @end section
*/

#define BLE_LAYER_TYPE_GAP NET_LAYER_TYPE('G', 'A', 'P', ' ')

struct ble_gap_handler_s;

/**
 BLE Generic Access Profile layer
 */
struct ble_gap_s
{
  struct net_layer_s layer;
  struct net_layer_s *sig;
  const struct ble_gap_handler_s *handler;
};

STRUCT_COMPOSE(ble_gap_s, layer);

struct ble_gap_handler_s
{
  void (*destroyed)(struct ble_gap_s *gap);
};

error_t ble_gap_init(
  struct ble_gap_s *gap,
  const struct ble_gap_handler_s *handler,
  struct net_scheduler_s *scheduler,
  struct net_layer_s *sig);

void ble_gap_cleanup(
  struct ble_gap_s *gap);

#endif
