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
   @module{BLE library}
   @short Generic Access Profile network layer

   @section {Description}

   This layer is not a layer per-se, but implement GAP behavior
   (advertising, pairing/bonding) for devices.

   @end section
*/

struct net_scheduler_s;
struct net_layer_s;

struct ble_gap_params_s
{
  struct ble_gattdb_s *db;
  struct net_layer_s *sig;
};

#define BLE_GAP_CONN_PARAMS_UPDATE 0x43757064

struct ble_gap_conn_params_update_s
{
  struct net_task_s task;

  uint16_t interval_min, interval_max;
  uint16_t slave_latency;
  uint16_t timeout;
};

STRUCT_COMPOSE(ble_gap_conn_params_update_s, task);

#endif
