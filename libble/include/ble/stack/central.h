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
   @short BLE central stack

   @section {Description}

   This utility component creates a full central stack for handling
   a connection.

   It is created from a connection request packet and a GATT database.

   @end section
*/

#ifndef BLE_STACK_CENTRAL_H
#define BLE_STACK_CENTRAL_H

#include <hexo/types.h>
#include <errno.h>
#include <ble/net/slave.h>
#include <ble/net/l2cap.h>
#if defined(CONFIG_BLE_CRYPTO)
#include <ble/net/sm.h>
#endif
#include <ble/net/signalling.h>
#include <ble/net/gatt.h>
#include <ble/net/gap.h>
#include <net/scheduler.h>

struct ble_central_handler_s;
struct dev_rng_s;

struct ble_central_s
{
  struct net_scheduler_s *scheduler;
  const struct ble_central_handler_s *handler;
  struct ble_slave_s slave;
};

STRUCT_COMPOSE(ble_central_s, slave);

struct ble_central_handler_s
{
  void (*dropped)(struct ble_central_s *peri,
                  uint8_t reason);
};

error_t ble_central_init(
  struct ble_central_s *peri,
  struct net_scheduler_s *sched,
  const struct ble_central_handler_s *handler,
  const char *radio,
  struct dev_rng_s *rng,
  const char *aes_dev,
  struct ble_gatt_db_s *gattdb,
  struct ble_peer_s *peer,
  dev_timer_value_t connection_packet_time,
  const struct buffer_s *connect_packet);

#endif
