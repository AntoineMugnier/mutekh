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
   @short BLE peripheral stack

   @section {Description}

   This utility component creates a full peripheral stack for handling
   a connection.

   It is created from a connection request packet and a GATT database.

   @end section
*/

#ifndef BLE_STACK_PERIPHERAL_H
#define BLE_STACK_PERIPHERAL_H

#include <hexo/types.h>
#include <errno.h>
#include <ble/net/slave.h>
#include <ble/net/l2cap.h>
#include <ble/net/sm.h>
#include <ble/net/signalling.h>
#include <ble/net/gatt.h>
#include <ble/net/gap.h>
#include <net/scheduler.h>

struct ble_peripheral_handler_s;
struct dev_rng_s;

struct ble_peripheral_s
{
  struct net_scheduler_s *scheduler;
  const struct ble_peripheral_handler_s *handler;
  struct ble_slave_s slave;
};

STRUCT_COMPOSE(ble_peripheral_s, slave);

struct ble_peripheral_handler_s
{
  void (*dropped)(struct ble_peripheral_s *peri,
                  uint8_t reason);
};

error_t ble_peripheral_init(
  struct ble_peripheral_s *peri,
  struct net_scheduler_s *sched,
  const struct ble_peripheral_handler_s *handler,
  const char *radio,
  struct dev_rng_s *rng,
  const char *aes_dev,
  struct ble_gatt_db_s *gattdb,
  struct ble_peer_s *peer,
  dev_timer_value_t connection_packet_time,
  const struct buffer_s *connect_packet);

#endif
