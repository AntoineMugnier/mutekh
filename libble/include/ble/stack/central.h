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

#ifndef BLE_STACK_CENTRAL_H
#define BLE_STACK_CENTRAL_H

#include <hexo/types.h>
#include <errno.h>
#include <ble/net/slave.h>
#include <ble/net/l2cap.h>
#include <ble/net/sm.h>
#include <ble/net/signalling.h>
#include <ble/net/gatt.h>
#include <ble/net/gap.h>
#include <net/scheduler.h>

struct ble_central_handler_s;

struct ble_central_s
{
  struct net_scheduler_s scheduler;
  struct ble_slave_s slave;
  struct ble_l2cap_s l2cap;
  struct ble_sm_s sm;
  struct ble_signalling_s signalling;
  struct ble_gatt_s gatt;
  struct ble_gap_s gap;
  const struct ble_central_handler_s *handler;
  uint8_t reason;
};

STRUCT_COMPOSE(ble_central_s, slave);
STRUCT_COMPOSE(ble_central_s, scheduler);

struct ble_central_handler_s
{
  void (*dropped)(struct ble_central_s *peri,
                  uint8_t reason);
};

error_t ble_central_init(
  struct ble_central_s *peri,
  const struct ble_central_handler_s *handler,
  const char *radio,
  const char *rtc,
  struct buffer_pool_s *packet_pool,
  struct ble_gatt_db_s *gattdb,
  dev_timer_value_t connection_packet_time,
  const struct buffer_s *connect_packet);

#endif
