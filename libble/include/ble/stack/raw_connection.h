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

#ifndef BLE_STACK_RAW_CONN_H
#define BLE_STACK_RAW_CONN_H

#include <hexo/types.h>
#include <errno.h>
#include <device/class/timer.h>

struct ble_stack_raw_conn_s;
struct net_layer_s;
struct ble_stack_context_s;
struct ble_addr_s;

struct ble_stack_raw_conn_handler_s
{
  void (*opened)(struct ble_stack_raw_conn_s *conn);
  void (*closed)(struct ble_stack_raw_conn_s *conn, uint8_t reason);
};

struct ble_stack_raw_conn_s
{
  struct ble_stack_context_s *context;
  const struct ble_stack_raw_conn_handler_s *handler;

  struct net_layer_s *adv;
  struct net_layer_s *phy;
};

error_t ble_stack_raw_conn_init(struct ble_stack_raw_conn_s *conn,
                                struct ble_stack_context_s *context,
                                const struct ble_stack_raw_conn_handler_s *handler);

error_t ble_stack_raw_conn_close(struct ble_stack_raw_conn_s *conn);
error_t ble_stack_raw_conn_advertise(struct ble_stack_raw_conn_s *conn);
error_t ble_stack_raw_conn_connect(struct ble_stack_raw_conn_s *conn, const struct ble_addr_s *addr);

#endif
