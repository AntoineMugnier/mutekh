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

#ifndef BLE_NET_SLAVE_H_
#define BLE_NET_SLAVE_H_

/**
   @file
   @module{BLE library}
   @short Slave connection layer

   @section {Description}

   This handles a connection from the slave side.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>
#include <device/class/timer.h>
#include <ble/protocol/error.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>

struct ble_slave_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*connection_lost)(void *delegate, struct net_layer_s *layer, uint8_t reason);
};

STRUCT_COMPOSE(ble_slave_delegate_vtable_s, base);

struct ble_slave_param_s
{
  struct ble_adv_connect_s conn_req;
  dev_timer_value_t connect_packet_timestamp;
};

#endif