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

#ifndef BLE_NET_MASTER_H_
#define BLE_NET_MASTER_H_

/**
   @file
   @module{BLE library}
   @short Master connection layer

   @section {Description}

   This handles a connection from the master side.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>
#include <ble/protocol/error.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>

#include <ble/util/channel_mapper.h>
#include <ble/util/timing_mapper.h>

#include <device/class/ble_radio.h>
#include <device/class/timer.h>

struct buffer_s;
struct ble_master_handler_s;

enum ble_master_layer_e
{
  BLE_MASTER_LAYER_L2CAP,
  BLE_MASTER_LAYER_GAP,
};

struct ble_master_parameters_s
{
  struct ble_adv_connect_s conn_req;
  dev_timer_value_t connect_packet_timestamp;
  struct ble_peer_s *peer;
};

#endif
