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

#ifndef LIBBLE_NET_L2CAP_H_
#define LIBBLE_NET_L2CAP_H_

/**
   @file
   @module{BLE library}
   @short L2CAP layer

   @section {Description}

   This handles L2CAP multiplexing.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>

struct ble_l2cap_s;
struct ble_sm_s;
struct ble_att_s;

#define BLE_LAYER_TYPE_L2CAP NET_LAYER_TYPE('L', 'C', 'a', 'p')

struct ble_l2cap_handler_s;

/**
 BLE L2CAP layer.

 Handles fragmentation and CID multiplexing.  For a compliant LE
 device, this layer expects SM and ATT layers.
 */
struct ble_l2cap_s
{
  struct net_layer_s layer;

  const struct ble_l2cap_handler_s *handler;
  struct net_layer_s *att;
  struct net_layer_s *sm;
  struct net_layer_s *signalling;
};

STRUCT_COMPOSE(ble_l2cap_s, layer);

struct ble_l2cap_handler_s
{
  void (*destroyed)(struct ble_l2cap_s *l2cap);
};

error_t ble_l2cap_init(
  struct ble_l2cap_s *l2cap,
  const struct ble_l2cap_handler_s *handler,
  struct net_scheduler_s *scheduler);

void ble_l2cap_cleanup(
  struct ble_l2cap_s *l2cap);

#endif
