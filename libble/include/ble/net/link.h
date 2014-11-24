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

#ifndef BLE_NET_LINK_H_
#define BLE_NET_LINK_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Network layer definition for Link-layer

   This handles a connection on low level.  In particular, this layer
   handles cryptography if enabled.  It also demuxes between @ref
   {BLE_LINK_CHILD_LLCP} {L2CAP} and @ref {BLE_LINK_CHILD_LLCP}
   {LLCP}.

   This layer is expected to be bound on top of a physical layer
   (either Master or Slave).

   The link layer handles throttling of data throughput, and may drop
   packets that are marked as unreliable is physical layer
   back-pressures too much.

   In order to notify link layer of current radio conditions, Phsycial
   layer should periodically update its current packet queue status to
   link layer through a @ref {ble_link_flow_update_s} {Link flow
   update task}.

   There is a generic implementation of this layer in the library that
   can be created through @ref ble_link_create.
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>
#include <net/task.h>

#include <ble/protocol/advertise.h>
#include <device/class/crypto.h>

struct buffer_s;
struct ble_link_handler_s;
struct ble_peer_s;

/**
   Link-layer children types.
 */
enum ble_link_child_type_e
{
  BLE_LINK_CHILD_LLCP,
  BLE_LINK_CHILD_L2CAP,
};

/**
   Task types for link-layer tasks
 */
enum ble_link_task_type_e
{
  /** This task type should be assigned to @ref ble_link_flow_update_s
      items. */
  BLE_LINK_FLOW_UPDATE = 0x465e4,
};

struct ble_link_param_s
{
#if defined(CONFIG_BLE_CRYPTO)
  struct device_crypto_s *crypto;
#endif
};

/** @this is a network notification type that should be send by
    physical layers to attached link layer.  It should carry type @ref
    BLE_LINK_FLOW_UPDATE. */
struct ble_link_flow_update_s
{
  struct net_task_s task;

  /** Count of packets that may be accepted by physical layer from now
      on.  This needs to be refreshed until more packets are
      accepted. */
  int32_t accepted_count;
};

STRUCT_COMPOSE(ble_link_flow_update_s, task);

#endif
