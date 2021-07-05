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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#ifndef BLE_NET_ADV_H_
#define BLE_NET_ADV_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Network layer definition for Advertiser

   This header defines Network Layer API for advertiser.  This layer
   should be implemented by device drivers.

   Advertising layer needs @ref {ble_advertiser_param_s}
   {advertisement parameters} among which:
   @list
   @item Advertising interval window timers,
   @item Local address,
   @item Advertising data (AD),
   @item Advertising packet type to send.
   @end list

   Parameters can be updated while the layer is alive.

   If advertisement is connectable, advertiser layer may notify its
   delegate for connection request through
   @ref{ble_advertiser_delegate_vtable_s::connection_requested}{relevant
   vtable entry}.
*/

#include <hexo/types.h>
#include <net/task.h>
#include <net/layer.h>

#include <ble/protocol/radio.h>
#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

struct ble_advertiser_param_s
{
  uint32_t interval_ms;
  uint32_t delay_max_ms;
  int16_t tx_power; // In 0.125 dBm steps (1/8)
  struct ble_addr_s local_addr;
  bool_t connectable;
  const void *ad;
  size_t ad_len;
  enum ble_phy_mode_e phy;
};

struct ble_advertiser_handler_s
{
  struct net_layer_handler_s base;

  error_t (*params_update)(struct net_layer_s *layer, const struct ble_advertiser_param_s *params);
};

STRUCT_COMPOSE(ble_advertiser_handler_s, base);

struct ble_advertiser_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  /** Return whether advertising should continue */
  bool_t (*connection_requested)(void *delegate, struct net_layer_s *layer,
                                 const struct ble_adv_connect_s *params,
                                 dev_timer_value_t anchor);
};

STRUCT_COMPOSE(ble_advertiser_delegate_vtable_s, base);

ALWAYS_INLINE
error_t ble_advertiser_params_update(struct net_layer_s *layer,
                                     const struct ble_advertiser_param_s *params)
{
  const struct ble_advertiser_handler_s *handler
    = const_ble_advertiser_handler_s_from_base(layer->handler);

  return handler->params_update(layer, params);
}

#endif
