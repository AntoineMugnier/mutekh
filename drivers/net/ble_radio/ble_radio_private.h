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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/
#ifndef BLE_RADIO_DRV_PRIVATE_H_
#define BLE_RADIO_DRV_PRIVATE_H_

#include <device/class/ble_radio.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>
struct net_scheduler_s;
struct net_layer_s;
struct net_layer_delegate_vtable_s;

struct ble_private_s {
  struct device_ble_radio_s radio;
  struct device_timer_s timer;
#if defined(CONFIG_BLE_CRYPTO)
  struct device_crypto_s crypto;
  struct dev_rng_s rng;
#endif
};

error_t ble_advertiser_create(struct net_scheduler_s *scheduler,
                              struct ble_private_s *priv,
                              const void *params,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer);

error_t ble_master_create(struct net_scheduler_s *scheduler,
                          struct ble_private_s *priv,
                          const void *params,
                          struct net_layer_s **layer);

error_t ble_slave_create(struct net_scheduler_s *scheduler,
                         struct ble_private_s *priv,
                         const void *params,
                         void *delegate,
                         const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer);

#endif
