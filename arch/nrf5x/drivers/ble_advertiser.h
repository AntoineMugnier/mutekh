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
#ifndef NRF5X_BLE_ADVERTISER_H_
#define NRF5X_BLE_ADVERTISER_H_

#include <device/class/ble_radio.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>

struct net_scheduler_s;
struct net_layer_s;
struct net_layer_delegate_vtable_s;
struct nrf5x_ble_private_s;

error_t nrf5x_ble_advertiser_create(struct net_scheduler_s *scheduler,
                                    struct nrf5x_ble_private_s *priv,
                                    const void *params,
                                    void *delegate,
                                    const struct net_layer_delegate_vtable_s *delegate_vtable,
                                    struct net_layer_s **layer);

#endif
