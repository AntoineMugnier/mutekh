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

#ifndef BLE_NET_GENERIC_H_
#define BLE_NET_GENERIC_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Generic BLE network layer factories
*/

#include <hexo/types.h>
#include <net/layer.h>

struct net_layer_delegate_vtable_s;
struct net_scheduler_s;
struct net_layer_s;

error_t ble_link_create(struct net_scheduler_s *scheduler,
                      const void *params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                      struct net_layer_s **layer);

error_t ble_llcp_create(struct net_scheduler_s *scheduler,
                        const void *params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                        struct net_layer_s **layer);

error_t ble_l2cap_create(struct net_scheduler_s *scheduler,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer);

error_t ble_att_create(struct net_scheduler_s *scheduler,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                       struct net_layer_s **layer);

error_t ble_gatt_create(struct net_scheduler_s *scheduler,
                        const void *params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                        struct net_layer_s **layer);

error_t ble_signaling_create(struct net_scheduler_s *scheduler,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer);

error_t ble_sm_create(struct net_scheduler_s *scheduler,
                      const void *params,
                      void *delegate,
                      const struct net_layer_delegate_vtable_s *delegate_vtable,
                      struct net_layer_s **layer);

error_t ble_gap_create(struct net_scheduler_s *scheduler,
                       const void *params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                       struct net_layer_s **layer);

error_t ble_scan_filter_create(struct net_scheduler_s *scheduler,
                               const void *params,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable,
                               struct net_layer_s **layer);

#endif
