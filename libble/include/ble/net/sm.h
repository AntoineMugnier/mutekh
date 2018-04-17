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

#ifndef BLE_SM_H_
#define BLE_SM_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Network layer definition for Security Manager (SM)

   This header defines Network Layer API for Security Manager
   Protocol.

   Security manager layer is involved in the pairing process.  Once
   paired, connections can be encrypted without SM layer taking
   action.

   SM layer expects its delegate to handle pairing requests and
   respond to them.

   SM is expected to be bound on a L2CAP layer.

   There is a generic implementation of this layer in the library that
   can be created through @ref ble_sm_create.
*/

#include <hexo/types.h>
#include <net/task.h>

#include <ble/protocol/address.h>
#include <ble/protocol/sm.h>

struct net_layer_s;
struct net_scheduler_s;
struct ble_peer_s;
struct dev_rng_s;

struct ble_sm_param_s
{
  struct ble_peer_s *peer;
  struct ble_addr_s local_addr;
  struct dev_rng_s *rng;
  struct device_crypto_s *crypto;
};

enum ble_sm_pairing_mode_e {
  BLE_SM_PAIRING_JUSTWORKS,
  BLE_SM_PAIRING_PIN,
  BLE_SM_PAIRING_OOB,
};

struct ble_sm_handler_s
{
  struct net_layer_handler_s base;

  void (*pairing_request)(struct net_layer_s *layer,
                          bool_t mitm_protection,
                          bool_t bonding);

  void (*pairing_accept)(struct net_layer_s *layer,
                         bool_t mitm_protection,
                         uint32_t pin,
                         const void *oob_data);

  void (*pairing_abort)(struct net_layer_s *layer, enum sm_reason reason);
};

STRUCT_COMPOSE(ble_sm_handler_s, base);

struct ble_sm_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*pairing_requested)(void *delegate, struct net_layer_s *layer,
                            bool_t bonding);

  void (*pairing_failed)(void *delegate, struct net_layer_s *layer,
                         enum sm_reason reason);

  void (*pairing_success)(void *delegate, struct net_layer_s *layer);

  void (*bonding_success)(void *delegate, struct net_layer_s *layer);
};

STRUCT_COMPOSE(ble_sm_delegate_vtable_s, base);

ALWAYS_INLINE void ble_sm_pairing_request(struct net_layer_s *layer,
                                          bool_t mitm_protection,
                                          bool_t bonding)
{
  const struct ble_sm_handler_s *handler = const_ble_sm_handler_s_from_base(layer->handler);
  handler->pairing_request(layer, mitm_protection, bonding);
}

ALWAYS_INLINE void ble_sm_pairing_accept(struct net_layer_s *layer, 
                                         bool_t mitm_protection,
                                         uint32_t pin,
                                         const void *oob_data)
{
  const struct ble_sm_handler_s *handler = const_ble_sm_handler_s_from_base(layer->handler);
  handler->pairing_accept(layer, mitm_protection, pin, oob_data);
}

ALWAYS_INLINE void ble_sm_pairing_abort(struct net_layer_s *layer, enum sm_reason reason)
{
  const struct ble_sm_handler_s *handler = const_ble_sm_handler_s_from_base(layer->handler);
  handler->pairing_abort(layer, reason);
}

#endif
