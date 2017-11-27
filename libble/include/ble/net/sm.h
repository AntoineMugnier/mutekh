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

   SM flow is the following:

   Central                                   Peripheral

   [Case 1: request from peripheral]

                                             Application calls ble_sm_pairing_request
                                           < Sends a security request packet to peer
   Receives a security request packet    < - SM waits a pairing request

   [Case 2: request from central]

   Application calls ble_sm_pairing_request

   [General case follows]

   SM calls pairing_requested on delegate
   Delegate calls ble_sm_pairing_accept
   SM sends a pairing request packet     >
   SM waits pairing response             - > SM receives a pairing request packet
                                             SM calls pairing_requested on delegate
                                             Delegate calls ble_sm_pairing_accept
                                           < SM sends a pairing response packet
   SM receives a pairing response packet <
   SM computes pairing mode                  SM computes pairing mode
   SM calls VM with relevant entry point     SM calls VM with relevant entry point

   [Legacy pairing mode VM flow]

   VM generates Random                       VM generates Random
   VM computes Confirm                       VM computes Confirm
   VM sends Confirm                      > - VM waits Confirm
   VM waits Confirm                      - > VM receives Confirm
                                           < VM sends Confirm
   VM receives Confirm                   < - VM waits Random
   VM sends Random                       >
   VM waits Random                       - > VM receives Random
                                             VM checks received Random
                                           < VM sends Random
   VM receives Random                    <
   VM checks received Random
   VM computes STK                           VM computes STK
   Pairing Done [App starts encrpyption]     Pairing Done
   VM Waits encryption                       VM Waits encryption

   Encryption started                        Encryption started

   VM Distributes LTK, ID, IRK, ...      > > VM Waits LTK, ID, IRK, ...
   VM Waits LTK, ID, IRK, ...            < < VM Distributes LTK, ID, IRK, ...
   Bonding done                              Bonding done
*/

#include <hexo/types.h>
#include <net/task.h>
#include <net/layer.h>

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

  /**
     Notifies the application a pairing was requested, depending on
     role of the device, this may either:
     - for central, one of the following:
       * in response to a call to ble_sm_pairing_request
       * in response to a security request from peer peripheral
     - for peripheral: in response to a pairing request

     Use should call back ble_sm_pairing_accept if it accepts the
     request.
  */
  void (*pairing_requested)(void *delegate, struct net_layer_s *layer,
                            bool_t bonding);

  /**
     Notifies the application a pairing failed, whatever the moment it
     happened.
  */
  void (*pairing_failed)(void *delegate, struct net_layer_s *layer,
                         enum sm_reason reason);

  /**
     Notifies the application a pairing went to its end correctly
     (i.e. an STK key was exchanged). It is up to the application to
     start encryption afterwards.
  */
  void (*pairing_success)(void *delegate, struct net_layer_s *layer);

  /**
     Notifies the application a bonding went to its end
     correctly. This happens after Central and Peripheral exchagned
     LTK, Indentification parameters, IRK, CSRK and LK, if required.
     Encryption may be restarted with LTK afterwards, if application
     wants (optional, but some peripheral implementations require this
     to consider link is secured).
   */
  void (*bonding_success)(void *delegate, struct net_layer_s *layer);
};

STRUCT_COMPOSE(ble_sm_delegate_vtable_s, base);

/**
   Initiates a pairing request with optional bonding. Depending on GAP
   role of the device, this may either:
   - for central: call back `pairing_requested` of delegate,
   - for peripheral: sent a security request to central's SM.
 */
ALWAYS_INLINE void ble_sm_pairing_request(struct net_layer_s *layer,
                                          bool_t mitm_protection,
                                          bool_t bonding)
{
  const struct ble_sm_handler_s *handler = const_ble_sm_handler_s_from_base(layer->handler);
  handler->pairing_request(layer, mitm_protection, bonding);
}

/**
   Positive application response to pairing_requested delegate callback.
 */
ALWAYS_INLINE void ble_sm_pairing_accept(struct net_layer_s *layer,
                                         bool_t mitm_protection,
                                         uint32_t pin,
                                         const void *oob_data)
{
  const struct ble_sm_handler_s *handler = const_ble_sm_handler_s_from_base(layer->handler);
  handler->pairing_accept(layer, mitm_protection, pin, oob_data);
}

/**
   Negative application response to pairing_requested delegate callback.
 */
ALWAYS_INLINE void ble_sm_pairing_abort(struct net_layer_s *layer, enum sm_reason reason)
{
  const struct ble_sm_handler_s *handler = const_ble_sm_handler_s_from_base(layer->handler);
  handler->pairing_abort(layer, reason);
}

#endif
