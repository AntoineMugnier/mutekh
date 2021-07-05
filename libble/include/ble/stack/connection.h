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

#ifndef BLE_STACK_CONNECTION_H_
#define BLE_STACK_CONNECTION_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Device connection stack utility

   @this contains all declarations for a Connection handler.
*/

#include <hexo/types.h>
#include <device/class/timer.h>
#include <ble/peer.h>
#if defined(CONFIG_BLE_CRYPTO)
# include <ble/protocol/sm.h>
#endif
#include <ble/net/phy.h>

struct net_layer_s;
struct ble_stack_context_s;
struct ble_stack_context_handler_s;
struct ble_gap_preferred_conn_params_s;
struct ble_adv_connect_s;

struct ble_stack_connection_s;

struct ble_stack_connection_handler_s
{
#if defined(CONFIG_BLE_CRYPTO)
  void (*pairing_requested)(struct ble_stack_connection_s *conn,
                            bool_t bonding);
  void (*pairing_failed)(struct ble_stack_connection_s *conn,
                         enum sm_reason reason);
  void (*bonding_success)(struct ble_stack_connection_s *conn);
#endif
  void (*connection_closed)(struct ble_stack_connection_s *conn,
                            uint8_t reason);
};

/** @internal */
struct ble_stack_context_handler_s
{
  void (*pairing_success)(struct ble_stack_connection_s *conn);

  void (*state_changed)(struct ble_stack_connection_s *conn,
                        bool_t connected);
};

/**
   @this is a connection stack utility.  This contains reference to
   layers that are alive while connected.
 */
struct ble_stack_connection_s
{
  const struct ble_stack_connection_handler_s *chandler;
  const struct ble_stack_context_handler_s *handler;
  struct ble_stack_context_s *context;
  struct net_layer_s *phy;
  struct net_layer_s *llcp;
  struct net_layer_s *att;
#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif
  struct ble_peer_s peer;
  dev_timer_value_t connection_tk;
  bool_t is_master;
};

error_t ble_stack_connection_create(struct ble_stack_connection_s *conn,
                                    struct ble_stack_context_s *context,
                                    const struct ble_stack_connection_handler_s *chandler,
                                    const struct ble_stack_context_handler_s *handler,
                                    bool_t is_master,
                                    const struct ble_phy_params_s *phy_params,
                                    const struct ble_gap_preferred_conn_params_s *wanted_timing);

#if defined(CONFIG_BLE_CRYPTO)

void ble_stack_connection_pairing_request(struct ble_stack_connection_s *conn,
                                            bool_t mitm_protection,
                                            bool_t bonding);

void ble_stack_connection_pairing_accept(struct ble_stack_connection_s *conn,
                                           bool_t mitm_protection,
                                           uint32_t pin,
                                           const void *oob_data);

void ble_stack_connection_pairing_abort(struct ble_stack_connection_s *conn,
                                          enum sm_reason reason);

#endif

void ble_stack_connection_drop(struct ble_stack_connection_s *conn);

#endif
