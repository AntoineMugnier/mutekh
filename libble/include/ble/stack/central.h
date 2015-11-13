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

/**
   @file
   @module{BLE library}
   @short BLE central stack

   @section {Description}

   This utility component creates a full central stack for handling
   a connection.

   It is created from a connection request packet and a GATT database.

   @end section
*/

#ifndef BLE_STACK_CENTRAL_H
#define BLE_STACK_CENTRAL_H

#include <hexo/types.h>
#include <errno.h>
#if defined(CONFIG_BLE_CRYPTO)
#include <ble/protocol/sm.h>
#endif
#include <ble/protocol/address.h>
#include <device/class/timer.h>

struct ble_central_handler_s;
struct ble_central_s;
struct ble_stack_context_s;
struct net_layer_s;
struct dev_rng_s;

enum ble_central_state_e {
  BLE_CENTRAL_IDLE,
  BLE_CENTRAL_SCANNING,
  BLE_CENTRAL_PAIRING,
  BLE_CENTRAL_CONNECTED,
};

enum ble_central_mode_e {
  BLE_CENTRAL_CONNECTABLE = 1, // Defaults to whitelist only
  BLE_CENTRAL_PAIRABLE = 2,
};

struct ble_central_handler_s
{
#if defined(CONFIG_BLE_CRYPTO)
  void (*pairing_requested)(struct ble_central_s *peri, bool_t bonding);
  void (*pairing_failed)(struct ble_central_s *peri, enum sm_reason reason);
  void (*pairing_success)(struct ble_central_s *peri);
#endif
  void (*connection_opened)(struct ble_central_s *peri, const struct ble_addr_s *addr);
  void (*connection_closed)(struct ble_central_s *peri, uint8_t reason);
  void (*state_changed)(struct ble_central_s *peri, enum ble_central_state_e state);
};

struct ble_central_params_s
{
  uint32_t scan_interval_ms;
  uint32_t scan_duration_ms;
};

struct ble_central_s
{
  const struct ble_central_handler_s *handler;
  struct ble_stack_context_s *context;
  struct net_layer_s *master;
  struct net_layer_s *llcp;
#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif
  struct net_layer_s *scan;
  struct ble_peer_s peer;

  dev_timer_value_t connection_tk;
  struct ble_addr_s addr;

  enum ble_central_state_e last_state : 8;
  uint8_t mode;
  struct ble_central_params_s params;
};

error_t ble_central_init(
  struct ble_central_s *peri,
  const struct ble_central_params_s *params,
  const struct ble_central_handler_s *handler,
  struct ble_stack_context_s *context);

#if defined(CONFIG_BLE_CRYPTO)

void ble_central_pairing_request(struct ble_central_s *peri,
                                 bool_t mitm_protection,
                                 bool_t bonding);

void ble_central_pairing_accept(struct ble_central_s *peri,
                                bool_t mitm_protection,
                                uint32_t pin,
                                const void *oob_data);

void ble_central_pairing_abort(struct ble_central_s *peri,
                               enum sm_reason reason);

#endif

void ble_central_mode_set(struct ble_central_s *peri, uint8_t mode);

#endif
