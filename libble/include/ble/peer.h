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

#ifndef BLE_PEER_H_
#define BLE_PEER_H_

/**
   @file
   @module{BLE library}
   @short Peer

   @section {Description}

   This is a peer representation

   @end section
*/

#include <hexo/types.h>
#include <ble/protocol/address.h>

#if defined(CONFIG_BLE_SECURITY_DB)
#include <ble/security_db.h>
#endif

struct ble_security_db_s;

#define BLE_SUBSCRIBED_CHAR_COUNT 8

struct ble_subscription_s {
  uint8_t service_index : 3;
  uint8_t char_index : 4;
  uint8_t mode : 1;
} __attribute__((__packed__));

struct ble_peer_s
{
  struct ble_subscription_s subscriptions[BLE_SUBSCRIBED_CHAR_COUNT];
  uint64_t id;

  struct ble_addr_s lookup_addr;
  struct ble_addr_s addr;

#if defined(CONFIG_BLE_CRYPTO)
  uint8_t stk[16];
  uint8_t irk[16];

  struct ble_security_db_s *db;

# if defined(CONFIG_BLE_SECURITY_DB)

  bool_t irk_present : 1;
  bool_t paired : 1;
  bool_t bonded : 1;
  bool_t mitm_protection : 1;
  bool_t secure_pairing : 1;
# endif
  bool_t stk_present : 1;
#endif

  bool_t addr_present : 1;
  bool_t dirty : 1;
  bool_t subscriptions_dirty : 1;
};

void ble_peer_init(struct ble_peer_s *peer,
                   struct ble_security_db_s *db,
                   const struct ble_addr_s *addr);

error_t ble_peer_reset(struct ble_peer_s *peer);

void ble_peer_addr_set(struct ble_peer_s *peer, const struct ble_addr_s *addr);

#if defined(CONFIG_BLE_CRYPTO)

/**
   Retrieve Session Key from entry.
 */
error_t ble_peer_sk_get(struct ble_peer_s *peer,
                        const uint8_t *skd,
                        const uint8_t *rand,
                        const uint16_t ediv,
                        uint8_t *sk);

#if defined(CONFIG_BLE_SECURITY_DB)
error_t ble_peer_ltk_get(struct ble_peer_s *peer, uint8_t *ltk);
error_t ble_peer_csrk_get(struct ble_peer_s *peer, uint8_t *csrk);
error_t ble_peer_id_get(struct ble_peer_s *peer, uint8_t *random, uint16_t *ediv);

void ble_peer_irk_set(struct ble_peer_s *peer, const uint8_t *irk);

#endif

error_t ble_peer_paired(struct ble_peer_s *peer,
                        bool_t bonded, bool_t mitm_protection, bool_t secure_pairing,
                        const uint8_t *stk);

error_t ble_peer_save(struct ble_peer_s *peer);

#endif

void ble_peer_subscriptions_set(struct ble_peer_s *peer,
                                struct ble_subscription_s subscriptions[static BLE_SUBSCRIBED_CHAR_COUNT]);

#endif
