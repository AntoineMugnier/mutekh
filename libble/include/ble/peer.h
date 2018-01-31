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
   @module {Bluetooth Low Energy library}
   @short Peer data

   @this contains definitions for peer data definitions.
*/

#include <hexo/types.h>
#include <ble/protocol/address.h>

#include <ble/security_db.h>

struct ble_security_db_s;

#define BLE_SUBSCRIBED_CHAR_COUNT 8

/**
   @this is a subscription entry saved for a peer.
 */
struct ble_subscription_s {
  uint8_t registry_index : 3;
  uint8_t char_index : 4;
  uint8_t mode : 1;
} __attribute__((__packed__));

/**
   @this is data associated to a peer.  Peer can be identified from
   its adrress before connection, or from its Peer ID on encryption
   request.  Some entries in this object are derived from
   calculations, other are distributed by the peer itself on pairing
   and are saved verbatim.

   Peer also contains subscription information, which is the list of
   characteristics the peer subscribed to during last connection.
   They are saved/restored by GATT server.
 */
struct ble_peer_s
{
  /** Subscription list */
  struct ble_subscription_s subscriptions[BLE_SUBSCRIBED_CHAR_COUNT];
  /** Peer serial ID, allocated on pairing */
  uint64_t id;

  /** Peer lookup address, this may contain a resolvable address or
      public device address. */
  struct ble_addr_s lookup_addr;
  /** Peer public address */
  struct ble_addr_s addr;

  /** Peer-distributed rand */
  uint8_t rand[8];
  /** Peer-distributed ediv */
  uint16_t ediv;
  /** Peer-distributed LTK */
  uint8_t ltk[16];

#if defined(CONFIG_BLE_CRYPTO)
  /** STK negociated by SM during pairing Phase 2 */
  uint8_t stk[16];
  /** Peer-distributed Identity Resolving Key */
  uint8_t irk[16];

  /** Pointer to security DB context */
  struct ble_security_db_s *db;

  /** Whether peer is paired */
  bool_t paired : 1;
  
  /** Whether peer has an associated IRK */
  bool_t irk_present : 1;
  /** Whether peer is bonded */
  bool_t bonded : 1;
  /** Whether peer requested MITM protection while pairing */
  bool_t mitm_protection : 1;
  /** Whether pairing was secure */
  bool_t secure_pairing : 1;
  /** Whether we currently hold a valid STK */
  bool_t stk_present : 1;
#endif

  /** Whether peer @tt rand and @tt ediv are present */
  bool_t identity_present : 1;
  /** Whether peer LTK is valid */
  bool_t ltk_present : 1;
  /** Whether peer address is valid */
  bool_t addr_present : 1;
  /** Whether some data changed and should be flushed to non-volatile
      storage */
  bool_t dirty : 1;
  /** Whether subscription changed and should be flushed to
      non-volatile storage */
  bool_t subscriptions_dirty : 1;
};

/**
   @this initializes a peer as member of a given peer database with a
   given lookup adress.  Depending on lookup address, peer data may or
   may not be filled from database.
 */
void ble_peer_init(struct ble_peer_s *peer,
                   struct ble_security_db_s *db,
                   const struct ble_addr_s *addr);

/**
   @this resets peer data, only lookup address is preserved.
 */
error_t ble_peer_reset(struct ble_peer_s *peer);

/**
   @this sets peer's public address
 */
void ble_peer_addr_set(struct ble_peer_s *peer, const struct ble_addr_s *addr);

#if defined(CONFIG_BLE_CRYPTO)

/**
   @this retrieves Session Key from entry when we are on the slave
   side of connection, either derived from LTK or STK, depending if
   present in peer.
 */
error_t ble_peer_sk_get(struct ble_peer_s *peer,
                        const uint8_t *skd,
                        const uint8_t *rand,
                        const uint16_t ediv,
                        uint8_t *sk);

/**
   @this marks peer as paired.
 */
error_t ble_peer_paired(struct ble_peer_s *peer,
                        bool_t bonded, bool_t mitm_protection, bool_t secure_pairing,
                        const uint8_t *stk);

/**
   @this retrieves peer-distributed LTK
 */
error_t ble_peer_ltk_get(struct ble_peer_s *peer, uint8_t *ltk);
/**
   @this retrieves peer-distributed CSRK
 */
error_t ble_peer_csrk_get(struct ble_peer_s *peer, uint8_t *csrk);
/**
   @this retrieves peer-distributed identification
 */
error_t ble_peer_id_get(struct ble_peer_s *peer, uint8_t *random, uint16_t *ediv);
/**
   @this sets peer-distributed IRK
 */
void ble_peer_irk_set(struct ble_peer_s *peer, const uint8_t *irk);

/**
   @this saves peer in persistent storage.
 */
error_t ble_peer_save(struct ble_peer_s *peer);

/**
   @this sets peer-distributed LTK
 */
void ble_peer_ltk_set(struct ble_peer_s *peer, const uint8_t *ltk);

/**
   @this sets peer-distributed identity
 */
void ble_peer_identity_set(struct ble_peer_s *peer, uint16_t ediv, const uint8_t *rand);

/**
   @this retrieves a session key from peer when we are the master side
   of connection.
 */
error_t ble_peer_master_sk_get(struct ble_peer_s *peer,
                               const uint8_t skd[static 16],
                               uint16_t ediv[static 1], uint8_t rand[static 8],
                               uint8_t sk[static 16]);

#endif

/**
   @this retrieves the subscription list from security DB.
 */
void ble_peer_subscriptions_set(struct ble_peer_s *peer,
                                struct ble_subscription_s subscriptions[static BLE_SUBSCRIBED_CHAR_COUNT]);

#endif
