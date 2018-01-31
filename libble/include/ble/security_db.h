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

#ifndef BLE_SECURITY_DB_H_
#define BLE_SECURITY_DB_H_

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short Security Database

   This header contains definitions for security database.
*/

#include <hexo/types.h>
#include <ble/protocol/address.h>
#if defined(CONFIG_DEVICE_PERSIST)
# include <device/class/persist.h>
#endif
#include <device/class/crypto.h>

struct dev_rng_s;
struct ble_peer_s;

#define BLE_SUBSCRIBED_CHAR_COUNT 8

/**
   @internal
   Per-device lookup data.
 */
struct ble_security_lookup_data_s
{
  struct ble_addr_s addr;
  uint32_t id;
  uint8_t irk[16];
};

struct ble_peer_data_s
{
  /** LTK, AES input order */
  uint8_t ltk[16];
  /** IRK, AES input order */
  uint8_t irk[16];
  /** CSRK, AES input order */
  uint8_t csrk[16];

  /** Randomizer as an integer */
  uint64_t rand;

  /** EDIV as an integer */
  uint16_t ediv;

  /** Whether IRK is valid */
  bool_t irk_present : 1;
  /** Whether CSRK is valid */
  bool_t csrk_present : 1;
  /** Whether peer @tt rand and @tt ediv are present */
  bool_t identity_present : 1;
  /** Whether LTK is valid */
  bool_t ltk_present : 1;
};

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
  /** Peer serial ID, allocated on pairing */
  uint32_t id;

  /** Peer lookup address, this may contain a resolvable address or
      public device address. */
  struct ble_addr_s lookup_addr;

  /** Peer public address */
  struct ble_addr_s addr;

  struct ble_peer_data_s remote;
  struct ble_peer_data_s local;

  /** STK, AES input order, not persistant */
  uint8_t stk[16];

  /** Whether we currently hold a valid STK */
  bool_t stk_present : 1;

  /** Whether we bonded */
  bool_t bonded : 1;

  /** Whether we bonded with MITM protection */
  bool_t mitm_protection : 1;

  /** Whether pairing was secure */
  bool_t secure_pairing : 1;

  /** Whether peer address is valid */
  bool_t addr_present : 1;

  /** Whether some data changed and should be flushed to non-volatile
      storage */
  bool_t dirty : 1;
};

/**
   @this is a BLE Security database containing data associated to
   peers.  Security DB contains:
   @list
   @item local device private key and IRK,
   @item per-peer cryptography parameters negociated during pairing,
   @item per-peer GATT DB subscriptions.
   @end list

   Security DB object always exists, but may, depending on
   configuration parameters, only handle a volatile context (if @ref
   #CONFIG_DEVICE_PERSIST is disabled) or no cryptographic operations
   at all (if @ref #CONFIG_BLE_CRYPTO is disabled).
*/
struct ble_security_db_s
{
  struct device_crypto_s aes;
  struct dev_rng_s *rng;
#if defined(CONFIG_DEVICE_PERSIST)
  struct device_persist_s persist;
#endif

  uint8_t pk[16];
  uint8_t irk[16];
  uint32_t next_id;

  struct ble_security_lookup_data_s peer[CONFIG_BLE_SECURITY_DB_MAX];
};

enum ble_security_db_rq_type_e
{
  /** Parameters: None */
  BLE_SECURITY_LOAD,
  /** Parameters: None */
  BLE_SECURITY_CLEAR,
  /** Parameters: id, need a valid peer pointer */
  BLE_SECURITY_PEER_LOAD,
  /** Parameters: peer, sets id */
  BLE_SECURITY_PEER_SAVE,
  /** Parameters: addr, sets id, only useful in address resolving
      context */
  BLE_SECURITY_PEER_LOOKUP,
  /** Parameters: id, subscriptions pointer */
  BLE_SECURITY_SUBSCRIPTIONS_LOAD,
  /** Parameters: id, subscriptions */
  BLE_SECURITY_SUBSCRIPTIONS_SAVE,
};

struct ble_security_db_rq_s
{
  struct dev_request_s base;

  enum ble_security_db_rq_type_e type;

  uint32_t id;
  union {
    struct ble_peer_s *peer;
    struct ble_addr_s addr;
    /** Exactly BLE_SUBSCRIBED_CHAR_COUNT entries */
    struct ble_subscription_s *subscriptions;
  };
};

STRUCT_COMPOSE(ble_security_db_rq_s, base);

/**
   @this initializes a peer database lookup context.

   @param persist Path to a @ref DRIVER_CLASS_PERSIST device, can be
   NULL if Security DB is @ref {#CONFIG_DEVICE_PERSIST} {disabled}
 */
error_t ble_security_db_init(struct ble_security_db_s *security_db,
                             const char *persist,
                             const char *aes,
                             struct dev_rng_s *rng);

/**
   @this releases all context data of cryptography DB.
 */
void ble_security_db_cleanup(struct ble_security_db_s *security_db);

/**
   @this loads data from storage backend
 */
void ble_security_request(struct ble_security_db_s *security_db,
                          struct ble_security_db_rq_s *rq);


#ifdef CONFIG_DEVICE_PERSIST
/**
   @this tells how many entries are populated in the security
   database.  This cannot be more than @ref
   #CONFIG_BLE_SECURITY_DB_MAX.
 */
uint_fast8_t ble_security_db_count(struct ble_security_db_s *security_db);
#else
ALWAYS_INLINE
uint_fast8_t ble_security_db_count(struct ble_security_db_s *security_db)
{
  return 0;
}
#endif

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

#if defined(CONFIG_BLE_CRYPTO)

/**
   @this marks peer as paired.
 */
void ble_peer_phase2_done(struct ble_peer_s *peer,
                          const uint8_t stk[static 16],
                          bool_t mitm_protection);

/**
   @this sets peer-distributed IRK
 */
void ble_peer_irk_set(struct ble_peer_s *peer, const uint8_t irk[static 16]);

/**
   @this sets peer-distributed LTK
 */
void ble_peer_ltk_set(struct ble_peer_s *peer, const uint8_t ltk[static 16]);

/**
   @this sets peer-distributed identity
 */
void ble_peer_identity_set(struct ble_peer_s *peer, uint64_t rand, uint16_t ediv);

/**
   @this sets peer's address
 */
void ble_peer_address_set(struct ble_peer_s *peer, const struct ble_addr_s *addr);

/**
   @this retrieves Session Key from entry, either derived from LTK or
   STK, depending if present in peer.
 */
error_t ble_peer_sk_get(struct ble_peer_s *peer,
                        bool_t is_slave,
                        const uint8_t skd[static 16],
                        uint64_t rand, uint16_t ediv,
                        uint8_t sk[static 16]);

#endif
