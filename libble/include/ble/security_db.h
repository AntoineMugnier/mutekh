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
   @module {Bluetooth Low Energy library}
   @short Security Database

   This header contains definitions for security database.
*/

#include <hexo/types.h>
#include <ble/protocol/address.h>
#if defined(CONFIG_BLE_SECURITY_DB)
# include <persist/persist.h>
#endif
#if defined(CONFIG_BLE_CRYPTO)
# include <device/class/crypto.h>
#endif

struct persist_context_s;
struct dev_rng_s;
struct ble_peer_s;

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
   #CONFIG_BLE_SECURITY_DB is disabled) or no cryptographic operations
   at all (if @ref #CONFIG_BLE_CRYPTO is disabled).
*/
struct ble_security_db_s
{
#if defined(CONFIG_BLE_CRYPTO)
  struct device_crypto_s aes;
  struct dev_rng_s *rng;
#endif
#if defined(CONFIG_BLE_SECURITY_DB)
  struct persist_context_s *persist;

  uint8_t pk[16];
  uint8_t irk[16];

  uint64_t paired_id[CONFIG_BLE_SECURITY_DB_MAX];
#endif
};

/**
   @this initializes a peer database lookup context.

   @param persist Path to a @ref DRIVER_CLASS_PERSIST device, can be
   NULL if Security DB is @ref {#CONFIG_BLE_SECURITY_DB} {disabled}
 */
error_t ble_security_db_init(struct ble_security_db_s *security_db,
                             struct persist_context_s *persist,
                             const char *aes,
                             struct dev_rng_s *rng);

/**
   @this releases all context data of cryptography DB.
 */
void ble_security_db_cleanup(struct ble_security_db_s *security_db);

#if defined(CONFIG_BLE_SECURITY_DB)
/**
   @this looks up a peer entry by address, either random (using IRK)
   or public.
 */
error_t ble_security_db_load(struct ble_security_db_s *security_db,
                               const struct ble_addr_s *addr,
                               struct ble_peer_s *peer);

error_t ble_security_db_peer_reconnect_addr_set(struct ble_security_db_s *security_db,
                                                uint8_t slot, struct ble_addr_s *addr);

/**
   @this tells whether passed address is known by security database.
 */
bool_t ble_security_db_contains(struct ble_security_db_s *security_db,
                                const struct ble_addr_s *addr);

/**
   @this tells how many entries are populated in the security
   database.  This cannot be more than @ref
   #CONFIG_BLE_SECURITY_DB_MAX.
 */
uint_fast8_t ble_security_db_count(struct ble_security_db_s *security_db);

/**
   @this erases a security_db entry by its number.
 */
error_t ble_security_db_remove(struct ble_security_db_s *security_db,
                               uint64_t id);

/**
   @this erases all security_db entries.
 */
void ble_security_db_clear(struct ble_security_db_s *security_db);

#else

ALWAYS_INLINE
uint_fast8_t ble_security_db_count(struct ble_security_db_s *security_db)
{
  return 0;
}

#endif

#endif
