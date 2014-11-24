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
   @module{BLE library}
   @short Peer database

   @section {Description}

   This is a BLE host database containing pairing data.

   @end section
*/

#include <hexo/types.h>
#include <ble/protocol/address.h>
#if defined(CONFIG_BLE_SECURITY_DB)
# include <device/class/persist.h>
#endif
#if defined(CONFIG_BLE_CRYPTO)
# include <device/class/crypto.h>
#endif

struct dev_rng_s;
struct ble_peer_s;

struct ble_security_db_s
{
#if defined(CONFIG_BLE_CRYPTO)
  struct device_crypto_s aes;
  struct dev_rng_s *rng;
#endif
#if defined(CONFIG_BLE_SECURITY_DB)
  struct device_persist_s persist;

  uint8_t pk[16];
  uint8_t irk[16];

  uint64_t paired_id[CONFIG_BLE_SECURITY_DB_MAX];
#endif
};

/**
   @this initializes a peer database lookup context.
 */
error_t ble_security_db_init(struct ble_security_db_s *security_db,
                             const char *persist,
                             const char *aes,
                             struct dev_rng_s *rng);

void ble_security_db_cleanup(struct ble_security_db_s *security_db);

#if defined(CONFIG_BLE_SECURITY_DB)
/**
   @this looks up a peer entry from an address, either random (using
   IRK) or public.
 */
error_t ble_security_db_lookup(struct ble_security_db_s *security_db,
                               const struct ble_addr_s *addr,
                               struct ble_peer_s *peer);

error_t ble_security_db_peer_reconnect_addr_set(struct ble_security_db_s *security_db,
                                                uint8_t slot, struct ble_addr_s *addr);

bool_t ble_security_db_contains(struct ble_security_db_s *security_db,
                                const struct ble_addr_s *addr);

uint_fast8_t ble_security_db_count(struct ble_security_db_s *security_db);

/**
   Erase a security_db entry.
 */
error_t ble_security_db_remove(struct ble_security_db_s *security_db,
                               uint64_t id);

/**
   Erase all security_db entries.
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
