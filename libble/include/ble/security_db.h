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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
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
#include <device/class/crypto.h>
#include <device/class/persist.h>

struct dev_rng_s;

struct ble_peer_s
{
  uint8_t stk[16];
  uint8_t irk[16];
  uint64_t id;

  struct ble_security_db_s *db;

  struct ble_addr_s lookup_addr;
  struct ble_addr_s addr;

  bool_t addr_present : 1;
  bool_t irk_present : 1;
  bool_t stk_present : 1;
  bool_t paired : 1;
  bool_t bonded : 1;
  bool_t mitm_protection : 1;
  bool_t secure_pairing : 1;
};

struct ble_security_db_s
{
  struct device_persist_s persist;
  struct device_crypto_s aes;
  struct dev_rng_s *rng;

  uint8_t pk[16];
  uint8_t irk[16];

  uint64_t paired_id[CONFIG_BLE_SECURITY_DB_MAX];
};

/**
   @this initializes a peer database lookup context.
 */
error_t ble_security_db_init(struct ble_security_db_s *security_db,
                             const char *persist,
                             const char *aes,
                             struct dev_rng_s *rng);

void ble_security_db_cleanup(struct ble_security_db_s *security_db);

/**
   @this looks up a peer entry from an address, either random (using
   IRK) or public.
 */
error_t ble_security_db_lookup(struct ble_security_db_s *security_db,
                               const struct ble_addr_s *addr,
                               struct ble_peer_s *peer);

/**
   Add a security_db entry.
 */
error_t ble_security_db_save(struct ble_security_db_s *security_db,
                        const struct ble_peer_s *peer);

/**
   Erase a security_db entry.
 */
error_t ble_security_db_remove(struct ble_security_db_s *security_db,
                               uint64_t id);

/**
   Erase all security_db entries.
 */
void ble_security_db_clear(struct ble_security_db_s *security_db);

void ble_peer_init(struct ble_peer_s *peer,
                   struct ble_security_db_s *db,
                   const struct ble_addr_s *addr);

error_t ble_peer_reset(struct ble_peer_s *peer);

/**
   Retrieve Session Key from entry.
 */
error_t ble_peer_sk_get(struct ble_peer_s *peer,
                        const uint8_t *skd,
                        const uint8_t *rand,
                        const uint16_t ediv,
                        uint8_t *sk);

error_t ble_peer_ltk_get(struct ble_peer_s *peer, uint8_t *ltk);
error_t ble_peer_id_get(struct ble_peer_s *peer, uint8_t *random, uint16_t *ediv);

void ble_peer_irk_set(struct ble_peer_s *peer, const uint8_t *irk);

void ble_peer_addr_set(struct ble_peer_s *peer, const struct ble_addr_s *addr);

error_t ble_peer_paired(struct ble_peer_s *peer,
                        bool_t bonded, bool_t mitm_protection, bool_t secure_pairing,
                        const uint8_t *stk);

#endif
