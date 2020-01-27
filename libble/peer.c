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

#define LOGK_MODULE_ID "bpdb"

#include <hexo/types.h>
#include <ble/protocol/address.h>

#include <mutek/printk.h>

#if defined(CONFIG_BLE_SECURITY_DB)
# include <persist/persist.h>
# include <device/class/crypto.h>
#endif

#if defined(CONFIG_BLE_CRYPTO)
# include <ble/crypto.h>
#endif

#include <ble/security_db.h>
#include <ble/peer.h>

#include <string.h>
#include "security_db_priv.h"

#if defined(CONFIG_BLE_SECURITY_DB)
static
error_t ble_peer_subscriptions_save(const struct ble_peer_s *peer)
{
  struct ble_security_db_s *db = peer->db;
  uint8_t index;

  if (!db)
    return -EINVAL;

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (db->paired_id[i] == peer->id) {
      index = i;

      goto write;
    }
  }

  return -ENOENT;

 write:

  return peer_subscribed_save(db, peer, index);
}
#endif

#if defined(CONFIG_BLE_CRYPTO)
error_t ble_peer_sk_get(struct ble_peer_s *peer,
                        const uint8_t *skd,
                        const uint8_t *random,
                        const uint16_t ediv,
                        uint8_t *sk)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  struct ble_security_db_s *db = peer->db;
  if (!db)
    return -EINVAL;

  uint8_t tmp[16];
  uint64_t did = endian_le64_na_load(random);
#endif
  uint8_t pk[16], skd_rev[16];
  error_t err;

  logk_trace("%s peer %lld, random %lld, ediv %04x", __FUNCTION__,
         peer->id, did, ediv);

#if defined(CONFIG_BLE_SECURITY_DB)
  if (ediv || did) {
    // LTK mode
    logk_trace("  LTK lookup mode");

    if (did != peer->id) {
      logk_trace("  peer %lld is not loaded", did);
      err = ble_peer_lookup_id(peer->db, peer, random, ediv);
      logk_trace("  peer %lld lookup: %d", did, err);
      if (err)
        return err;
    }

    err = ble_security_db_key_get(db, did, KEY_HANDLE_LTK, tmp);
    if (err)
      return err;

    logk_debug("LTK:        %P", tmp, 16);

    memrevcpy(pk, tmp, 16);
  } else
#endif
  {
    // STK mode when just paired

    logk_debug("  STK lookup mode, STK: %d", peer->stk_present);

    if (!peer->stk_present)
      return -ENOENT;

    logk_debug("STK:        %P", peer->stk, 16);

    memrevcpy(pk, peer->stk, 16);

    peer->stk_present = 0;
  }

  logk_trace("SKD:        %P", skd, 16);

  memrevcpy(skd_rev, skd, 16);

  err = ble_e(&peer->db->aes, pk, skd_rev, sk);

  logk_trace("SK:         %P", sk, 16);

  return err;
}
#endif

void ble_peer_init(struct ble_peer_s *peer,
                   struct ble_security_db_s *db,
                   const struct ble_addr_s *addr)
{
  memset(peer, 0, sizeof(*peer));

#if defined(CONFIG_BLE_CRYPTO)
  peer->db = db;
#endif
  peer->lookup_addr = *addr;
  peer->dirty = 0;

  memset(peer->subscriptions, 0xff, sizeof(struct ble_subscription_s) * BLE_SUBSCRIBED_CHAR_COUNT);
  peer->subscriptions_dirty = 0;
}

error_t ble_peer_reset(struct ble_peer_s *peer)
{
  struct ble_addr_s tmp;
  struct ble_security_db_s *db = NULL;

#if defined(CONFIG_BLE_CRYPTO)
  db = peer->db;

  if (!db)
    return -EINVAL;
#endif

  tmp = peer->lookup_addr;

  enum ble_addr_random_type_e rt = ble_addr_random_type(&tmp);

  ble_peer_init(peer, db, &tmp);

  if (tmp.type == BLE_ADDR_PUBLIC || rt == BLE_ADDR_RANDOM_STATIC) {
    peer->addr = tmp;
    peer->addr_present = 1;
  }

#if defined(CONFIG_BLE_SECURITY_DB)
  return ble_security_db_next_id(peer->db, &peer->id);
#else
  return 0;
#endif
}

void ble_peer_addr_set(struct ble_peer_s *peer, const struct ble_addr_s *addr)
{
  logk_debug("%s peer id %lld", __FUNCTION__, peer->id);

#if defined(CONFIG_BLE_CRYPTO)
  if (!peer->db)
    return;
#endif

  if (peer->addr_present && !ble_addr_cmp(addr, &peer->addr))
    return;

  peer->addr = *addr;
  peer->addr_present = 1;

  peer->dirty = 1;
}

#if defined(CONFIG_BLE_SECURITY_DB)
error_t ble_peer_ltk_get(struct ble_peer_s *peer, uint8_t *ltk)
{
    return ble_security_db_key_get(peer->db, peer->id, KEY_HANDLE_LTK, ltk);
}

error_t ble_peer_csrk_get(struct ble_peer_s *peer, uint8_t *csrk)
{
    return ble_security_db_key_get(peer->db, peer->id, KEY_HANDLE_CSRK, csrk);
}

error_t ble_peer_id_get(struct ble_peer_s *peer, uint8_t *random, uint16_t *ediv)
{
  uint8_t tmp[16];
  error_t err;

  err = ble_security_db_key_get(peer->db, peer->id, KEY_HANDLE_DIV, tmp);
  if (err)
    return err;

  *ediv = endian_le16_na_load(tmp);
  endian_le64_na_store(random, peer->id);

  return 0;
}

void ble_peer_irk_set(struct ble_peer_s *peer, const uint8_t *irk)
{
  logk_debug("%s peer id %lld", __FUNCTION__, peer->id);

  if (!peer->db)
    return;

  if (peer->irk_present && !memcmp(irk, peer->irk, 16))
    return;

  memcpy(peer->irk, irk, 16);
  peer->irk_present = 1;

  peer->dirty = 1;
}

#endif

#if defined(CONFIG_BLE_CRYPTO)
error_t ble_peer_paired(struct ble_peer_s *peer,
                        bool_t bonded,
                        bool_t mitm_protection,
                        bool_t secure_pairing,
                        const uint8_t *stk)
{
  if (!peer->db && bonded)
    return -ENOTSUP;

  logk_debug("%s peer id %lld", __FUNCTION__, peer->id);

#if defined(CONFIG_BLE_SECURITY_DB)
  peer->bonded = bonded;
  peer->mitm_protection = mitm_protection;
  peer->secure_pairing = secure_pairing;
  peer->paired = 1;
#endif

  peer->stk_present = !!stk;
  if (stk)
    memcpy(peer->stk, stk, 16);

  peer->dirty = 1;

  return 0;
}

error_t ble_peer_save(struct ble_peer_s *peer)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  struct ble_security_db_s *db = peer->db;
  error_t err;

  logk_debug("Saving peer %lld", peer->id);

  if (!db) {
    logk_error("No DB for peer");
    return -EINVAL;
  }

  if (!peer->paired && !peer->bonded && !peer->addr_present && !peer->irk_present) {
    logk_error("Not saving useless peer %lld", peer->id);
    return -EINVAL;
  }

  if (peer->dirty) {
    err = ble_security_db_save(db, peer);
    if (err)
      return err;

    peer->dirty = 0;
  }

  if (peer->subscriptions_dirty) {
    err = ble_peer_subscriptions_save(peer);
    if (err)
      return err;

    peer->subscriptions_dirty = 0;
  }

  return 0;
#else
  return -ENOTSUP;
#endif
}

void ble_peer_ltk_set(struct ble_peer_s *peer, const uint8_t *ltk)
{
  memcpy(peer->ltk, ltk, 16);
  peer->ltk_present = 1;
  peer->dirty = 1;
}

void ble_peer_identity_set(struct ble_peer_s *peer, uint16_t ediv, const uint8_t *rand)
{
  memcpy(peer->rand, rand, 8);
  peer->ediv = ediv;
  peer->identity_present = 1;
  peer->dirty = 1;
}

error_t ble_peer_master_sk_get(struct ble_peer_s *peer,
                               const uint8_t skd[static 16],
                               uint16_t ediv[static 1], uint8_t rand[static 8],
                               uint8_t sk[static 16])
{
  uint8_t pk[16];
  uint8_t skd_rev[16];

  if (peer->stk_present) {
    memrevcpy(pk, peer->stk, 16);
    memset(rand, 0, 8);
    *ediv = 0;

    logk_debug("STK:        %P", peer->stk, 16);

    peer->stk_present = 0;
  } else if (peer->identity_present && peer->ltk_present) {
    memrevcpy(pk, peer->ltk, 16);
    memcpy(rand, peer->rand, 8);
    *ediv = peer->ediv;
    logk_debug("LTK:        %P", peer->ltk, 16);
  } else {
    return -ENOENT;
  }

  logk_trace("SKD:        %P", skd, 16);

  memrevcpy(skd_rev, skd, 16);

  error_t err = ble_e(&peer->db->aes, pk, skd_rev, sk);

  logk_trace("SK:         %P", pk, 16);

  return err;
}

#endif

void ble_peer_subscriptions_set(struct ble_peer_s *peer,
                                struct ble_subscription_s subscriptions[static BLE_SUBSCRIBED_CHAR_COUNT])
{
  if (!memcmp(subscriptions, peer->subscriptions,
              sizeof(struct ble_subscription_s) * BLE_SUBSCRIBED_CHAR_COUNT))
    return;

  peer->subscriptions_dirty = 1;
  memcpy(peer->subscriptions, subscriptions,
         sizeof(struct ble_subscription_s) * BLE_SUBSCRIBED_CHAR_COUNT);
}
