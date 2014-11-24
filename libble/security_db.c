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

#include <hexo/types.h>
#include <ble/protocol/address.h>

#include <mutek/printk.h>

#include <device/class/persist.h>
#include <device/class/crypto.h>
#include <ble/crypto.h>

#include <ble/security_db.h>

#include <string.h>

enum key_handle_e
{
  KEY_HANDLE_IRK,
  KEY_HANDLE_LTK,
  KEY_HANDLE_DIV,
};

__attribute__((packed))
struct ble_security_db_entry_s
{
  uint8_t irk[16];
  uint8_t ltk[16];

  uint64_t id;

  uint8_t addr[6];

  bool_t addr_present : 1;
  bool_t irk_present : 1;
  bool_t addr_random : 1;
  bool_t bonded : 1;
  bool_t mitm_protection : 1;
  bool_t secure_pairing : 1;
};

static const struct dev_persist_descriptor_s security_db_entry_blob = {
  .uid = 0x3200,
  .type = DEV_PERSIST_BLOB,
  .size = sizeof(struct ble_security_db_entry_s),
};

static const struct dev_persist_descriptor_s security_db_device_counter = {
  .uid = 0x3301,
  .type = DEV_PERSIST_COUNTER,
  .size = 16,
};

static const struct dev_persist_descriptor_s security_db_pk_blob = {
  .uid = 0x3300,
  .type = DEV_PERSIST_BLOB,
  .size = 16,
};

error_t ble_security_db_key_get(struct ble_security_db_s *db,
                                uint64_t did,
                                uint64_t key_handle,
                                uint8_t *out)
{
  uint64_t tmp[2] = { did, key_handle };

  return ble_e(&db->aes, db->pk, (const uint8_t *)tmp, out);
}

static void peer_entry_addr_load(const struct ble_security_db_entry_s *entry,
                                 struct ble_addr_s *addr)
{
  memcpy(addr->addr, entry->addr, 6);

  addr->type = entry->addr_random ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
}

static void peer_entry_store(struct ble_security_db_entry_s *entry,
                             const struct ble_peer_s *peer)
{
  memcpy(entry->irk, peer->irk, 16);
  memcpy(entry->addr, peer->addr.addr, 6);

  entry->addr_random = peer->addr.type == BLE_ADDR_RANDOM;
  entry->addr_present = peer->addr_present;
  entry->irk_present = peer->irk_present;

  entry->id = peer->id;
  entry->bonded = peer->bonded;
  entry->mitm_protection = peer->mitm_protection;
  entry->secure_pairing = peer->secure_pairing;
}

static void peer_entry_load(const struct ble_security_db_entry_s *entry,
                            struct ble_peer_s *peer)
{
  memcpy(peer->irk, entry->irk, 16);
  peer_entry_addr_load(entry, &peer->addr);

  peer->id = entry->id;
  peer->addr_present = entry->addr_present;
  peer->irk_present = entry->irk_present;
  peer->bonded = entry->bonded;
  peer->mitm_protection = entry->mitm_protection;
  peer->secure_pairing = entry->secure_pairing;
  peer->paired = 1;
}

error_t ble_security_db_init(struct ble_security_db_s *db,
                             const char *persist,
                             const char *aes,
                             struct dev_rng_s *rng)
{
  error_t err;
  const void *tmp;

  memset(db, 0, sizeof(*db));

  err = device_get_accessor_by_path(&db->persist, NULL, persist, DRIVER_CLASS_PERSIST);
  if (err)
    return err;

  err = device_get_accessor_by_path(&db->aes, NULL, aes, DRIVER_CLASS_CRYPTO);
  if (err)
    goto put_persist;

  db->rng = rng;

  err = dev_persist_wait_read(&db->persist, &security_db_pk_blob, 0, &tmp);
  if (!err) {
    memcpy(db->pk, tmp, 16);
  } else {
    err = dev_rng_wait_read(db->rng, db->pk, 16);
    if (err)
      goto put_aes;

    dev_persist_wait_write(&db->persist, &security_db_pk_blob, 0, db->pk);
    // We can still run until reboot if key fails to write...
  }

  err = dev_persist_wait_read(&db->persist, &security_db_device_counter, 0, &tmp);
  if (err) {
    uint64_t init = 1;

    dev_persist_wait_write(&db->persist, &security_db_pk_blob, 0, &init);
  }

  err = dev_persist_wait_read(&db->persist, &security_db_device_counter, 0, &tmp);
  if (err) {
    uint64_t init = 1;

    dev_persist_wait_write(&db->persist, &security_db_pk_blob, 0, &init);
  }

  err = ble_security_db_key_get(db, 0, KEY_HANDLE_IRK, db->irk);
  if (err)
    goto put_aes;

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    const struct ble_security_db_entry_s *entry;
    err = dev_persist_wait_read(&db->persist, &security_db_entry_blob, i, (const void**)&entry);

    if (!err)
      db->paired_id[i] = entry->id;
    else
      db->paired_id[i] = 0;
  }

  return 0;

 put_aes:
  device_put_accessor(&db->aes);
 put_persist:
  device_put_accessor(&db->persist);
  return err;
}

void ble_security_db_cleanup(struct ble_security_db_s *db)
{
  device_put_accessor(&db->persist);
  device_put_accessor(&db->aes);
}

error_t ble_security_db_lookup(struct ble_security_db_s *db,
                               const struct ble_addr_s *addr,
                               struct ble_peer_s *peer)
{
  enum ble_addr_random_type_e rt = ble_addr_random_type(addr);
  error_t err;

  memset(peer, 0, sizeof(*peer));

  peer->lookup_addr = *addr;

  if (addr->type == BLE_ADDR_PUBLIC || rt == BLE_ADDR_RANDOM_STATIC) {
    peer->addr = *addr;
    peer->addr_present = 1;
  }

  if (addr->type != BLE_ADDR_PUBLIC
      && rt != BLE_ADDR_RANDOM_STATIC
      && rt != BLE_ADDR_RANDOM_RESOLVABLE)
    return -ENOTSUP;

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    const struct ble_security_db_entry_s *entry;

    if (db->paired_id[i] == 0)
      continue;

    err = dev_persist_wait_read(&db->persist, &security_db_entry_blob, i, (const void**)&entry);

    if (err) {
      // This should not happen
      db->paired_id[i] = 0;
      continue;
    }

    if (addr->type == BLE_ADDR_PUBLIC || rt == BLE_ADDR_RANDOM_STATIC) {
      struct ble_addr_s tmp;

      peer_entry_addr_load(entry, &tmp);

      if (ble_addr_cmp(addr, &tmp))
        continue;
    } else {
      uint8_t tmp[3];

      if (!entry->irk_present)
        continue;

      ble_ah(&db->aes, entry->irk, addr->addr, tmp);
      if (memcmp(tmp, &addr->addr[3], 3))
        continue;
    }

    peer_entry_load(entry, peer);
    return 0;
  }

  return -ENOENT;
}

/**
   Add a security_db entry.

   Select oldest entry to overwrite.
 */
error_t ble_security_db_save(struct ble_security_db_s *db,
                             const struct ble_peer_s *peer)
{
  uint64_t oldest_id = -1;
  int8_t index = -1;
  struct ble_security_db_entry_s entry;
  error_t err;
  struct ble_peer_s existing;

  if (peer->id == 0)
    return -EINVAL;

  peer_entry_store(&entry, peer);

  // Check collision
  err = ble_security_db_lookup(db, &peer->addr, &existing);
  if (err) {
    // Entry with id == 0 can always be overwritten
    existing.id = 0;
  }

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (db->paired_id[i] == existing.id) {
      index = i;

      goto write;
    }

    if (db->paired_id[i] < oldest_id) {
      oldest_id = db->paired_id[i];
      index = i;
    }
  }

 write:
  db->paired_id[index] = peer->id;

  printk("SecDB: Saving peer %lld\n", peer->id);

  return dev_persist_wait_write(&db->persist, &security_db_entry_blob, index, &entry);
}

/**
   Erase a security_db entry.
 */
error_t ble_security_db_remove(struct ble_security_db_s *db,
                               uint64_t id)
{
  uint8_t index = 0;

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (db->paired_id[i] == id) {
      index = i;

      goto remove;
    }
  }

  return -ENOENT;

 remove:
  db->paired_id[index] = 0;
  return dev_persist_wait_remove(&db->persist, &security_db_entry_blob, index);
}

/**
   Erase all security_db entries.
 */
void ble_security_db_clear(struct ble_security_db_s *db)
{
  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (!db->paired_id[i])
      continue;

    db->paired_id[i] = 0;
    dev_persist_wait_remove(&db->persist, &security_db_entry_blob, i);
  }
}

static
error_t ble_peer_lookup_id(struct ble_security_db_s *db,
                           struct ble_peer_s *peer,
                           const uint8_t *random,
                           const uint16_t ediv)
{
  uint64_t did = endian_le64_na_load(random);
  error_t err;
  uint8_t tmp[16];

  if (!peer && !random)
    return -ENOENT;

  err = ble_security_db_key_get(db, did, KEY_HANDLE_DIV, tmp);
  if (err)
    return err;

  // Check EDIV
  if (ediv != endian_le16_na_load(tmp))
    return -EINVAL;

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    const struct ble_security_db_entry_s *entry;

    if (db->paired_id[i] != did)
      continue;

    err = dev_persist_wait_read(&db->persist, &security_db_entry_blob, i, (const void**)&entry);

    if (err) {
      // This should not happen
      db->paired_id[i] = 0;
      continue;
    }

    peer_entry_load(entry, peer);
    return 0;
  }

  return -ENOENT;
}

error_t ble_peer_sk_get(struct ble_peer_s *peer,
                        const uint8_t *skd,
                        const uint8_t *random,
                        const uint16_t ediv,
                        uint8_t *sk)
{
  if (!peer->db)
    return -EINVAL;

  uint8_t tmp[16], pk[16], skd_rev[16];
  uint64_t did = endian_le64_na_load(random);
  error_t err;
  struct ble_security_db_s *db = peer->db;

  printk("%s peer %lld, random %lld, ediv %d\n", __FUNCTION__,
         peer->id, did, 8, ediv);

  if (ediv || did) {
    // LTK mode
    printk("  LTK lookup mode\n");

    if (did != peer->id) {
      printk("  peer %lld is not loaded\n", did);
      err = ble_peer_lookup_id(peer->db, peer, random, ediv);
      printk("  peer %lld lookup: %d\n", did, err);
      if (err)
        return err;
    }

    err = ble_security_db_key_get(db, did, KEY_HANDLE_LTK, tmp);
    if (err)
      return err;

    printk("LTK:        %P\n", tmp, 16);

    memrevcpy(pk, tmp, 16);
  } else {
    // STK mode when just paired

    printk("  STK lookup mode, STK: %d\n", peer->stk_present);

    if (!peer->stk_present)
      return -ENOENT;

    printk("STK:        %P\n", peer->stk, 16);

    memrevcpy(pk, peer->stk, 16);
  }

  printk("SKD:        %P\n", skd, 16);

  memrevcpy(skd_rev, skd, 16);

  err = ble_e(&db->aes, pk, skd_rev, sk);

  printk("SK:         %P\n", sk, 16);

  return err;
}

void ble_peer_init(struct ble_peer_s *peer,
                   struct ble_security_db_s *db,
                   const struct ble_addr_s *addr)
{
  memset(peer, 0, sizeof(*peer));

  peer->db = db;
  peer->lookup_addr = *addr;
}

error_t ble_peer_reset(struct ble_peer_s *peer)
{
  if (!peer->db)
    return -EINVAL;

  struct ble_addr_s tmp;
  error_t err;
  struct ble_security_db_s *db = peer->db;

  tmp = peer->lookup_addr;
  memset(peer, 0, sizeof(*peer));
  peer->lookup_addr = tmp;

  peer->db = db;

  err = dev_persist_wait_inc(&peer->db->persist, &security_db_device_counter, 0);
  if (err)
    return err;

  return dev_persist_wait_counter_read(&peer->db->persist, &security_db_device_counter,
                                       0, &peer->id);
}

error_t ble_peer_ltk_get(struct ble_peer_s *peer, uint8_t *ltk)
{
    return ble_security_db_key_get(peer->db, peer->id, KEY_HANDLE_LTK, ltk);
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
  memcpy(peer->irk, irk, 16);
  peer->irk_present = 1;
}

void ble_peer_addr_set(struct ble_peer_s *peer, const struct ble_addr_s *addr)
{
  peer->addr = *addr;
  peer->addr_present = 1;
}

error_t ble_peer_paired(struct ble_peer_s *peer,
                        bool_t bonded,
                        bool_t mitm_protection,
                        bool_t secure_pairing,
                        const uint8_t *stk)
{
  if (!peer->db)
    return -EINVAL;

  struct ble_security_db_s *db = peer->db;

  peer->bonded = bonded;
  peer->mitm_protection = mitm_protection;
  peer->secure_pairing = secure_pairing;
  peer->paired = 1;

  peer->stk_present = !!stk;
  if (stk)
    memcpy(peer->stk, stk, 16);

  return ble_security_db_save(db, peer);
}
