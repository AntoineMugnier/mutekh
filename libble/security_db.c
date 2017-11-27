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

#define LOGK_MODULE_ID "bsec"

#include <hexo/types.h>
#include <ble/protocol/address.h>

#include <mutek/printk.h>

#include <device/class/crypto.h>
#include <ble/crypto.h>

#include <ble/security_db.h>

#if defined(CONFIG_DEVICE_PERSIST)
# include <device/class/persist.h>
#endif

#include <string.h>

#if defined(CONFIG_DEVICE_PERSIST)
__attribute__((packed))
struct ble_security_db_entry_s
{
  uint8_t irk[16];
  uint8_t ltk[16];
  uint8_t rand[8];
  uint8_t addr[6];
  uint16_t ediv;
  uint32_t id;

  bool_t addr_present : 1;
  bool_t irk_present : 1;
  bool_t ltk_present : 1;
  bool_t identity_present : 1;
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
#endif

static
void peer_entry_store(struct ble_security_db_entry_s *entry,
                      const struct ble_peer_s *peer)
{
  memcpy(entry->ltk, peer->remote.ltk, 16);
  memcpy(entry->irk, peer->remote.irk, 16);
  memcpy(entry->csrk, peer->remote.csrk, 16);
  memcpy(entry->addr, peer->addr.addr, 6);
  entry->rand = peer->remote.rand;
  entry->ediv = peer->remote.ediv;

  entry->addr_random = peer->addr.type == BLE_ADDR_RANDOM;
  entry->addr_present = peer->addr_present;
  entry->ltk_present = peer->remote.ltk_present;
  entry->irk_present = peer->remote.irk_present;
  entry->csrk_present = peer->remote.csrk_present;
  entry->identity_present = peer->remote.identity_present;

  entry->id = peer->id;
  entry->mitm_protection = peer->mitm_protection;
  entry->secure_pairing = peer->secure_pairing;
}

static
void peer_entry_load(const struct ble_security_db_entry_s *entry,
                     struct ble_peer_s *peer)
{
  memcpy(peer->remote.ltk, entry->ltk, 16);
  memcpy(peer->remote.irk, entry->irk, 16);
  memcpy(peer->remote.csrk, entry->csrk, 16);
  memcpy(peer->addr.addr, entry->addr, 6);
  peer->remote.rand = entry->rand;
  peer->remote.ediv = entry->ediv;

  peer->addr.type = entry->addr_random ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
  peer->addr_present = entry->addr_present;
  peer->remote.ltk_present = entry->ltk_present;
  peer->remote.irk_present = entry->irk_present;
  peer->remote.csrk_present = entry->csrk_present;
  peer->remote.identity_present = entry->identity_present;

  peer->id = entry->id;
  peer->mitm_protection = entry->mitm_protection;
  peer->secure_pairing = entry->secure_pairing;

  peer->bonded = 1;
}

error_t ble_security_db_init(struct ble_security_db_s *db,
                             const char *persist,
                             const char *aes,
                             struct dev_rng_s *rng)
{
  error_t err;

  memset(db, 0, sizeof(*db));

#if defined(CONFIG_DEVICE_PERSIST)
  err = device_get_accessor_by_path(&db->persist.base, NULL, persist, DRIVER_CLASS_PERSIST);
  if (err)
    return err;
#endif

  err = device_get_accessor_by_path(&db->aes.base, NULL, aes, DRIVER_CLASS_CRYPTO);
  if (err)
    goto put_persist;

  db->rng = rng;

#if defined(CONFIG_DEVICE_PERSIST)
  const void *tmp;
  err = dev_persist_wait_read(&db->persist, &security_db_pk_blob, 0, &tmp);
  if (!err) {
    memcpy(db->pk, tmp, 16);

    logk("loaded device private key: %P", db->pk, 16);
  } else {
#endif
    err = dev_rng_wait_read(db->rng, db->pk, 16);
    if (err)
      goto put_aes;

#if defined(CONFIG_DEVICE_PERSIST)
    logk("created device private key: %P", db->pk, 16);

    dev_persist_wait_write(&db->persist, &security_db_pk_blob, 0, db->pk);
    // We can still run until reboot if key fails to write...
  }
#endif

#if defined(CONFIG_DEVICE_PERSIST)
  err = dev_persist_wait_read(&db->persist, &security_db_device_counter, 0, &tmp);
  if (err)
    dev_persist_wait_inc(&db->persist, &security_db_device_counter, 0);
#endif

  err = ble_security_db_key_get(db, 0, KEY_HANDLE_IRK, db->irk);
  if (err)
    goto put_aes;

  logk("device IRK: %P", db->irk, 16);

#if defined(CONFIG_DEVICE_PERSIST)
  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    const struct ble_security_db_entry_s *entry;
    err = dev_persist_wait_read(&db->persist, &security_db_entry_blob, i, (const void**)&entry);

    if (err) {
      db->paired_id[i] = 0;
    } else {
      db->paired_id[i] = entry->id;

      for (uint8_t j = 0; j < i; ++j) {
        if (db->paired_id[i] == db->paired_id[j]) {
          db->paired_id[i] = 0;
          dev_persist_wait_remove(&db->persist, &security_db_entry_blob, i);
          dev_persist_wait_remove(&db->persist, &security_db_subscribed_blob, i);
          goto next;
        }
      }

      logk("peer %lld slot %d", db->paired_id[i], i);
      if (entry->addr_present) {
        struct ble_addr_s addr;
        peer_entry_addr_load(entry, &addr);
        logk("  address " BLE_ADDR_FMT "", BLE_ADDR_ARG(&addr));
      }
      if (entry->irk_present) {
        logk("  IRK %P", entry->irk, 16);
      }
      uint8_t tmp[16];
      ble_security_db_key_get(db, db->paired_id[i], KEY_HANDLE_LTK, tmp);
      logk("  LTK %P", tmp, 16);
    }

  next:;
  }
#endif

  return 0;

 put_aes:
  device_put_accessor(&db->aes.base);
 put_persist:
#if defined(CONFIG_DEVICE_PERSIST)
  device_put_accessor(&db->persist.base);
#endif
  return err;
}

void ble_security_db_cleanup(struct ble_security_db_s *db)
{
#if defined(CONFIG_DEVICE_PERSIST)
  device_put_accessor(&db->persist.base);
#endif
  device_put_accessor(&db->aes.base);
}

#if defined(CONFIG_DEVICE_PERSIST)
uint_fast8_t ble_security_db_count(struct ble_security_db_s *db)
{
  uint_fast8_t ret = 0;

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (db->paired_id[i] == 0)
      continue;

    ret++;
  }

  return ret;
}
#endif

error_t ble_security_db_peer_reconnect_addr_set(struct ble_security_db_s *db,
                                                uint8_t slot, struct ble_addr_s *addr)
{
#if defined(CONFIG_DEVICE_PERSIST)
  if (db->paired_id[slot] == 0)
    return -ENOENT;

  const struct ble_security_db_entry_s *entry;
  error_t err = dev_persist_wait_read(&db->persist, &security_db_entry_blob,
                                      slot, (const void**)&entry);

  if (err)
    return err;

  if (entry->addr_present) {
    peer_entry_addr_load(entry, addr);
    return 0;
  }

  if (entry->irk_present) {
    ble_addr_random_type_set(addr, BLE_ADDR_RANDOM_RESOLVABLE);
    ble_ah(&db->aes, entry->irk, addr->addr + 3, addr->addr);
    return 0;
  }
#endif

  return -EINVAL;
}

#if defined(CONFIG_DEVICE_PERSIST)
bool_t ble_security_db_contains(struct ble_security_db_s *db,
                                const struct ble_addr_s *addr)
{
  enum ble_addr_random_type_e rt = ble_addr_random_type(addr);
  error_t err;

  if (addr->type != BLE_ADDR_PUBLIC
      && rt != BLE_ADDR_RANDOM_STATIC
      && rt != BLE_ADDR_RANDOM_RESOLVABLE)
    return 0;

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

      ble_ah(&db->aes, entry->irk, addr->addr + 3, tmp);
      if (memcmp(tmp, addr->addr, 3))
        continue;
    }

    return 1;
  }

  return 0;
}
#endif

error_t ble_security_db_load(struct ble_security_db_s *db,
                               const struct ble_addr_s *addr,
                               struct ble_peer_s *peer)
{
  enum ble_addr_random_type_e rt = ble_addr_random_type(addr);
  error_t err;

  ble_peer_init(peer, db, addr);

#if defined(CONFIG_DEVICE_PERSIST)
  logk_trace("looking up "BLE_ADDR_FMT"", BLE_ADDR_ARG(addr));

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

      ble_ah(&db->aes, entry->irk, addr->addr + 3, tmp);
      if (memcmp(tmp, addr->addr, 3))
        continue;
    }

    peer_entry_load(entry, peer);
    peer_subscribed_load(db, peer, i);
    return 0;
  }
#endif

  return -ENOENT;
}

/**
   Add a security_db entry.

   Select oldest entry to overwrite.
 */
error_t ble_security_db_save(struct ble_security_db_s *db,
                             const struct ble_peer_s *peer)
{
#if defined(CONFIG_DEVICE_PERSIST)
  uint64_t oldest_id = -1;
  int8_t index = -1;
  struct ble_security_db_entry_s entry;
  error_t err;
  struct ble_peer_s existing;

  if (peer->id == 0)
    return -EINVAL;

  logk("saving peer id %lld", peer->id);

  peer_entry_store(&entry, peer);

  // Check collision
  err = ble_security_db_load(db, &peer->lookup_addr, &existing);
  if (err) {
    // Entry with id == 0 can always be overwritten
    existing.id = 0;
  } else {
    logk("address "BLE_ADDR_FMT" found present as peer %lld", BLE_ADDR_ARG(&peer->lookup_addr), existing.id);
  }

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (existing.id && db->paired_id[i] == existing.id) {
      index = i;

      logk("found slot %d with id %lld matching peer to save", i, db->paired_id[i]);

      goto write;
    }

    if (db->paired_id[i] < oldest_id) {
      oldest_id = db->paired_id[i];
      index = i;
    }

    if (db->paired_id[i] == -1 || db->paired_id[i] == 0) {
      index = i;

      logk("found entry %d to be free", i);

      goto write;
    }
  }

  logk("using slot %d as oldest to replace", index);

 write:
  db->paired_id[index] = peer->id;

  logk("saving peer %lld slot %d: %P...", peer->id, index, &entry, sizeof(entry));

  err = dev_persist_wait_write(&db->persist, &security_db_entry_blob, index, &entry);
  peer_subscribed_save(db, peer, index);

  if (err)
    logk_error("error %d", err);

  return err;
#else
  return 0;
#endif
}

/**
   Erase a security_db entry.
 */
error_t ble_security_db_remove(struct ble_security_db_s *db,
                               uint64_t id)
{
#if defined(CONFIG_DEVICE_PERSIST)
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
#else
  return -ENOENT;
#endif
}

/**
   Erase all security_db entries.
 */
void ble_security_db_clear(struct ble_security_db_s *db)
{
#if defined(CONFIG_DEVICE_PERSIST)
  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (!db->paired_id[i])
      continue;

    db->paired_id[i] = 0;
    dev_persist_wait_remove(&db->persist, &security_db_entry_blob, i);
  }
#endif
}

error_t ble_peer_lookup_id(struct ble_security_db_s *db,
                           struct ble_peer_s *peer,
                           const uint8_t *random,
                           const uint16_t ediv)
{
#if defined(CONFIG_DEVICE_PERSIST)
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

    logk("load peer %lld slot %d: %d", did, i, err);

    if (err) {
      // This should not happen
      db->paired_id[i] = 0;
      continue;
    }

    peer_entry_load(entry, peer);
    peer_subscribed_load(db, peer, i);
    return 0;
  }
#endif

  return -ENOENT;
}

error_t ble_security_db_next_id(struct ble_security_db_s *db,
                                uint64_t *value)
{
#if defined(CONFIG_DEVICE_PERSIST)
  error_t err;

  logk_trace("wait inc");

  err = dev_persist_wait_inc(&db->persist, &security_db_device_counter, 0);
  if (err)
    return err;

  logk_trace("read");

  return dev_persist_wait_counter_read(&db->persist, &security_db_device_counter,
                                       0, value);
#endif
  *value = 0;

  return 0;
}
