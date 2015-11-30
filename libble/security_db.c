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

#include <hexo/types.h>
#include <ble/protocol/address.h>

#include <mutek/printk.h>

#include <device/class/persist.h>
#include <device/class/crypto.h>
#include <ble/crypto.h>

#include <ble/security_db.h>
#include <ble/peer.h>
#include "security_db_priv.h"

#include <string.h>

#if 0
# define dprintk printk
#else
# define dprintk(k...) do {} while (0)
#endif

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

static const struct dev_persist_descriptor_s security_db_subscribed_blob = {
  .uid = 0x3220,
  .type = DEV_PERSIST_BLOB,
  .size = BLE_SUBSCRIBED_CHAR_COUNT * sizeof(struct ble_subscription_s),
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

#if defined(CONFIG_BLE_SECURITY_DB)
error_t ble_security_db_key_get(struct ble_security_db_s *db,
                                uint64_t did,
                                uint64_t key_handle,
                                uint8_t *out)
{
  uint64_t tmp[2] = { did, key_handle };

  return ble_e(&db->aes, db->pk, (const uint8_t *)tmp, out);
}

static
void peer_entry_addr_load(const struct ble_security_db_entry_s *entry,
                          struct ble_addr_s *addr)
{
  memcpy(addr->addr, entry->addr, 6);

  addr->type = entry->addr_random ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
}

static
void peer_entry_store(struct ble_security_db_entry_s *entry,
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

static
void peer_entry_load(const struct ble_security_db_entry_s *entry,
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
  memset(peer->subscriptions, 0xff, sizeof(peer->subscriptions));
}

void peer_subscribed_load(struct ble_security_db_s *db,
                          struct ble_peer_s *peer,
                          uint8_t offset)
{
  const void *data;

  if (dev_persist_wait_read(&db->persist, &security_db_subscribed_blob,
                            offset, &data) == 0) {
    memcpy(peer->subscriptions, data, sizeof(peer->subscriptions));
  } else {
    memset(peer->subscriptions, 0xff, sizeof(peer->subscriptions));
  }
}

error_t peer_subscribed_save(struct ble_security_db_s *db,
                             const struct ble_peer_s *peer,
                             uint8_t offset)
{
  return dev_persist_wait_write(&db->persist, &security_db_subscribed_blob,
                                offset, peer->subscriptions);
}
#endif

error_t ble_security_db_init(struct ble_security_db_s *db,
                             const char *persist,
                             const char *aes,
                             struct dev_rng_s *rng)
{
  error_t err;

  memset(db, 0, sizeof(*db));

#if defined(CONFIG_BLE_SECURITY_DB)
  err = device_get_accessor_by_path(&db->persist, NULL, persist, DRIVER_CLASS_PERSIST);
  if (err)
    return err;
#endif

  err = device_get_accessor_by_path(&db->aes, NULL, aes, DRIVER_CLASS_CRYPTO);
  if (err)
    goto put_persist;

  db->rng = rng;

#if defined(CONFIG_BLE_SECURITY_DB)
  const void *tmp;
  err = dev_persist_wait_read(&db->persist, &security_db_pk_blob, 0, &tmp);
  if (!err) {
    memcpy(db->pk, tmp, 16);

    printk("Loaded device private key: %P\n", db->pk, 16);
  } else {
    err = dev_rng_wait_read(db->rng, db->pk, 16);
    if (err)
      goto put_aes;

    printk("Created device private key: %P\n", db->pk, 16);

    dev_persist_wait_write(&db->persist, &security_db_pk_blob, 0, db->pk);
    // We can still run until reboot if key fails to write...
  }

  err = dev_persist_wait_read(&db->persist, &security_db_device_counter, 0, &tmp);
  if (err)
    dev_persist_wait_inc(&db->persist, &security_db_device_counter, 0);

  err = ble_security_db_key_get(db, 0, KEY_HANDLE_IRK, db->irk);
  if (err)
    goto put_aes;

  printk("Device IRK: %P\n", db->irk, 16);

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

      printk("SecDB peer %lld slot %d\n", db->paired_id[i], i);
      if (entry->addr_present) {
        struct ble_addr_s addr;
        peer_entry_addr_load(entry, &addr);
        printk("  address " BLE_ADDR_FMT "\n", BLE_ADDR_ARG(&addr));
      }
      if (entry->irk_present) {
        printk("  IRK %P\n", entry->irk, 16);
      }
      uint8_t tmp[16];
      ble_security_db_key_get(db, db->paired_id[i], KEY_HANDLE_LTK, tmp);
      printk("  LTK %P\n", tmp, 16);
    }

  next:;
  }
#endif

  return 0;

 put_aes:
  device_put_accessor(&db->aes);
 put_persist:
#if defined(CONFIG_BLE_SECURITY_DB)
  device_put_accessor(&db->persist);
#endif
  return err;
}

void ble_security_db_cleanup(struct ble_security_db_s *db)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  device_put_accessor(&db->persist);
#endif
  device_put_accessor(&db->aes);
}

#if defined(CONFIG_BLE_SECURITY_DB)
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


error_t ble_security_db_peer_reconnect_addr_set(struct ble_security_db_s *db,
                                                uint8_t slot, struct ble_addr_s *addr)
{
  if (db->paired_id[slot] == 0)
    return -ENOENT;

  const struct ble_security_db_entry_s *entry;
  error_t err = dev_persist_wait_read(&db->persist, &security_db_entry_blob, slot, (const void**)&entry);

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

  return -EINVAL;
}

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

static
error_t ble_security_db_load(struct ble_security_db_s *db,
                               const struct ble_addr_s *addr,
                               struct ble_peer_s *peer)
{
  enum ble_addr_random_type_e rt = ble_addr_random_type(addr);
  error_t err;

  ble_peer_init(peer, db, addr);

  printk("%s looking up "BLE_ADDR_FMT"\n", __FUNCTION__, BLE_ADDR_ARG(addr));

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

  printk("Saving peer id %lld\n", peer->id);

  peer_entry_store(&entry, peer);

  // Check collision
  err = ble_security_db_load(db, &peer->lookup_addr, &existing);
  if (err) {
    // Entry with id == 0 can always be overwritten
    existing.id = 0;
  } else {
    printk("Address "BLE_ADDR_FMT" found present as peer %lld\n", BLE_ADDR_ARG(&peer->lookup_addr), existing.id);
  }

  for (uint8_t i = 0; i < CONFIG_BLE_SECURITY_DB_MAX; ++i) {
    if (existing.id && db->paired_id[i] == existing.id) {
      index = i;

      printk("Found slot %d with id %lld matching peer to save\n", i, db->paired_id[i]);

      goto write;
    }

    if (db->paired_id[i] < oldest_id) {
      oldest_id = db->paired_id[i];
      index = i;
    }

    if (db->paired_id[i] == -1 || db->paired_id[i] == 0) {
      index = i;

      printk("Found entry %d to be free\n", i);

      goto write;
    }
  }

  printk("Using slot %d as oldest to replace\n", index);

 write:
  db->paired_id[index] = peer->id;

  printk("SecDB: Saving peer %lld slot %d: %P...", peer->id, index, &entry, sizeof(entry));

  err = dev_persist_wait_write(&db->persist, &security_db_entry_blob, index, &entry);
  peer_subscribed_save(db, peer, index);

  printk(" %d\n", err);

  return err;
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

    printk("%s load %lld slot %d: %d\n", __FUNCTION__, did, i, err);

    if (err) {
      // This should not happen
      db->paired_id[i] = 0;
      continue;
    }

    peer_entry_load(entry, peer);
    peer_subscribed_load(db, peer, i);
    return 0;
  }

  return -ENOENT;
}

error_t ble_security_db_next_id(struct ble_security_db_s *db,
                                uint64_t *value)
{
  error_t err;

  dprintk("%s wait inc\n", __FUNCTION__);

  err = dev_persist_wait_inc(&db->persist, &security_db_device_counter, 0);
  if (err)
    return err;

  dprintk("%s read\n", __FUNCTION__);

  return dev_persist_wait_counter_read(&db->persist, &security_db_device_counter,
                                       0, value);
}
#endif
