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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_GATTDB_CLIENT_H
#define BLE_GATTDB_CLIENT_H

#include <hexo/types.h>
#include <hexo/decls.h>

#include <ble/gattdb/db.h>

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short GATT Database Client

   @section {Description}

   A GATT DB client is an access to a GATT database.  It can be
   concurrent to other clients for a given database.

   Client is the consumer-side of a GATT database.  For a typical BLE
   Peripheral, it is the connection from a remote Central.

   Client has a lookup role, where it can query the database.  It also
   has a publish/subscribe role where it can monitor a particular
   attribute value.

   @end section
*/

struct ble_uuid_s;
struct ble_peer_s;
struct ble_gattdb_s;
struct ble_gattdb_service_s;
struct ble_gattdb_registry_s;
struct ble_gattdb_client_s;
struct ble_subscription_s;
struct ble_gattdb_client_subs_s;
struct buffer_s;

#define GCT_CONTAINER_ALGO_ble_gattdb_client_sl CLIST

struct ble_gattdb_client_subs_s
{
  struct ble_gattdb_listener_s listener;
  uint16_t mode;
  struct ble_gattdb_client_s *client;
  GCT_CONTAINER_ENTRY(ble_gattdb_client_sl, client_item);
};

STRUCT_COMPOSE(ble_gattdb_client_subs_s, listener);

GCT_CONTAINER_TYPES(ble_gattdb_client_sl, struct ble_gattdb_client_subs_s *, client_item);

struct ble_gattdb_client_handler_s
{
  error_t (*att_value_changed)(struct ble_gattdb_client_s *client, uint16_t value_handle,
                               uint16_t mode, const void *data, size_t size);
#if defined(CONFIG_BLE_GATTDB_STREAM)
  void (*att_stream_resume)(struct ble_gattdb_client_s *client,
                            uint16_t value_handle);
#endif
  void (*att_subscription_changed)(struct ble_gattdb_client_s *client);
};

struct ble_gattdb_client_s
{
  struct ble_gattdb_s *db;
  const struct ble_gattdb_client_handler_s *handler;
  ble_gattdb_client_sl_root_t subs_list;
  struct {
    struct ble_gattdb_registry_s *registry;
    uint16_t handle;
  } cursor;
  bool_t encrypted : 1;
  bool_t authenticated : 1;
};

void ble_gattdb_client_open(struct ble_gattdb_client_s *client,
                             const struct ble_gattdb_client_handler_s *handler,
                             struct ble_gattdb_s *db);

void ble_gattdb_client_close(struct ble_gattdb_client_s *client);

ALWAYS_INLINE
uint16_t ble_gattdb_client_tell(struct ble_gattdb_client_s *client)
{
  return client->cursor.handle;
}

enum ble_att_error_e ble_gattdb_client_seek(struct ble_gattdb_client_s *client,
                                          uint16_t handle);

enum ble_att_error_e ble_gattdb_client_next(struct ble_gattdb_client_s *client);

enum ble_att_error_e ble_gattdb_client_type_get(struct ble_gattdb_client_s *client,
                                              const struct ble_uuid_s **type);

enum ble_att_error_e ble_gattdb_client_read(struct ble_gattdb_client_s *client,
                                          size_t offset,
                                          void *data, size_t *size);

enum ble_att_error_e ble_gattdb_client_write(struct ble_gattdb_client_s *client,
                                           const void *data, size_t size);

#if defined(CONFIG_BLE_GATTDB_STREAM)
error_t ble_gattdb_client_att_stream_get(struct ble_gattdb_client_s *client,
                                         uint16_t value_handle,
                                         struct buffer_s *buffer);
#endif

void ble_gattdb_client_subscription_get(struct ble_gattdb_client_s *client,
                                      struct ble_subscription_s *subscriptions,
                                      size_t count);

void ble_gattdb_client_subscription_set(struct ble_gattdb_client_s *client,
                                      const struct ble_subscription_s *subscriptions,
                                      size_t count);

/*
  @this implements a write from client.

  @param db Database
  @param reg Service handle
  @param charid Characteristic offset in service
*/
enum ble_att_error_e ble_gattdb_char_write(struct ble_gattdb_client_s *client,
                                            struct ble_gattdb_registry_s *reg,
                                            uint8_t charid,
                                            const void *data, size_t size);

/*
  @this implements a write from client.

  @param db Database
  @param reg Service handle
  @param charid Characteristic offset in service
*/
enum ble_att_error_e ble_gattdb_char_read(struct ble_gattdb_client_s *client,
                                           struct ble_gattdb_registry_s *reg,
                                           uint8_t charid, size_t offset,
                                           void *data, size_t *size);

#endif
