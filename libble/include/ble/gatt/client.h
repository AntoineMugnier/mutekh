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

#ifndef BLE_GATT_CLIENT_H
#define BLE_GATT_CLIENT_H

#include <hexo/types.h>
#include <hexo/decls.h>

#include "db.h"

/**
   @file
   @module{BLE library}
   @short GATT Client

   @section {Description}

   A GATT client is an access to a GATT database.  It can be
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
struct ble_gatt_db_s;
struct ble_gatt_db_service_s;
struct ble_gatt_client_s;
struct ble_subscription_s;
struct ble_gatt_client_subs_s;

#define GCT_CONTAINER_ALGO_ble_gatt_client_sl CLIST

struct ble_gatt_client_subs_s
{
  struct ble_gatt_db_listener_s listener;
  uint16_t mode;
  struct ble_gatt_client_s *client;
  GCT_CONTAINER_ENTRY(ble_gatt_client_sl, client_item);
};

STRUCT_COMPOSE(ble_gatt_client_subs_s, listener);

GCT_CONTAINER_TYPES(ble_gatt_client_sl, struct ble_gatt_client_subs_s *, client_item);

struct ble_gatt_client_handler_s
{
  void (*att_value_changed)(struct ble_gatt_client_s *client, uint16_t value_handle,
                            uint16_t mode, const void *data, size_t size);
  void (*att_subscription_changed)(struct ble_gatt_client_s *client);
};

struct ble_gatt_client_s
{
  struct ble_gatt_db_s *db;
  const struct ble_gatt_client_handler_s *handler;
  ble_gatt_client_sl_root_t subs_list;
  struct {
    struct ble_gatt_db_service_s *service;
    uint16_t handle;
  } cursor;
  bool_t encrypted : 1;
  bool_t authenticated : 1;
};

void ble_gatt_client_db_open(struct ble_gatt_client_s *client,
                             const struct ble_gatt_client_handler_s *handler,
                             struct ble_gatt_db_s *db);

void ble_gatt_client_db_close(struct ble_gatt_client_s *client);

ALWAYS_INLINE
uint16_t ble_gatt_client_tell(struct ble_gatt_client_s *client)
{
  return client->cursor.handle;
}

enum ble_att_error_e ble_gatt_client_seek(struct ble_gatt_client_s *client,
                                          uint16_t handle);

enum ble_att_error_e ble_gatt_client_next(struct ble_gatt_client_s *client);

enum ble_att_error_e ble_gatt_client_type_get(struct ble_gatt_client_s *client,
                                              const struct ble_uuid_s **type);

enum ble_att_error_e ble_gatt_client_read(struct ble_gatt_client_s *client,
                                          size_t offset,
                                          void *data, size_t *size);

enum ble_att_error_e ble_gatt_client_write(struct ble_gatt_client_s *client,
                                           const void *data, size_t size);

void ble_gatt_client_subscription_get(struct ble_gatt_client_s *client,
                                      struct ble_subscription_s *subscriptions,
                                      size_t count);

void ble_gatt_client_subscription_set(struct ble_gatt_client_s *client,
                                      const struct ble_subscription_s *subscriptions,
                                      size_t count);

#endif
