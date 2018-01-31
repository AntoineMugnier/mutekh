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

#ifndef BLE_GATTDB_H
#define BLE_GATTDB_H

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short GATT Database context

   A GATT database merges GATT service definitions in a GATT database,
   each registered GATT service has a @ref {ble_gattdb_registry_s}
   {registry context}.
*/

#include <hexo/types.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/container_clist.h>

#include <ble/uuid.h>

struct ble_gattdb_registry_s;

#define GCT_CONTAINER_ALGO_ble_gattdb_registry_list CLIST
#define GCT_CONTAINER_ALGO_ble_gattdb_listener_list CLIST

struct ble_gattdb_client_s;
struct ble_gattdb_listener_s;

#define BLE_GATTDB_LISTENER_FUNC(x)                     \
  void (x)(struct ble_gattdb_listener_s *listener,      \
           struct ble_gattdb_registry_s *registry,        \
           bool_t reliable,                              \
           const void *data, size_t size)

typedef BLE_GATTDB_LISTENER_FUNC(ble_gattdb_registry_value_changed_func_t);

struct ble_gattdb_listener_s
{
  GCT_CONTAINER_ENTRY(ble_gattdb_listener_list, item);
  ble_gattdb_registry_value_changed_func_t *value_changed;
  struct ble_gattdb_registry_s *registry;
  uint8_t chr_index;
};

GCT_CONTAINER_TYPES(ble_gattdb_listener_list, struct ble_gattdb_listener_s *, item);

enum ble_gattdb_hndl_type_e
{
  BLE_GATTDB_HNDL_SERVICE,
#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  BLE_GATTDB_HNDL_INCLUDE,
#endif
  BLE_GATTDB_HNDL_CHAR_TYPE,
  BLE_GATTDB_HNDL_CHAR_VALUE,
  BLE_GATTDB_HNDL_CCCD,
#if defined(CONFIG_BLE_GATTDB_BROADCAST)
  BLE_GATTDB_HNDL_SCCD,
#endif
#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  BLE_GATTDB_HNDL_CEP,
#endif
#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  BLE_GATTDB_HNDL_CHAR_DESC,
#endif
};

struct ble_gattdb_hndl_s
{
  uint16_t index : 8;
#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  uint16_t descriptor : 5;
  uint16_t type : 3;
#else
  uint16_t type : 8;
#endif
} __attribute__((__packed__));

struct ble_gattdb_registry_s
{
  GCT_CONTAINER_ENTRY(ble_gattdb_registry_list, db_item);
  ble_gattdb_listener_list_root_t listener_list;

  const struct ble_gattdb_service_s *service;
  struct ble_gattdb_s *db;
  struct ble_gattdb_hndl_s *handle;

  uint16_t start_handle;
  uint8_t index;
  uint8_t handle_count;
#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  uint8_t include_count;
#endif
};

GCT_CONTAINER_TYPES(ble_gattdb_registry_list, struct ble_gattdb_registry_s *, db_item);

GCT_CONTAINER_FCNS(ble_gattdb_registry_list, ALWAYS_INLINE, ble_gattdb_registry_list,
                   init, destroy, pushback, remove, isempty, head, tail, next, prev);

struct ble_gattdb_s
{
  ble_gattdb_registry_list_root_t registry_list;
  // Own build-in gatt info server
  struct ble_gattdb_registry_s gattdb_registry;
};

error_t ble_gattdb_init(struct ble_gattdb_s *db);
void ble_gattdb_cleanup(struct ble_gattdb_s *db);


struct ble_gattdb_registry_s *
ble_gattdb_registry_get_by_index(struct ble_gattdb_s *db,
    uint_fast8_t index);

struct ble_gattdb_registry_s *
ble_gattdb_registry_get_by_type(struct ble_gattdb_s *db,
                                const struct ble_uuid_s *type,
                                uint_fast8_t index);

struct ble_gattdb_registry_s *
ble_gattdb_registry_get_by_service(struct ble_gattdb_s *db,
                                   const struct ble_gattdb_service_s *service,
                                   uint_fast8_t index);

error_t ble_gattdb_service_register(struct ble_gattdb_registry_s *reg,
                                     struct ble_gattdb_s *db,
                                     const struct ble_gattdb_service_s *service);

void ble_gattdb_service_unregister(struct ble_gattdb_registry_s *reg);

/**
  @this sends notifications to listeners that characteristic changed
  value.  This does not update the value to the characteristic
  backend.

  @param reg Registry handle
  @param charid Characteristic offset in service
*/
void ble_gattdb_char_changed(struct ble_gattdb_registry_s *reg,
                              uint8_t charid, bool_t reliable,
                              const void *data, size_t size);

#if defined(CONFIG_BLE_GATTDB_STREAM)
void ble_gattdb_char_stream_resume(struct ble_gattdb_registry_s *reg,
                                   uint8_t charid);
#endif

void ble_gattdb_listener_register(struct ble_gattdb_registry_s *reg,
                                   struct ble_gattdb_listener_s *listener,
                                   ble_gattdb_registry_value_changed_func_t *func,
                                   uint8_t chr_index);

void ble_gattdb_listener_unregister(struct ble_gattdb_registry_s *reg,
                                     struct ble_gattdb_listener_s *listener);

/**
   @this reads a standard characteristic for a standard service in
   database.  It lookups for a couple of UUID16s.

   Retrieved data pointer is writted to @tt data, data buffer size is
   written in integer pointed by @tt size. Data is usable in place.
 */
error_t ble_gattdb_std_char_read(struct ble_gattdb_s *db,
    uint16_t srv_uuid16, uint16_t chr_uuid16,
    const void **data, size_t *size);

/**
   @this retrieves UUID16 service list for use in advertisement data.
   @tt srvlist is filled with uuid16s of advertised services up to @tt
   size bytes.

   Return value is total count of bytes needed to contain all exported
   services, even if only @tt size first bytes of @tt srvlist are
   filled.
 */
size_t ble_gattdb_srv16_list_get(struct ble_gattdb_s *db,
    void *srvlist, size_t size);

/**
   @this retrieves UUID128 service list for use in advertisement
   data. UUID16 service types are not returned.  @tt srvlist is filled
   with uuid128s of advertised services up to @tt size bytes, in
   relevant byte order.

   Return value is total count of bytes needed to contain all exported
   services, even if only @tt size first bytes of @tt srvlist are
   filled. Return value is always multiple of 16 bytes.
 */
size_t ble_gattdb_srv128_list_get(struct ble_gattdb_s *db,
    void *srvlist, size_t size);

#endif
