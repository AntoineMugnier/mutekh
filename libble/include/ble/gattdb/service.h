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

#ifndef BLE_GATTDB_SERVICE_H
#define BLE_GATTDB_SERVICE_H

/**
   @file
   @module{BLE library}
   @short GATT Service

   @section {Description}

   A GATT service is a static and constant data structure defining
   both schema and access to characteristics.

   Characteristics may be either:
   @list
   @item constant: values are read-only,
   @item plain: values are memory-backed,
   @item dynamic: values are handled through callbacks,
   @item dynamic_prepared: values are handled through callbacks and long writes are supported.
   @end list

   For each characteristic in a service, various properties and
   descriptors can be set.

   @end section
*/

#include <hexo/types.h>
#include <ble/uuid.h>

enum ble_gattdb_characteristic_mode_e
{
  BLE_GATTDB_CHARACTERISTIC_CONSTANT,
  BLE_GATTDB_CHARACTERISTIC_PLAIN,
  BLE_GATTDB_CHARACTERISTIC_DYNAMIC,
#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  BLE_GATTDB_CHARACTERISTIC_DYNAMIC_PREPARED,
#endif
  BLE_GATTDB_CHARACTERISTIC_RAW,
};

enum ble_gattdb_permission_e
{
  BLE_GATTDB_PERM_OTHER_READ = (1 << 0),
  BLE_GATTDB_PERM_OTHER_WRITE = (1 << 1),

#if defined(CONFIG_BLE_CRYPTO)
  BLE_GATTDB_PERM_ENC_READ = (1 << 2),
  BLE_GATTDB_PERM_ENC_WRITE = (1 << 3),
  BLE_GATTDB_PERM_AUTH_READ = (1 << 4),
  BLE_GATTDB_PERM_AUTH_WRITE = (1 << 5),
#else
  BLE_GATTDB_PERM_ENC_READ = BLE_GATTDB_PERM_OTHER_READ,
  BLE_GATTDB_PERM_ENC_WRITE = BLE_GATTDB_PERM_OTHER_WRITE,
  BLE_GATTDB_PERM_AUTH_READ = BLE_GATTDB_PERM_OTHER_READ,
  BLE_GATTDB_PERM_AUTH_WRITE = BLE_GATTDB_PERM_OTHER_WRITE,
#endif

  BLE_GATTDB_NOTIFIABLE = (1 << 6),
  BLE_GATTDB_INDICABLE = (1 << 7),

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
  BLE_GATTDB_BROADCASTABLE = (1 << 8),
#endif
};

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
struct ble_gattdb_descriptor_s
{
  const struct ble_uuid_s *type;
  const void *data;
  uint16_t size;
  bool_t writable;
};
#endif

struct ble_gattdb_client_s;
struct ble_gattdb_registry_s;

struct ble_gattdb_characteristic_s
{
  const struct ble_uuid_s *type;
  uint16_t permissions;
  enum ble_gattdb_characteristic_mode_e mode : 8;
#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  uint8_t descriptor_count;
  const struct ble_gattdb_descriptor_s *descriptor;
#endif

  union {
    struct {
      const void *data;
      size_t size;
    } constant;

    struct {
      void *data;
      size_t size;

      uint8_t (*on_subscribe)(struct ble_gattdb_registry_s *reg, uint8_t charid,
                              bool_t subscribed);

      uint8_t (*on_changed)(struct ble_gattdb_client_s *client,
                            struct ble_gattdb_registry_s *reg, uint8_t charid);
    } plain;

    struct {
      uint8_t (*on_subscribe)(struct ble_gattdb_registry_s *reg, uint8_t charid,
                              bool_t subscribed);

      uint8_t (*on_write)(struct ble_gattdb_client_s *client,
                          struct ble_gattdb_registry_s *reg, uint8_t charid,
                          const void *data, size_t size);

      uint8_t (*on_read)(struct ble_gattdb_client_s *client,
                         struct ble_gattdb_registry_s *reg, uint8_t charid,
                         uint16_t offset,
                         void *data, size_t *size);
    } dynamic;

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    struct {
      uint8_t (*on_subscribe)(struct ble_gattdb_registry_s *reg, uint8_t charid,
                              bool_t subscribed);

      uint8_t (*on_read)(struct ble_gattdb_client_s *client,
                         struct ble_gattdb_registry_s *reg, uint8_t charid,
                         uint16_t offset,
                         void *data, size_t *size);

      uint8_t (*on_write_part)(struct ble_gattdb_client_s *client,
                               struct ble_gattdb_registry_s *reg, uint8_t charid,
                               uint16_t offset,
                               const void *data, size_t size);

      uint8_t (*on_execute)(struct ble_gattdb_client_s *client,
                            struct ble_gattdb_registry_s *reg, uint8_t charid);
    } dynamic_prepared;
#endif
  } data;
};

struct ble_gattdb_service_s
{
  const struct ble_uuid_s *type;
#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  const struct ble_gattdb_service_s **include;
#endif
  const struct ble_gattdb_characteristic_s *characteristic;
  uint8_t characteristic_count;
  uint8_t flags;
};

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

enum ble_gattdb_service_flags_e {
  BLE_GATTDB_SERVICE_PRIMARY = 1,
  BLE_GATTDB_SERVICE_ADVERTISED = 2,
};

#define BLE_GATTDB_CHAR_CONSTANT_BLOB(type_, value_, size_)  \
    {                                                           \
        .type = type_,                                          \
        .permissions = BLE_GATTDB_PERM_OTHER_READ,                \
        .mode = BLE_GATTDB_CHARACTERISTIC_CONSTANT,               \
        .data.constant.data = (value_),                         \
        .data.constant.size = (size_),                          \
    }

#define BLE_GATTDB_CHAR_CONSTANT_STRING(type_, value_)       \
    BLE_GATTDB_CHAR_CONSTANT_BLOB(type_, value_, sizeof(value_) - 1)


#if defined(CONFIG_BLE_GATTDB_INCLUDE)
# define BLE_GATTDB_SERVICE_INCLUDED_(included_)                  \
  .include = included_ + 0,
#else
# define BLE_GATTDB_SERVICE_INCLUDED_(included_)
#endif

#define BLE_GATTDB_SERVICE_DECL(name_, flags_, type_, included_, chars_...) \
  const struct ble_gattdb_service_s name_ = {                             \
    .type = type_,                                          \
    .characteristic = (const struct ble_gattdb_characteristic_s[]){ chars_ }, \
    .characteristic_count = ARRAY_SIZE(((const struct ble_gattdb_characteristic_s[]){ chars_ })), \
    .flags = flags_,                                                    \
    BLE_GATTDB_SERVICE_INCLUDED_(included_)                   \
  }

#define BLE_GATTDB_CHAR(type_, perms_, args_...)              \
  {                                                         \
    .type = (type_),                                        \
    .permissions = (perms_),                                \
    args_                                                   \
  }

#define BLE_GATTDB_CHAR_DATA_CONSTANT(data_, size_)           \
  .mode = BLE_GATTDB_CHARACTERISTIC_CONSTANT,                 \
  .data.constant.data = (data_),                            \
  .data.constant.size = (size_)

#define BLE_GATTDB_CHAR_DATA_PLAIN(data_, size_, subs_, changed_)  \
  .mode = BLE_GATTDB_CHARACTERISTIC_PLAIN,                         \
  .data.plain.data = (data_),                                    \
  .data.plain.size = (size_),                                  \
  .data.plain.on_subscribe = (subs_),                          \
  .data.plain.on_changed = (changed_)

#define BLE_GATTDB_CHAR_DATA_DYNAMIC(read_, write_, subs_)  \
  .mode = BLE_GATTDB_CHARACTERISTIC_DYNAMIC,                \
  .data.dynamic.on_read = (read_),                        \
  .data.dynamic.on_write = (write_),                      \
  .data.dynamic.on_subscribe = (subs_)

#define BLE_GATTDB_DESCRIPTORS(items_...)  \
  .descriptor_count = ARRAY_SIZE(((const struct ble_gattdb_descriptor_s[]){ items_ })), \
  .descriptor = (const struct ble_gattdb_descriptor_s[]){ items_ }

#define BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION(desc_)            \
    {                                                       \
      .type = BLE_UUID_SHORT_P(BLE_UUID_GATT_CHARACTERISTIC_USER_DESCRIPTION_DESC), \
      .data = desc_,      \
      .size = sizeof(desc_) - 1,                   \
    }


#endif
