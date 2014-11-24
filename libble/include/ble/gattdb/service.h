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
   @module {Bluetooth Low Energy library}
   @short GATT Database Service declaration

   @xsee {GATT Database}
*/

#include <hexo/types.h>
#include <ble/uuid.h>

/**
   @this defines various characteristic data modes.
   @xsee {Characteristic data modes}
 */
enum ble_gattdb_characteristic_mode_e
{
  /** Constant data mode, characteristic data will be defined in @ref
      {ble_gattdb_characteristic_s::data::constant}.
   */
  BLE_GATTDB_CHARACTERISTIC_CONSTANT,
  /** Plain data mode, characteristic data will be defined in @ref
      {ble_gattdb_characteristic_s::data::plain}.
   */
  BLE_GATTDB_CHARACTERISTIC_PLAIN,
  /** Dynamic data mode, characteristic data will be defined in @ref
      {ble_gattdb_characteristic_s::data::dynamic}.
   */
  BLE_GATTDB_CHARACTERISTIC_DYNAMIC,
#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  /** Dynamic Prepared data mode, this is still unimplemented.
   */
  BLE_GATTDB_CHARACTERISTIC_DYNAMIC_PREPARED,
#endif
};

/**
   @this defines various characteristic access permissions.
   @xsee {Characteristic perimissions}
 */
enum ble_gattdb_permission_e
{
  /** Characteristic may be read by anyone */
  BLE_GATTDB_PERM_OTHER_READ = (1 << 0),
  /** Characteristic may be written by anyone */
  BLE_GATTDB_PERM_OTHER_WRITE = (1 << 1),

#if defined(CONFIG_BLE_CRYPTO)
  /** Characteristic may be read over an encrypted link */
  BLE_GATTDB_PERM_ENC_READ = (1 << 2),
  /** Characteristic may be written over an encrypted link */
  BLE_GATTDB_PERM_ENC_WRITE = (1 << 3),
  /** Characteristic may be read over an secure link */
  BLE_GATTDB_PERM_AUTH_READ = (1 << 4),
  /** Characteristic may be written over an secure link */
  BLE_GATTDB_PERM_AUTH_WRITE = (1 << 5),
#else
  BLE_GATTDB_PERM_ENC_READ = BLE_GATTDB_PERM_OTHER_READ,
  BLE_GATTDB_PERM_ENC_WRITE = BLE_GATTDB_PERM_OTHER_WRITE,
  BLE_GATTDB_PERM_AUTH_READ = BLE_GATTDB_PERM_OTHER_READ,
  BLE_GATTDB_PERM_AUTH_WRITE = BLE_GATTDB_PERM_OTHER_WRITE,
#endif

  /** Characteristic may notified */
  BLE_GATTDB_NOTIFIABLE = (1 << 6),
  /** Characteristic may indicated */
  BLE_GATTDB_INDICABLE = (1 << 7),

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
  /** Characteristic may broadcasted (still unimplemented) */
  BLE_GATTDB_BROADCASTABLE = (1 << 8),
#endif
};

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
/**
   @this defines additional descriptor data.  This is needed for
   non-gatt descirptors (CCCD is not concerned here).
 */
struct ble_gattdb_descriptor_s
{
  /** Descriptor type */
  const struct ble_uuid_s *type;
  /** Descriptor value */
  const void *data;
  /** Descriptor value size, it is static */
  uint16_t size;
  /** Whether to allow client to write to characteristic */
  bool_t writable;
};
#endif

struct ble_gattdb_client_s;
struct ble_gattdb_registry_s;

/**
   @this defines a characteristic in the service definition.
 */
struct ble_gattdb_characteristic_s
{
  /** Characteristic type */
  const struct ble_uuid_s *type;
  /** Characteristic permissions */
  uint16_t permissions;
  /** Characteristic data mode */
  enum ble_gattdb_characteristic_mode_e mode : 8;
#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  /** Characteristic descriptor count */
  uint8_t descriptor_count;
  /** Characteristic descriptor array */
  const struct ble_gattdb_descriptor_s *descriptor;
#endif

  /** Characteristic data definition, depending on @tt mode */
  union {
    /** Characteris data constant mode */
    struct {
      /** Data buffer */
      const void *data;
      /** Data buffer size */
      size_t size;
    } constant;

    /** Characteristic data plain mode */
    struct {
      /** Data buffer */
      void *data;
      /** Data buffer size */
      size_t size;

      /** Callback on subscription of characteristic */
      uint8_t (*on_subscribe)(struct ble_gattdb_registry_s *reg, uint8_t charid,
                              bool_t subscribed);

      /** Callback on value change of characteristic.  If this
          function returns an error, write is rejected and value is
          not updated.

          @returns @ref {ble_att_error_e} {an error value}
      */
      uint8_t (*on_changed)(struct ble_gattdb_client_s *client,
                            struct ble_gattdb_registry_s *reg, uint8_t charid);
    } plain;

    /** Characteristic data dynamic mode */
    struct {
      /** Callback on subscription of characteristic */
      uint8_t (*on_subscribe)(struct ble_gattdb_registry_s *reg, uint8_t charid,
                              bool_t subscribed);

      /** Callback on write to characteristic. */
      uint8_t (*on_write)(struct ble_gattdb_client_s *client,
                          struct ble_gattdb_registry_s *reg, uint8_t charid,
                          const void *data, size_t size);

      /** Callback on read from characteristic. */
      uint8_t (*on_read)(struct ble_gattdb_client_s *client,
                         struct ble_gattdb_registry_s *reg, uint8_t charid,
                         uint16_t offset,
                         void *data, size_t *size);
    } dynamic;

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    /** Characteristic dynamic prepared mode, unimplemented */
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

/**
   @this defines the service definition for GATT Database.
   @xsee {Services declaration}
 */
struct ble_gattdb_service_s
{
  /** Service type */
  const struct ble_uuid_s *type;
#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  /** Service include list, NULL terminated */
  const struct ble_gattdb_service_s **include;
#endif
  /** Service characteristic array */
  const struct ble_gattdb_characteristic_s *characteristic;
  /** Service characteristic array size */
  uint8_t characteristic_count;
  /** @ref {ble_gattdb_service_flags_e} {Service flags} */
  uint8_t flags;
};

/**
   @this defines service flags.
   @xsee {Service flags}
*/
enum ble_gattdb_service_flags_e {
  /** Service will be declared as Primary service in the DB */
  BLE_GATTDB_SERVICE_PRIMARY = 1,
  /** Service will be listed in @ref ble_gattdb_srv16_list_get and
      @ref ble_gattdb_srv128_list_get. */
  BLE_GATTDB_SERVICE_ADVERTISED = 2,
};

/**
   Helper to define a characteristic in Constant mode, where backing
   data is a blob.
 */
#define BLE_GATTDB_CHAR_CONSTANT_BLOB(type_, value_, size_)  \
    {                                                           \
        .type = type_,                                          \
        .permissions = BLE_GATTDB_PERM_OTHER_READ,                \
        .mode = BLE_GATTDB_CHARACTERISTIC_CONSTANT,               \
        .data.constant.data = (value_),                         \
        .data.constant.size = (size_),                          \
    }

/**
   Helper to define a characteristic in Constant mode, where backing
   data is a string.
 */
#define BLE_GATTDB_CHAR_CONSTANT_STRING(type_, value_)       \
    BLE_GATTDB_CHAR_CONSTANT_BLOB(type_, value_, sizeof(value_) - 1)


#if defined(CONFIG_BLE_GATTDB_INCLUDE)
/**
   Helper to define inclusion list for a service.
 */
# define BLE_GATTDB_SERVICE_INCLUDED_(included_)                  \
  .include = included_ + 0,
#else
# define BLE_GATTDB_SERVICE_INCLUDED_(included_)
#endif

/**
   Helper to define a service.
 */
#define BLE_GATTDB_SERVICE_DECL(name_, flags_, type_, included_, chars_...) \
  const struct ble_gattdb_service_s name_ = {                             \
    .type = type_,                                          \
    .characteristic = (const struct ble_gattdb_characteristic_s[]){ chars_ }, \
    .characteristic_count = ARRAY_SIZE(((const struct ble_gattdb_characteristic_s[]){ chars_ })), \
    .flags = flags_,                                                    \
    BLE_GATTDB_SERVICE_INCLUDED_(included_)                   \
  }

/**
   Helper to define a characteristic.
 */
#define BLE_GATTDB_CHAR(type_, perms_, args_...)              \
  {                                                         \
    .type = (type_),                                        \
    .permissions = (perms_),                                \
    args_                                                   \
  }

/**
   Helper to define a characteristic data in constant mode.
 */
#define BLE_GATTDB_CHAR_DATA_CONSTANT(data_, size_)           \
  .mode = BLE_GATTDB_CHARACTERISTIC_CONSTANT,                 \
  .data.constant.data = (data_),                            \
  .data.constant.size = (size_)

/**
   Helper to define a characteristic data in plain mode.
 */
#define BLE_GATTDB_CHAR_DATA_PLAIN(data_, size_, subs_, changed_)  \
  .mode = BLE_GATTDB_CHARACTERISTIC_PLAIN,                         \
  .data.plain.data = (data_),                                    \
  .data.plain.size = (size_),                                  \
  .data.plain.on_subscribe = (subs_),                          \
  .data.plain.on_changed = (changed_)

/**
   Helper to define a characteristic data in dynamic mode.
 */
#define BLE_GATTDB_CHAR_DATA_DYNAMIC(read_, write_, subs_)  \
  .mode = BLE_GATTDB_CHARACTERISTIC_DYNAMIC,                \
  .data.dynamic.on_read = (read_),                        \
  .data.dynamic.on_write = (write_),                      \
  .data.dynamic.on_subscribe = (subs_)

/**
   Helper to define descriptors in a characteristic.
 */
#define BLE_GATTDB_DESCRIPTORS(items_...)  \
  .descriptor_count = ARRAY_SIZE(((const struct ble_gattdb_descriptor_s[]){ items_ })), \
  .descriptor = (const struct ble_gattdb_descriptor_s[]){ items_ }

/**
   Helper to define a @em {user description} descriptor.
 */
#define BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION(desc_)            \
    {                                                       \
      .type = BLE_UUID_BT_BASED_P(BLE_GATT_DESC_GATT_CHARACTERISTIC_USER_DESCRIPTION), \
      .data = desc_,      \
      .size = sizeof(desc_) - 1,                   \
    }


#endif
