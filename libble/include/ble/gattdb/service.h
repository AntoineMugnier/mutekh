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

#include <ble/uuid.h>

/**
  @file
  @module {Bluetooth Low Energy library}
  @short GATT Database Service declaration

  A service declaration is a @ref {ble_gattdb_service_s} object
  containing all the characteristics and their parameters.

  Standard @em Service, @em Characteristic and @em Descriptors
  assigned-numbers are provided in relevant headers for @ref
  {@ble/gatt/service.h} {services}, @ref
  {@ble/gatt/characteristic.h} {characteristics}, and @ref
  {@ble/gatt/descriptor.h} {descriptors} assigned numbers, they can
  be used freely for all device's service declarations.

  Service, Characteristics and their parameters are defined
  through constant structures.  This can be written explicitly,
  but there are various macros provided to shorten the declaration
  process.

  Here is the explicit version of a @em {Device Information
  Service} declaration where we define characteristic values, UUID
  objects and characteristic array, and finally, service object.

  @example examples/ble/rgb_led/dis.c:long_service_decl

  But this code is too long and verbose. It should be shorter to
  get maintainable.  Hopefully, library defines helper macros to
  define services.

  For instance, the above declaration can be shortened as:

  @example examples/ble/rgb_led/dis.c:service_decl

  Because a service declaration is simple and explicit, there is no
  library of service declarations: this is unneeded.

  Then, registration of service is done through a call to
  @ref{ble_gattdb_service_register}.  This is enough to get the
  service exposed in the GATT database.  @ref {@ble/gattdb/db.h}.

  A custom service gets declared the same way.  For instance,
  Apple-designed @url
  {https://developer.apple.com/bluetooth/Apple-Bluetooth-Low-Energy-MIDI-Specification.pdf}
  {MIDI over BLE Service} gets declared as:

  @example examples/ble/midi/midi.c:service_decl

  This declares a service with type @tt
  {03B80E5A-EDE8-4B33-A751-6CE34EC4C700}.  This service will be
  declared as primary and advertised in AD.  It contains a
  write/notify secure characteristic with type @tt
  {7772E5DB-3868-4112-A1A9-F2669D106BF3}.  User-provided @tt
  {on_midi_data_write} function will be called to handle writes to
  the characteristic.

  See @sourceref {examples/ble/midi/midi.c} for complete example.

  @em Services defined with the GATT DB library only consider @em
  Characteristics.  All indices target characteristic index in
  service, or descriptor index in characteristic.  Handle
  allocation for attributes is completely hidden.  For instance,
  third argument to @tt on_midi_data_write above is the
  characteristic number in the characteristic array (will be @tt
  0).  This permits to set the same callback function for many
  characteristics.

  @section tU {Service flags}

    @ref {ble_gattdb_service_flags_e} {Service flags} let the service
    be defined as Primary, and allow to advertise the service in
    @em {Advertise Data (AD)}.

    If @ref {BLE_GATTDB_SERVICE_PRIMARY} {primary service flag} is not
    set, service will be exposed as @em {secondary service} in GATT
    DB.

    @ref {BLE_GATTDB_SERVICE_ADVERTISED} {Advertised service flag}
    makes the service UUID exported in @ref ble_gattdb_srv16_list_get
    and @ref ble_gattdb_srv128_list_get calls, used internally by
    stack context to build @em AD.

  @end section

  @section tU {Characteristic data modes}

    There are 3 main @ref {ble_gattdb_characteristic_mode_e}
    {characteristic modes}:
    @list
      @item @ref {BLE_GATTDB_CHARACTERISTIC_CONSTANT} {Constant},
        where characteristic value is a constant data object in
        memory, without callback functions;
      @item @ref {BLE_GATTDB_CHARACTERISTIC_PLAIN} {Plain}, where
        characteristic value is a global data object, but where
        callback functions may be defined to handle read, writes,
        and subscription to notification or indication;
      @item @ref {BLE_GATTDB_CHARACTERISTIC_DYNAMIC} {Dynamic},
        where data is dynamically computed for each access.
    @end list

    @section U {Constant mode}

      For @ref {ble_gattdb_characteristic_s::data::constant}
      {Constant mode}, user must define data blob and size.
      Relevant macro is @ref #BLE_GATTDB_CHAR_CONSTANT_BLOB.  There
      is a shortcut for constant string: @ref
      #BLE_GATTDB_CHAR_CONSTANT_STRING.

      @code
static const uint16_t appearance = BLE_GAP_APPEARANCE_HID_MOUSE;

BLE_GATTDB_SERVICE_DECL(gap_service,
  BLE_GATTDB_SERVICE_PRIMARY,
  BLE_UUID_SHORT_P(BLE_UUID_GENERIC_ACCESS_SERVICE),
  NULL,
  BLE_GATTDB_CHAR_CONSTANT_BLOB(
    BLE_UUID_SHORT_P(BLE_UUID_GAP_APPEARANCE_CHAR),
    &appearance, sizeof(appearance)),
  BLE_GATTDB_CHAR_CONSTANT_STRING(
    BLE_UUID_SHORT_P(BLE_UUID_GAP_DEVICE_NAME_CHAR),
    "Mouse"),
  );
      @end code

    @end section

    @section U {Plain mode}

      For @ref {ble_gattdb_characteristic_s::data::plain} {Plain mode},
      user must define data blob and size, and may specify callbacks for
      subscription and write actions.

      @code
static uint8_t battery_level;

static
uint8_t batt_level_subscribe(struct ble_gattdb_registry_s *reg,
                         uint8_t charid,
                         bool_t subscribed)
{
  printk("Battery level %s\n", subscribed ? "subscribed" : "unsubscribed");

  return 0;
}

BLE_GATTDB_SERVICE_DECL(
  batt_service,
  BLE_GATTDB_SERVICE_PRIMARY,
  BLE_UUID_SHORT_P(BLE_UUID_BATTERY_SERVICE),
  NULL,
  // One characteristic
  BLE_GATTDB_CHAR(
    // Type
    BLE_UUID_SHORT_P(BLE_UUID_BATTERY_LEVEL_CHAR),
    // Readable by all, notifiable
    BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_PERM_OTHER_READ,
    // Backed by a global data blob "battery_level"
    // Library must call batt_level_subscribe on subscription change
    // No write callback
    BLE_GATTDB_CHAR_DATA_PLAIN(&battery_level, sizeof(battery_level),
                           batt_level_subscribe, NULL),
  );
      @end code

    @end section

    @section U {Dynamic mode}

      For @ref {ble_gattdb_characteristic_s::data::dynamic} {Dynamic
      mode}, user must define callbacks for subscription, read and
      write actions, depending on characteristic access flags.

      @example examples/ble/midi/midi.c:write_handler|service_decl|details

    @end section

    For notifiable characteristics, code may call @ref
    ble_gattdb_char_changed to push a notification to GATT client.

  @end section

  @section tU {Characteristic perimissions}

    Characteristic permissions can limit how characteristic values are
    accessed by peer GATT client.  See @ref {ble_gattdb_permission_e}.

    Access to Characteristic values is an inclusive list: @tt
    {BLE_GATTDB_PERM_AUTH_...} implies @tt {BLE_GATTDB_PERM_ENC_...}
    implies @tt {BLE_GATTDB_PERM_OTHER_...}.  A client coming in
    clear text has @em OTHER permissions.  Once encrypted, @em ENC
    permissions are also granted, then, if pairing is authenticated,
    @em AUTH permissions are also granted.

    Library checks for readability for allowing subscription to
    characteristic value notifications and indications.  A @em
    dynamic characteristic that can not be read but notified only
    should still be declared as @ref BLE_GATTDB_PERM_OTHER_READ,
    @ref BLE_GATTDB_PERM_ENC_READ or @ref BLE_GATTDB_PERM_AUTH_READ
    in order for the library to tell how to enforce acces control to
    the value.  @ref
    {ble_gattdb_characteristic_s::data::dynamic::on_read} {on_read
    callback} can still be @tt NULL in such cases.

  @end section

  @section tU {Service includes}

    Service includes are mostly useful for standard profiles
    implementation.  In the library, service include is done
    referencing a service definition structure from another.  This
    could lead to some ambiguity if included service is present more
    than once in GATT DB, but this case never actually happens in
    practice.

  @end section
*/

#include <hexo/error.h>
#include <hexo/types.h>
#include <ble/uuid.h>

struct buffer_s;

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
#if defined(CONFIG_BLE_GATTDB_STREAM)
  /** Streaming data mode, characteristic may be read from, or
      notified/indicated, but not written to.  Gatt DB will call user
      code back to get more data to send to client.
   */
  BLE_GATTDB_CHARACTERISTIC_STREAM,
#endif
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

#if defined(CONFIG_BLE_GATTDB_STREAM)
    /** Characteristic data stream mode */
    struct {
      /** Callback on subscription */
      uint8_t (*on_subscribe)(struct ble_gattdb_registry_s *reg,
                              uint8_t charid,
                              bool_t subscribed);

      /** Callback on read from characteristic (used on explicit read,
          if supported). */
      uint8_t (*on_read)(struct ble_gattdb_client_s *client,
                         struct ble_gattdb_registry_s *reg, uint8_t charid,
                         uint16_t offset,
                         void *data, size_t *size);

      /** Callback getting next item to stream.  May return -EAGAIN to
          tell gattdb that there is no more data to stream available. */
      error_t (*on_get_data)(struct ble_gattdb_client_s *client,
                             struct ble_gattdb_registry_s *reg,
                             uint8_t charid,
                             struct buffer_s *buffer);
    } stream;
#endif

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

#if defined(CONFIG_BLE_GATTDB_STREAM)
/**
   Helper to define a characteristic data in streaming mode.
 */
#define BLE_GATTDB_CHAR_DATA_STREAM(read_, get_data_, subs_)  \
  .mode = BLE_GATTDB_CHARACTERISTIC_STREAM,                   \
  .data.stream.on_read = (read_),                            \
  .data.stream.on_get_data = (get_data_),                    \
  .data.stream.on_subscribe = (subs_)
#endif

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
