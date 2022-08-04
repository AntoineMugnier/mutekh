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

#ifndef BLE_UUID_H_
#define BLE_UUID_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short UUID definition and helpers

   Bluetooth uses UUIDs in a special way.  There are short-hand
   versions using a special "Bluetoot base UUID" as common value.  In
   MutekH, BLE library does not use shortened UUIDs in memory.
   Shortening is only done where protocol requires so.

   UUIDs are defined as a 16-byte structure.  @ref
   {ble_uuid_is_bluetooth_based} {A set of} @ref {ble_uuid_uuid16_get}
   {accessor functions} and a @ref {ble_uuid_cmp} {comparator}
   function are provided.

   @ref {#BLE_UUID_BT_BASED_P}, @ref {#BLE_UUID_P} yield valid
   pointers to constant UUIDs for given values.

   With the following code

   @code
static const struct ble_uuid_s dis_service_type_1 =
{
  .value = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00,
    0x00, 0x80,
    0x00, 0x10,
    0x00, 0x00,
    0x0a, 0x18, 0x00, 0x00},
};

static const struct ble_uuid_s dis_service_type_2 = BLE_UUID(0x180a, 0x0000, 0x1000, 0x8000, 0x00805f9b34fbULL);

static const struct ble_uuid_s dis_service_type_3 = BLE_UUID_BT_BASED(0x180a);
   @end code

   those five statements yield a pointer to a constant UUID object
   with value @tt {0000180a-0000-1000-8000-00805f9b34fb}.


   @code
&dis_service_type_1;
&dis_service_type_2;
&dis_service_type_3;
BLE_UUID_P(0x180a, 0x0000, 0x1000, 0x8000, 0x00805f9b34fbULL);
BLE_UUID_BT_BASED_P(0x180a);
   @end code
*/

#include <hexo/types.h>
#include <hexo/endian.h>

/**
   @this defines a UUID object.  UUIDs are stored in Bluetooth network
   order in memory.
 */
struct ble_uuid_s
{
  union {
    uint8_t value[16];
    uint16_t value16[8];
    uint32_t value32[4];
  };
};

/**
   @this is a shorthand initializer for UUID structure.  It expects
   the 5 parts of UUID value as arguments, in natural display order.

   The following code defines UUID @tt
   {12345678-3344-5566-7788-abcdef678901}:
   @code
   struct ble_uuid_s test = BLE_UUID(0x12345678, 0x3344, 0x5566, 0x7788, 0xabcdef678901ULL);
   @end code
 */
#define BLE_UUID(a, b, c, d, e) {               \
    .value = {                                  \
      0xff & ((uint64_t)(e)),                   \
      0xff & ((uint64_t)(e) >> 8),              \
      0xff & ((uint64_t)(e) >> 16),             \
      0xff & ((uint64_t)(e) >> 24),             \
      0xff & ((uint64_t)(e) >> 32),             \
      0xff & ((uint64_t)(e) >> 40),             \
      0xff & ((uint16_t)(d)),                   \
      0xff & ((uint16_t)(d) >> 8),              \
      0xff & ((uint16_t)(c)),                   \
      0xff & ((uint16_t)(c) >> 8),              \
      0xff & ((uint16_t)(b)),                   \
      0xff & ((uint16_t)(b) >> 8),              \
      0xff & ((uint32_t)(a)),                   \
      0xff & ((uint32_t)(a) >> 8),              \
      0xff & ((uint32_t)(a) >> 16),             \
      0xff & ((uint32_t)(a) >> 24),             \
    },                                          \
      }

/**
   @this is an shorthand initializer for a Bluetooth-based UUID
   object.  It can be used to initialize a constant structure.

   The following code defines UUID @tt
   {00001812-0000-1000-8000-00805f9b34fb}:
   @code
   static const struct ble_uuid_s ble_hid_service_type = BLE_UUID_BT_BASED(0x1812);
   @end code
 */
#define BLE_UUID_BT_BASED(x) BLE_UUID(x, 0, 0x1000, 0x8000, 0x805f9b34fbULL)

#define BLE_UUID_TO_P(x) (&(const struct ble_uuid_s)x)

/**
   @this is an shorthand initializer for a UUID object @strong
   pointer.
 */
#define BLE_UUID_P(a, b, c, d, e) BLE_UUID_TO_P(BLE_UUID(a, b, c, d, e))

/**
   @this is an shorthand initializer for a Bluetooth-based UUID
   object.  It can be used to initialize a constant structure.

   The following code defines a pointer to UUID @tt
   {00001812-0000-1000-8000-00805f9b34fb}:
   @code
   static const struct ble_uuid_s *ble_hid_service_type_ptr = BLE_UUID_BT_BASED_P(0x1812);
   @end code

   @see ble_uuid_bluetooth_based
 */
#define BLE_UUID_BT_BASED_P(x) BLE_UUID_TO_P(BLE_UUID_BT_BASED(x))

/**
   @this is a format string for printing a UUID.  It should be used
   with @ref {#BLE_UUID_ARG}.

   @code
   struct ble_uuid_s *type = ...;
   printk("Service type UUID: " BLE_UUID_FMT "\n", BLE_UUID_ARG(type));
   @end code
 */
#define BLE_UUID_FMT "%08x-%04x-%04x-%04x-%04x%08x"

/** @see {#BLE_UUID_FMT} */
#define BLE_UUID_ARG(x)                         \
  endian_le32((x)->value32[3]),                 \
  endian_le16((x)->value16[5]),                 \
  endian_le16((x)->value16[4]),                 \
  endian_le16((x)->value16[3]),                 \
  endian_le16((x)->value16[2]),                 \
  endian_le32((x)->value32[0])

/**
   Bluetooth base UUID
 */
extern const struct ble_uuid_s bluetooth_base_uuid;

/**
   @this checks whether passed UUID is based on bluetooth base UUID.
   If so, it may be shortened to either UUID16 or UUID32.
 */
bool_t ble_uuid_is_bluetooth_based(const struct ble_uuid_s *uuid);

/**
   @this checks whether passed UUID is based on bluetooth base UUID
   and fits a 16-bit constant.  If so, it may be shortened to a
   UUID16.
 */
bool_t ble_uuid_is_uuid16(const struct ble_uuid_s *uuid);

/**
   @this retrieves UUID16 value from a short UUID expressed in its
   128-bit form.  This does not check whether UUID is actually
   bluetooth-based.
 */
uint16_t ble_uuid_uuid16_get(const struct ble_uuid_s *uuid);

/**
   @this checks whether passed UUID is based on bluetooth base UUID
   and fits an UUID32.
 */
bool_t ble_uuid_is_uuid32(const struct ble_uuid_s *uuid);

/**
   @this retrieves UUID32 value from a short UUID expressed in its
   128-bit form.  This does not check whether UUID is actually
   bluetooth-based.
 */
uint32_t ble_uuid_uuid32_get(const struct ble_uuid_s *uuid);

/**
   @this compares two UUIDs and returns difference between their first
   differing byte.  This can be used to sort UUIDs or check for
   equality.

   @returns 0 when matching, or a difference value.
 */
int8_t ble_uuid_cmp(const struct ble_uuid_s *a, const struct ble_uuid_s *b);

/**
   @this initializes 128 bits of a UUID to be expanded form a short
   UUID (either UUID16 or UUID32).

   This is a dynamic counterpart for @ref {#BLE_UUID_BT_BASED}.
 */
void ble_uuid_bluetooth_based(struct ble_uuid_s *uuid, uint32_t short_uuid);

#endif
