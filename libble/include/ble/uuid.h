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
   @module{BLE library}
   @short UUID handling

   @section {Description}

   Bluetooth uses UUIDs in a special way.  There are two short-hand
   versions using a special "base UUID" as common value.  In MutekH,
   BLE library does not use shortened UUIDs in memory.  Shortening is
   only done where protocol requires so.

   This header provides declaration for the UUID structure, and some
   helpers to compare, initialize and check UUID features.

   @end section
*/

#include <hexo/types.h>

/**
   UUID object
 */
struct ble_uuid_s
{
  uint8_t value[16];
};

#define BLE_UUID(a, b, c, d, e) {               \
    .value = {                                  \
      0xff & ((uint32_t)(a) >> 24),             \
      0xff & ((uint32_t)(a) >> 16),             \
      0xff & ((uint32_t)(a) >> 8),              \
      0xff & ((uint32_t)(a)),                   \
      0xff & ((uint16_t)(b) >> 8),              \
      0xff & ((uint16_t)(b)),                   \
      0xff & ((uint16_t)(c) >> 8),              \
      0xff & ((uint16_t)(c)),                   \
      0xff & ((uint16_t)(d) >> 8),              \
      0xff & ((uint16_t)(d)),                   \
      0xff & ((uint64_t)(e) >> 40),             \
      0xff & ((uint64_t)(e) >> 32),             \
      0xff & ((uint64_t)(e) >> 24),             \
      0xff & ((uint64_t)(e) >> 16),             \
      0xff & ((uint64_t)(e) >> 8),              \
      0xff & ((uint64_t)(e)),                   \
    },                                          \
      }

/**
   @this is an initializer for a short UUID object.  It can be used to
   initialize a constant structure.

   @code
   static const struct ble_uuid_s ble_hid_service_type = BLE_UUID_SHORT(0x1812);
   @end code
 */
#define BLE_UUID_SHORT(x) BLE_UUID(x, 0, 0x1000, 0x8000, 0x805f9b34fbULL)

#define BLE_UUID_P(a, b, c, d, e) (&(const struct ble_uuid_s)BLE_UUID(a, b, c, d, e))
#define BLE_UUID_SHORT_P(x) (&(const struct ble_uuid_s)BLE_UUID_SHORT(x))

/**
   @this is a format string for printing a UUID.  It may be used with
   @ref BLE_UUID_ARG.

   @code
   struct ble_uuid_s *type = ...;
   printk("Service type UUID: " BLE_UUID_FMT "\n", BLE_UUID_ARG(type));
   @end code
 */
#define BLE_UUID_FMT "%08x-%04x-%04x-%04x-%04x%08x"

/** @see BLE_UUID_FMT */
#define BLE_UUID_ARG(x)                   \
  endian_be32_na_load((x)->value),        \
    endian_be16_na_load((x)->value + 4),  \
    endian_be16_na_load((x)->value + 6),  \
    endian_be16_na_load((x)->value + 8),  \
    endian_be16_na_load((x)->value + 10), \
    endian_be32_na_load((x)->value + 12)

/**
   Bluetooth base UUID
 */
extern const struct ble_uuid_s bluetooth_base_uuid;

/**
   @this checks whether passed UUID is based on bluetooth base UUID.
   If so, it may be shortened.
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
   128-bit form.
 */
uint16_t ble_uuid_uuid16_get(const struct ble_uuid_s *uuid);

/**
   @this checks whether passed UUID is based on bluetooth base UUID.
 */
bool_t ble_uuid_is_uuid32(const struct ble_uuid_s *uuid);

/**
   @this retrieves UUID32 value from a short UUID expressed in its
   128-bit form.
 */
uint32_t ble_uuid_uuid32_get(const struct ble_uuid_s *uuid);

/**
   @this compares two UUID and returns difference between their first
   differing nibble.  This can be used to sort UUIDs or check for
   equality.

   @returns 0 when matching, or a difference.
 */
uint8_t ble_uuid_cmp(const struct ble_uuid_s *a, const struct ble_uuid_s *b);

/**
   @this initializes 128 bits of a UUID to be expanded form for a
   short UUID (either UUID16 or UUID32).

   This is a dynamic counterpart for @ref BLE_UUID_SHORT.
 */
void ble_uuid_bluetooth_based(struct ble_uuid_s *uuid, uint32_t short_uuid);

#endif
