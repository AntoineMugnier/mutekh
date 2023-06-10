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

#ifndef BLE_GATTC_SERVICE_H
#define BLE_GATTC_SERVICE_H

/**
   @file
   @module {Bluetooth Low Energy library}
   @short GATT Client Service description

   A GATT Client must provide service declarations for each registry
   entry. This headers defined data structures for static declaration
   of services to look after.
*/

#include <hexo/types.h>
#include <ble/uuid.h>

struct ble_gattc_registry_s;

struct ble_gattc_characteristic_s;
struct ble_gattc_descriptor_s;
struct ble_gattc_service_s

enum ble_gattc_service_flags_e
{
  BLE_GATTC_SERVICE_PRIMARY = 1,
  BLE_GATTC_SERVICE_SOLLICIT = 2,
};

struct ble_gattc_descriptor_s
{
  const struct ble_uuid_s *type;
};

#define BLE_GATTC_DESCRIPTOR(type_) { .type = type_, }

struct ble_gattc_characteristic_s
{
  const struct ble_uuid_s *type;
  const struct ble_gattc_descriptor_s *descriptor;
  uint16_t max_size;
  uint16_t descriptor_count : 6;

  void (*on_discovery)(struct ble_gattc_registry_s *reg,
                       uint8_t descriptor_index, uint8_t instance_index,
                       uint16_t value_handle);
};

#define BLE_GATTC_CHAR(type_, flags_, desc_...)                         \
  {                                                                     \
    .type = (type_),                                                    \
    .descriptor = (const struct ble_gattc_descriptor_s[]){ desc_ },     \
    .descriptor_count = ARRAY_SIZE(((const struct ble_gattc_descriptor_s[]){ desc_ })), \
    .flags = flags_,                                                    \
  }

struct ble_gattc_service_s
{
  const struct ble_uuid_s *type;
  const struct ble_gattc_characteristic_s *characteristic;
  uint8_t characteristic_count;
  uint8_t flags;

  void (*on_error)(struct ble_gattc_registry_s *reg,
                   uint8_t char_index, uint8_t instance_index,
                   uint8_t action, uint8_t att_error);

  void (*on_enumeration_done)(struct ble_gattc_registry_s *reg);
};

#define BLE_GATTC_SERVICE(name_, flags_, type_, chars_...)              \
  const struct ble_gattc_service_s name_ = {                            \
    .type = type_,                                                      \
    .characteristic = (const struct ble_gattc_characteristic_s[]){ chars_ }, \
    .characteristic_count = ARRAY_SIZE(((const struct ble_gattc_characteristic_s[]){ chars_ })), \
    .flags = flags_,                                                    \
  }

#endif
