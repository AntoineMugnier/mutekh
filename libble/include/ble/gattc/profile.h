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
   @module{BLE library}
   @short GATT Client Service declaration

   @section {Description}

   A GATT Client declares service it expects and which characteristics
   should be looked up for in service.  This header defines
   declaration of one expected service.

   @end section
*/

#include <hexo/types.h>
#include <ble/uuid.h>

struct ble_gattc_registry_s;

enum ble_gattc_descriptor_flag_e
{
  BLE_GATTC_DESCRIPTOR_REQUIRED = 1,
  BLE_GATTC_DESCRIPTOR_READ = 2,
};

struct ble_gattc_descriptor_s
{
  const struct ble_uuid_s *type;
  uint8_t flags;
};

enum ble_gattc_characteristic_flag_e
{
  BLE_GATTC_CHARACTERISTIC_REQUIRED = 1,
  BLE_GATTC_CHARACTERISTIC_UNIQUE = 2,
  BLE_GATTC_CHARACTERISTIC_READ = 4,
};

enum ble_gattc_data_origin_e
{
  BLE_GATTC_DATA_READ,
  BLE_GATTC_DATA_NOTIFICATION,
  BLE_GATTC_DATA_INDICATION,
};

struct ble_gattc_characteristic_s
{
  const struct ble_uuid_s *type;
  const struct ble_uuid_s *descriptor;
  uint8_t descriptor_count;
  uint8_t flags;

  void (*on_discovered)(struct ble_gattc_registry_s *reg,
                        uint8_t service_index,
                        uint8_t count);

  void (*on_data)(struct ble_gattc_registry_s *reg,
                  uint8_t service_index,
                  uint8_t char_index,
                  enum ble_gattc_data_origin_e origin,
                  const void *data, size_t size);

  void (*on_descriptor_data)(struct ble_gattc_registry_s *reg,
                             uint8_t service_index,
                             uint8_t char_index,
                             uint8_t descriptor_index,
                             const void *data, size_t size);
};

enum ble_gattc_service_flags_e
{
  BLE_GATTC_SERVICE_REQUIRED = 1,
  BLE_GATTC_SERVICE_UNIQUE = 2,
  BLE_GATTC_SERVICE_PRIMARY = 4,
  BLE_GATTC_SERVICE_SOLLICIT = 8,
};

struct ble_gattc_service_s
{
  const struct ble_uuid_s *type;
  const struct ble_gattc_characteristic_s *characteristic;
  uint8_t characteristic_count;
  uint8_t flags;

  void (*on_discovered)(struct ble_gattc_registry_s *reg,
                        uint8_t index);
};

#define BLE_GATTC_SERVICE_DECL(name_, flags_, type_, chars_...)         \
  const struct ble_gattc_service_s name_ = {                            \
    .type = type_,                                                      \
    .characteristic = (const struct ble_gattc_characteristic_s[]){ chars_ }, \
    .characteristic_count = ARRAY_SIZE(((const struct ble_gattc_characteristic_s[]){ chars_ })), \
    .flags = flags_,                                                    \
  }

#define BLE_GATTC_CHAR(type_, perms_, desc_...)                         \
  {                                                                     \
    .type = (type_),                                                    \
    .descriptor = (const struct ble_gattc_descriptor_s[]){ chars_ },    \
    .descriptor_count = ARRAY_SIZE(((const struct ble_gattc_descriptor_s[]){ chars_ })), \
    .flags = flags_,                                                    \
  }

#endif
