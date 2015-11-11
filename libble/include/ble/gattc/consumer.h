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

#ifndef BLE_GATTC_CONSUMER_H
#define BLE_GATTC_CONSUMER_H

/**
   @file
   @module{BLE library}
   @short GATT Service consumer declaration

   @section {Description}

   A GATT Client declares a consumer.  A Consumer expects a service
   and which characteristics.

   @end section
*/

#include <hexo/types.h>
#include <ble/uuid.h>

struct ble_gattc_consumer_s;

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
};

enum ble_gattc_service_flags_e
{
  BLE_GATTC_SERVICE_REQUIRED = 1,
  BLE_GATTC_SERVICE_UNIQUE = 2,
  BLE_GATTC_SERVICE_PRIMARY = 4,
  BLE_GATTC_SERVICE_SOLLICIT = 8,
};

enum ble_gattc_service_state_e
{
  BLE_GATTC_SERVICE_DISCONNECTED,
  BLE_GATTC_SERVICE_ENUMERATING,
  BLE_GATTC_SERVICE_READY,
  BLE_GATTC_SERVICE_FAILED,
};

struct ble_gattc_service_s
{
  const struct ble_uuid_s *type;
  const struct ble_gattc_characteristic_s *characteristic;
  uint8_t characteristic_count;
  uint8_t flags;

  void (*on_state_changed)(struct ble_gattc_consumer_s *cons,
                           enum ble_gattc_service_state_e state);

  void (*on_characteristic_updated)(struct ble_gattc_consumer_s *cons,
                                    uint8_t char_index,
                                    uint8_t instance_index,
                                    const uint8_t *value, size_t size);
};

#define BLE_GATTC_SERVICE_DECL(name_, flags_, type_, chars_...)         \
  const struct ble_gattc_service_s name_ = {                            \
    .type = type_,                                                      \
    .characteristic = (const struct ble_gattc_characteristic_s[]){ chars_ }, \
    .characteristic_count = ARRAY_SIZE(((const struct ble_gattc_characteristic_s[]){ chars_ })), \
    .flags = flags_,                                                    \
  }

#define BLE_GATTC_CHAR_DECL(type_, flags_, desc_...)                    \
  {                                                                     \
    .type = (type_),                                                    \
    .descriptor = (const struct ble_gattc_descriptor_s[]){ desc_ },     \
    .descriptor_count = ARRAY_SIZE(((const struct ble_gattc_descriptor_s[]){ desc_ })), \
    .flags = flags_,                                                    \
  }

struct ble_gattc_characteristic_instance_s
{
  uint16_t characteristic_handle;
  uint16_t end_group_handle;
  uint16_t cccd_handle;
  // Indexed by descriptor table of char decl
  uint16_t *desc;
};

struct ble_gattc_characteristic_exp_s
{
  struct ble_gattc_characteristic_instance_s *instance;
  size_t instance_count;
};

struct ble_gattc_service_instance_s
{
  uint16_t service_handle;
  uint16_t end_group_handle;
  // Indexed by char table of service decl
  struct ble_gattc_characteristic_exp_s characteristic[];
};

struct ble_gattc_consumer_s
{
  
  const struct ble_gattc_service_s *decl;
  struct ble_gattc_service_instance_s *service;
  size_t service_count;
};

#endif
