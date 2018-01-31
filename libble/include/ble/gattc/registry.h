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

#ifndef BLE_GATTC_REGISTRY_H
#define BLE_GATTC_REGISTRY_H

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short GATT Client registry

   Registry of a service in a GATT client.
*/

#include <hexo/types.h>
#include <ble/uuid.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/container_clist.h>

struct ble_gattc_s;

#define GCT_CONTAINER_ALGO_ble_gattc_registry_list CLIST

struct ble_gattc_char_inst_s
{
  /** From service Attribute */
  uint8_t handle_base_offset;
  /** In descriptor char array */
  uint8_t char_index;
  /** Total handle count in char */
  uint8_t handle_count : 4;
  /** Offset between Characteristic Attribute and CCCD Attribute
      handles */
  uint8_t cccd_offset : 4;
  /** Instance of characteristic at index @tt char_index */
  uint8_t instance_index : 4;
};

struct ble_gattc_include_inst_s
{
  struct ble_uuid_s type;
  uint16_t handle;
};

struct ble_gattc_service_inst_s
{
  uint16_t service_handle;
  uint16_t end_group_handle;
  struct ble_gattc_char_inst_s *characteristic;
  size_t characteristic_count;
  struct ble_gattc_include_inst_s *include;
  size_t include_count;
};

struct ble_gattc_registry_s
{
  GCT_CONTAINER_ENTRY(ble_gattc_registry_list, item);

  const struct ble_gattc_service_s *desc;
  struct ble_gattc_service_inst_s *service;
  size_t service_count;
};

GCT_CONTAINER_TYPES(ble_gattc_registry_list, struct ble_gattc_registry_s *, item);

GCT_CONTAINER_FCNS(ble_gattc_registry_list, ALWAYS_INLINE, ble_gattc_registry_list,
                   init, destroy, pushback, remove, isempty, head, tail, next, prev);

error_t ble_gattc_char_read(struct ble_gattc_registry_s *reg,
                            uint8_t srv_instance, uint8_t char_index,
                            uint8_t char_instance);

error_t ble_gattc_char_write(struct ble_gattc_registry_s *reg,
                             uint8_t srv_instance, uint8_t char_index,
                             uint8_t char_instance,
                             const void *data, size_t size,
                             enum ble_gattc_subscription_mode_e mode);

error_t ble_gattc_register(struct ble_gattc_registry_s *reg,
                           struct ble_gattc_s *client,
                           const struct ble_gattc_service_s *srv);

error_t ble_gattc_unregister(struct ble_gattc_registry_s *reg);

#endif
