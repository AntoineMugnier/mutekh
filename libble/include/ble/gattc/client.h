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

#ifndef BLE_GATTC_CLIENT_H
#define BLE_GATTC_CLIENT_H

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short GATT Client context

*/

#include <hexo/types.h>
#include <ble/uuid.h>

#include <ble/gattc/registry.h>

struct ble_gattc_s
{
  ble_gattc_registry_list_root_t registry_list;
};

error_t ble_gattc_init(struct ble_gattc_s *client);

void ble_gattc_deinit(struct ble_gattc_s *client);

size_t ble_gattc_sollicitation_16_get(struct ble_gattc_s *client,
                                      void *srvlist, size_t size);

size_t ble_gattc_sollicitation_128_get(struct ble_gattc_s *client,
                                       void *srvlist, size_t size);

#endif
