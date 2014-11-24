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

#ifndef BLE_BEACON_ADVERTISER_H_
#define BLE_BEACON_ADVERTISER_H_

/**
   @file
   @module{BLE library}
   @short BLE Beacon advertiser utility

   @section {Description}

   Utility functions that create an advertiser layer with relevant
   parameters to implement an iBeacon functionality.

   @end section
*/

#include <hexo/error.h>
#include <ble/uuid.h>
#include <ble/protocol/address.h>

struct net_layer_s;
struct ble_stack_context_s;

struct ble_beacon_config_s
{
  struct ble_addr_s local_addr;
  struct ble_uuid_s group_uuid;
  int8_t one_meter_rssi;
  uint16_t major, minor;
  uint32_t interval_ms;
};

error_t ble_beacon_create(
    struct ble_stack_context_s *context,
    const struct ble_beacon_config_s *config,
    struct net_layer_s **beaconer);

error_t ble_beacon_update(
    struct net_layer_s *beaconer,
    const struct ble_beacon_config_s *config);

#endif
