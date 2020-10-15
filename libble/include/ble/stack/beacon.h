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

#ifndef BLE_STACK_BEACON_H_
#define BLE_STACK_BEACON_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Beacon advertiser utility

   @this contains all declarations for a Beacon functionnality.
*/

#include <hexo/error.h>
#include <ble/uuid.h>
#include <ble/protocol/address.h>
#include <ble/protocol/radio.h>

struct net_layer_s;
struct ble_stack_context_s;

/**
   @this contains all data a beacon advertises for.
 */
struct ble_beacon_config_s
{
  enum ble_phy_mode_e phy;
  /** Device address the beacon uses */
  struct ble_addr_s local_addr;
  /** Beacon group */
  struct ble_uuid_s group_uuid;
  /** Indicative RSSI measured one meter apart from beacon */
  int8_t one_meter_rssi;
  /** Major/minor */
  uint16_t major, minor;
  /** Interval in milliseconds between advertise events */
  uint32_t interval_ms;
};

/**
   @this creates an advertiser layer with relevant parameters to
   implement an iBeacon functionality.

   On success, returned layer will advertise as long as it is
   referenced.  It may be destroed through @ref net_layer_refdec.

   @param beaconer Layer reference (out)
   @returns 0 on success, or an error
*/
error_t ble_beacon_create(
    struct ble_stack_context_s *context,
    const struct ble_beacon_config_s *config,
    struct net_layer_s **beaconer);

/**
   @this updates configuration of beacon layer.
*/
error_t ble_beacon_update(
    struct net_layer_s *beaconer,
    const struct ble_beacon_config_s *config);

#endif
