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

   This utility uses a BLE_Radio device to implement a beacon.  All
   that has to be passed to this utility is a beacon configuration
   structure.

   Beaconing will never stop unless destroyed.

   @end section
*/

#include <hexo/error.h>
#include <ble/beacon/config.h>

struct ble_beacon_advertiser_s;

error_t ble_beacon_create(
    const char *ble_dev_name,
    const struct ble_beacon_config_s *config,
    struct ble_beacon_advertiser_s **beacon);

void ble_beacon_destroy(struct ble_beacon_advertiser_s *beacon);

#endif
