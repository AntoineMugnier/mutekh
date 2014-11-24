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

  Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#ifndef BLE_GATT_SERVICE_GAP_H
#define BLE_GATT_SERVICE_GAP_H

#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/descriptor.h>

struct ble_gap_preferred_conn_params_s {
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t latency;
    uint16_t timeout;
};

#endif
