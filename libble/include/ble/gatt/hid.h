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

#ifndef BLE_GATT_SERVICE_HID_H
#define BLE_GATT_SERVICE_HID_H

#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/descriptor.h>

#define BLE_HID_EXTERNAL_REPORT_REFERENCE(type_)            \
    {                                                       \
      .type = BLE_UUID_BT_BASED_P(BLE_GATT_DESC_EXTERNAL_REPORT_REFERENCE), \
        .data = (const uint8_t[]){ (type_) & 0xff, (type_) >> 8 },      \
        .size = 2,                                          \
    }

#define BLE_HID_REPORT_REFERENCE(type_, id_)                \
    {                                                       \
        .type = BLE_UUID_BT_BASED_P(BLE_GATT_DESC_REPORT_REFERENCE),       \
        .data = (const uint8_t[]){ id_, type_ },            \
        .size = 2,                                          \
    }

#define BLE_HID_INPUT_REPORT(id) BLE_HID_REPORT_REFERENCE(1, id)
#define BLE_HID_OUTPUT_REPORT(id) BLE_HID_REPORT_REFERENCE(2, id)
#define BLE_HID_FEATURE_REPORT(id) BLE_HID_REPORT_REFERENCE(3, id)

#endif
