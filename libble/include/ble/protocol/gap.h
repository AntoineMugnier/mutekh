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

#ifndef BLE_PROTOCOL_GAP_H_
#define BLE_PROTOCOL_GAP_H_

enum {
  BLE_GAP_FLAGS = 0x01,
  BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE = 0x02,
  BLE_GAP_UUID16_SERVICE_LIST_COMPLETE = 0x03,
  BLE_GAP_UUID32_SERVICE_LIST_INCOMPLETE = 0x04,
  BLE_GAP_UUID32_SERVICE_LIST_COMPLETE = 0x05,
  BLE_GAP_UUID128_SERVICE_LIST_INCOMPLETE = 0x06,
  BLE_GAP_UUID129_SERVICE_LIST_COMPLETE = 0x07,
  BLE_GAP_SHORTENED_LOCAL_NAME = 0x08,
  BLE_GAP_COMPLETE_LOCAL_NAME = 0x09,
  BLE_GAP_TX_POWER_LEVEL = 0x0A,
  BLE_GAP_CLASS_OF_DEVICE = 0x0D,
  BLE_GAP_SIMPLE_PAIRING_HASH_C = 0x0E,
  BLE_GAP_SIMPLE_PAIRING_RANDOMIZER_R = 0x0F,
  BLE_GAP_DEVICE_ID = 0x10,
  BLE_GAP_SECURITY_MANAGER_TK_VALUE = 0x10,
  BLE_GAP_SECURITY_MANAGER_OUT_OF_BAND_FLAGS = 0x11,
  BLE_GAP_SLAVE_CONNECTION_INTERVAL_RANGE = 0x12,
  BLE_GAP_UUID16_SERVICE_SOLICITATION_LIST = 0x14,
  BLE_GAP_UUID128_SERVICE_SOLICITATION_LIST = 0x15,
  BLE_GAP_SERVICE_DATA = 0x16,
  BLE_GAP_PUBLIC_TARGET_ADDRESS = 0x17,
  BLE_GAP_RANDOM_TARGET_ADDRESS = 0x18,
  BLE_GAP_APPEARANCE = 0x19,
  BLE_GAP_ADVERTISING_INTERVAL = 0x1A,
  BLE_GAP_LE_BLUETOOTH_DEVICE_ADDRESS = 0x1B,
  BLE_GAP_LE_ROLE = 0x1C,
  BLE_GAP_SIMPLE_PAIRING_HASH_C_256 = 0x1D,
  BLE_GAP_SIMPLE_PAIRING_RANDOMIZER_R_256 = 0x1E,
  BLE_GAP_3D_INFORMATION_DATA = 0x3D,
  BLE_GAP_MANUFACTURER_SPECIFIC_DATA = 0xFF,
};

#endif
