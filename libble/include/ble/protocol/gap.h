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
  BLE_GAP_UUID128_SERVICE_LIST_COMPLETE = 0x07,
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
  BLE_GAP_UUID32_SERVICE_SOLICITATION_LIST = 0x1f,
  BLE_GAP_SERVICE_DATA_UUID32 = 0x20,
  BLE_GAP_SERVICE_DATA_UUID128 = 0x21,
  BLE_GAP_LE_SC_CONFIRMATION_VALUE = 0x22,
  BLE_GAP_LE_SC_RANDOM_VALUE = 0x23,
  BLE_GAP_URI = 0x24,
  BLE_GAP_3D_INFORMATION_DATA = 0x3D,
  BLE_GAP_MANUFACTURER_SPECIFIC_DATA = 0xFF,
};

enum {
  BLE_GAP_FLAGS_LIMITED_ADV         = 0x01,
  BLE_GAP_FLAGS_GENERAL_ADV         = 0x02,
  BLE_GAP_FLAGS_BREDR_NOT_SUPPORTED = 0x04,
};

enum {
  BLE_GAP_APPEARANCE_UNKNOWN                 = 0x0000,
  BLE_GAP_APPEARANCE_GENERIC_PHONE           = 0x0040,
  BLE_GAP_APPEARANCE_GENERIC_COMPUTER        = 0x0080,
  BLE_GAP_APPEARANCE_GENERIC_WATCH           = 0x00C0,
  BLE_GAP_APPEARANCE_WATCH_SPORTS            = 0x00C1,
  BLE_GAP_APPEARANCE_GENERIC_CLOCK           = 0x0100,
  BLE_GAP_APPEARANCE_GENERIC_DISPLAY         = 0x0140,
  BLE_GAP_APPEARANCE_GENERIC_RC              = 0x0180,
  BLE_GAP_APPEARANCE_GENERIC_EYE_GALSSES     = 0x01C0,
  BLE_GAP_APPEARANCE_GENERIC_TAG             = 0x0200,
  BLE_GAP_APPEARANCE_GENERIC_KEYRING         = 0x0240,
  BLE_GAP_APPEARANCE_GENERIC_MEDIA_PLAYER    = 0x0280,
  BLE_GAP_APPEARANCE_GENERIC_BARCODE_SCANNER = 0x02C0,
  BLE_GAP_APPEARANCE_GENERIC_THERMOMETER     = 0x0300,
  BLE_GAP_APPEARANCE_GENERIC_THERMO_EAR      = 0x0301,
  BLE_GAP_APPEARANCE_GENERIC_HR_SENSOR       = 0x0340,
  BLE_GAP_APPEARANCE_GENERIC_HRS_BELT        = 0x0341,
  BLE_GAP_APPEARANCE_GENERIC_BLOOD_PRESSURE  = 0x0380,
  BLE_GAP_APPEARANCE_GENERIC_BP_ARM          = 0x0381,
  BLE_GAP_APPEARANCE_GENERIC_BP_WRIST        = 0x0382,
  BLE_GAP_APPEARANCE_GENERIC_HID             = 0x03C0,
  BLE_GAP_APPEARANCE_HID_KEYBOARD            = 0x03C1,
  BLE_GAP_APPEARANCE_HID_MOUSE               = 0x03C2,
  BLE_GAP_APPEARANCE_HID_JOYSTICK            = 0x03C3,
  BLE_GAP_APPEARANCE_HID_GAMEPAD             = 0x03C4,
  BLE_GAP_APPEARANCE_HID_DIGITIZER_TYABLET   = 0x03C5,
  BLE_GAP_APPEARANCE_HID_DIGITAL_CARDREADER  = 0x03C6,
  BLE_GAP_APPEARANCE_HID_DIGITAL_PEN         = 0x03C7,
  BLE_GAP_APPEARANCE_HID_BARCODE_SCANNER     = 0x03C8,
};

#endif