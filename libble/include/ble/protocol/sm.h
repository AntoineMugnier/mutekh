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

#ifndef BLE_PROTOCOL_SM_H_
#define BLE_PROTOCOL_SM_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for Security Manager layer
 */

enum sm_command
{
  BLE_SM_PAIRING_REQUEST = 0x01,
  BLE_SM_PAIRING_RESPONSE = 0x02,
  BLE_SM_PAIRING_CONFIRM = 0x03,
  BLE_SM_PAIRING_RANDOM = 0x04,
  BLE_SM_PAIRING_FAILED = 0x05,
  BLE_SM_ENCRYPTION_INFORMATION = 0x06,
  BLE_SM_MASTER_IDENTIFICATION = 0x07,
  BLE_SM_IDENTITY_INFORMATION = 0x08,
  BLE_SM_IDENTITY_ADDRESS_INFORMATION = 0x09,
  BLE_SM_SIGNING_INFORMATION = 0x0A,
  BLE_SM_SECURITY_REQUEST = 0x0B,
};

enum sm_io_cap
{
  BLE_SM_IO_CAP_DISPLAY_ONLY = 0,
  BLE_SM_IO_CAP_DISPLAY_YES_NO = 1,
  BLE_SM_IO_CAP_KEYBOARD_ONLY = 2,
  BLE_SM_IO_CAP_NO_INPUT_NO_OUTPUT = 3,
  BLE_SM_IO_CAP_KEYBOARD_DISPLAY = 4,
};

enum sm_bonding
{
  BLE_SM_REQ_BONDING_MASK = 0x3,
  BLE_SM_REQ_BONDING = 0x1,
  BLE_SM_REQ_MITM = 0x4,
  BLE_SM_REQ_SC = 0x8,
  BLE_SM_REQ_KEYPRESS = 0x10,
  BLE_SM_REQ_CT2 = 0x20,
};

enum sm_keys
{
  BLE_SM_ENC_KEY = 0x1,
  BLE_SM_ID_KEY = 0x2,
  BLE_SM_SIGN_KEY = 0x4,
  BLE_SM_LINK_KEY = 0x8,
};

enum sm_reason
{
  BLE_SM_REASON_PASSKEY_ENTRY_FAILED = 1,
  BLE_SM_REASON_OOB_NOT_AVAILABLE = 2,
  BLE_SM_REASON_AUTHENTICATION_REQUIREMENTS = 3,
  BLE_SM_REASON_CONFIRM_VALUE_FAILED = 4,
  BLE_SM_REASON_PAIRING_NOT_SUPPORTED = 5,
  BLE_SM_REASON_ENCRYPTION_KEY_SIZE = 6,
  BLE_SM_REASON_COMMAND_NOT_SUPPORTED = 7,
  BLE_SM_REASON_UNSPECIFIED_REASON = 8,
  BLE_SM_REASON_REPEATED_ATTEMPTS = 9,
  BLE_SM_REASON_INVALID_PARAMETERS = 0xa,
  BLE_SM_REASON_DHKEY_CHECK_FAILED = 0xb,
  BLE_SM_REASON_NUMERIC_COMPARISON_FAILED = 0xc,
  BLE_SM_REASON_BREDR_IN_PROGRESS = 0xd,
  BLE_SM_REASON_CTKD_NOT_ALLOWED = 0xe,
};

#endif
