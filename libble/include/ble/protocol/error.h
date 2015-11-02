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

#ifndef BLE_PROTOCOL_ERROR_H
#define BLE_PROTOCOL_ERROR_H

enum ble_error_e
{
  BLE_SUCCESS = 0x00,
  BLE_UNKNOWN_HCI_COMMAND = 0x01,
  BLE_UNKNOWN_CONNECTION_IDENTIFIER = 0x02,
  BLE_HARDWARE_FAILURE = 0x03,
  BLE_PAGE_TIMEOUT = 0x04,
  BLE_AUTHENTICATION_FAILURE = 0x05,
  BLE_PIN_OR_KEY_MISSING = 0x06,
  BLE_MEMORY_CAPACITY_EXCEEDED = 0x07,
  BLE_CONNECTION_TIMEOUT = 0x08,
  BLE_CONNECTION_LIMIT_EXCEEDED = 0x09,
  BLE_SYNCHRONOUS_CONNECTION_LIMIT_TO_A_DEVICE_EXCEEDED = 0x0A,
  BLE_ACL_CONNECTION_ALREADY_EXISTS = 0x0B,
  BLE_COMMAND_DISALLOWED = 0x0C,
  BLE_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES = 0x0D,
  BLE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS = 0x0E,
  BLE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR = 0x0F,
  BLE_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED = 0x10,
  BLE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE = 0x11,
  BLE_INVALID_HCI_COMMAND_PARAMETERS = 0x12,
  BLE_REMOTE_USER_TERMINATED_CONNECTION = 0x13,
  BLE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES = 0x14,
  BLE_EMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF = 0x15,
  BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST = 0x16,
  BLE_REPEATED_ATTEMPTS = 0x17,
  BLE_PAIRING_NOT_ALLOWED = 0x18,
  BLE_UNKNOWN_LMP_PDU = 0x19,
  BLE_UNSUPPORTED_REMOTE_FEATURE = 0x1A,
  BLE_SCO_OFFSET_REJECTED = 0x1B,
  BLE_SCO_INTERVAL_REJECTED = 0x1C,
  BLE_SCO_AIR_MODE_REJECTED = 0x1D,
  BLE_INVALID_PARAMETERS = 0x1E,
  BLE_UNSPECIFIED_ERROR = 0x1F,
  BLE_UNSUPPORTED_PARAMETER_VALUE = 0x20,
  BLE_ROLE_CHANGE_NOT_ALLOWED = 0x21,
  BLE_RESPONSE_TIMEOUT = 0x22,
  BLE_LMP_ERROR_TRANSACTION_COLLISION = 0x23,
  BLE_LMP_PDU_NOT_ALLOWED = 0x24,
  BLE_ENCRYPTION_MODE_NOT_ACCEPTABLE = 0x25,
  BLE_LINK_KEY_CANNOT_BE_CHANGED = 0x26,
  BLE_REQUESTED_QOS_NOT_SUPPORTED = 0x27,
  BLE_INSTANT_PASSED = 0x28,
  BLE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED = 0x29,
  BLE_DIFFERENT_TRANSACTION_COLLISION = 0x2A,
  BLE_QOS_UNACCEPTABLE_PARAMETER = 0x2C,
  BLE_QOS_REJECTED = 0x2D,
  BLE_CHANNEL_CLASSIFICATION_NOT_SUPPORTED = 0x2E,
  BLE_INSUFFICIENT_SECURITY = 0x2F,
  BLE_PARAMETER_OUT_OF_MANDATORY_RANGE = 0x30,
  BLE_ROLE_SWITCH_PENDING = 0x32,
  BLE_RESERVED_SLOT_VIOLATION = 0x34,
  BLE_ROLE_SWITCH_FAILED = 0x35,
  BLE_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE = 0x36,
  BLE_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST = 0x37,
  BLE_HOST_BUSY_PAIRING = 0x38,
  BLE_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND = 0x39,
  BLE_CONTROLLER_BUSY = 0x3A,
  BLE_UNACCEPTABLE_CONNECTION_PARAMETERS = 0x3B,
  BLE_DIRECTED_ADVERTISING_TIMEOUT = 0x3C,
  BLE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE = 0x3D,
  BLE_CONNECTION_FAILED_TO_BE_ESTABLISHED = 0x3E,
  BLE_MAC_CONNECTION_FAILED = 0x3F,
  BLE_COARSE_CLOCK_ADJUSTMENT_REJECTED_BUT_WILL_TRY_TO_ADJUST_USING_CLOCK_DRAGGING = 0x40,
  BLE_ERROR_COUNT
};

const char *const ble_error_string[BLE_ERROR_COUNT];

#endif