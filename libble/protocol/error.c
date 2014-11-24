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

#include <ble/protocol/error.h>

const char *const ble_error_string[BLE_ERROR_COUNT] =
{
  [BLE_SUCCESS] = "Success",
  [BLE_UNKNOWN_HCI_COMMAND] = "Unknown HCI Command",
  [BLE_UNKNOWN_CONNECTION_IDENTIFIER] = "Unknown Connection Identifier",
  [BLE_HARDWARE_FAILURE] = "Hardware Failure",
  [BLE_PAGE_TIMEOUT] = "Page Timeout",
  [BLE_AUTHENTICATION_FAILURE] = "Authentication Failure",
  [BLE_PIN_OR_KEY_MISSING] = "PIN or Key Missing",
  [BLE_MEMORY_CAPACITY_EXCEEDED] = "Memory Capacity Exceeded",
  [BLE_CONNECTION_TIMEOUT] = "Connection Timeout",
  [BLE_CONNECTION_LIMIT_EXCEEDED] = "Connection Limit Exceeded",
  [BLE_SYNCHRONOUS_CONNECTION_LIMIT_TO_A_DEVICE_EXCEEDED] = "Synchronous Connection Limit To A Device Exceeded",
  [BLE_ACL_CONNECTION_ALREADY_EXISTS] = "ACL Connection Already Exists",
  [BLE_COMMAND_DISALLOWED] = "Command Disallowed",
  [BLE_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES] = "Connection Rejected due to Limited Resources",
  [BLE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS] = "Connection Rejected Due To Security Reasons",
  [BLE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR] = "Connection Rejected due to Unacceptable BD_ADDR",
  [BLE_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED] = "Connection Accept Timeout Exceeded",
  [BLE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE] = "Unsupported Feature or Parameter Value",
  [BLE_INVALID_HCI_COMMAND_PARAMETERS] = "Invalid HCI Command Parameters",
  [BLE_REMOTE_USER_TERMINATED_CONNECTION] = "Remote User Terminated Connection",
  [BLE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES] = "Remote Device Terminated Connection due to Low Resources",
  [BLE_EMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF] = "Connection Terminated due to Power Off",
  [BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST] = "Connection Terminated By Local Host",
  [BLE_REPEATED_ATTEMPTS] = "Repeated Attempts",
  [BLE_PAIRING_NOT_ALLOWED] = "Pairing Not Allowed",
  [BLE_UNKNOWN_LMP_PDU] = "Unknown LMP PDU",
  [BLE_UNSUPPORTED_REMOTE_FEATURE] = "Unsupported Remote Feature",
  [BLE_SCO_OFFSET_REJECTED] = "SCO Offset Rejected",
  [BLE_SCO_INTERVAL_REJECTED] = "SCO Interval Rejected",
  [BLE_SCO_AIR_MODE_REJECTED] = "SCO Air Mode Rejected",
  [BLE_INVALID_PARAMETERS] = "Invalid LMP/LL Parameters",
  [BLE_UNSPECIFIED_ERROR] = "Unspecified Error",
  [BLE_UNSUPPORTED_PARAMETER_VALUE] = "Unsupported LMP/LL Parameter Value",
  [BLE_ROLE_CHANGE_NOT_ALLOWED] = "Role Change Not Allowed",
  [BLE_RESPONSE_TIMEOUT] = "LMP/LL Response Timeout",
  [BLE_LMP_ERROR_TRANSACTION_COLLISION] = "LMP Error Transaction Collision",
  [BLE_LMP_PDU_NOT_ALLOWED] = "LMP PDU Not Allowed",
  [BLE_ENCRYPTION_MODE_NOT_ACCEPTABLE] = "Encryption Mode Not Acceptable",
  [BLE_LINK_KEY_CANNOT_BE_CHANGED] = "Link Key cannot be Changed",
  [BLE_REQUESTED_QOS_NOT_SUPPORTED] = "Requested QoS Not Supported",
  [BLE_INSTANT_PASSED] = "Instant Passed",
  [BLE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED] = "Pairing With Unit Key Not Supported",
  [BLE_DIFFERENT_TRANSACTION_COLLISION] = "Different Transaction Collision",
  [BLE_QOS_UNACCEPTABLE_PARAMETER] = "QoS Unacceptable Parameter",
  [BLE_QOS_REJECTED] = "QoS Rejected",
  [BLE_CHANNEL_CLASSIFICATION_NOT_SUPPORTED] = "Channel Classification Not Supported",
  [BLE_INSUFFICIENT_SECURITY] = "Insufficient Security",
  [BLE_PARAMETER_OUT_OF_MANDATORY_RANGE] = "Parameter Out Of Mandatory Range",
  [BLE_ROLE_SWITCH_PENDING] = "Role Switch Pending",
  [BLE_RESERVED_SLOT_VIOLATION] = "Reserved Slot Violation",
  [BLE_ROLE_SWITCH_FAILED] = "Role Switch Failed",
  [BLE_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE] = "Extended Inquiry Response Too Large",
  [BLE_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST] = "Secure Simple Pairing Not Supported By Host",
  [BLE_HOST_BUSY_PAIRING] = "Host Busy - Pairing",
  [BLE_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND] = "Connection Rejected due to No Suitable Channel Found",
  [BLE_CONTROLLER_BUSY] = "Controller Busy",
  [BLE_UNACCEPTABLE_CONNECTION_PARAMETERS] = "Unacceptable Connection Parameters",
  [BLE_DIRECTED_ADVERTISING_TIMEOUT] = "Directed Advertising Timeout",
  [BLE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE] = "Connection Terminated due to MIC Failure",
  [BLE_CONNECTION_FAILED_TO_BE_ESTABLISHED] = "Connection Failed to be Established",
  [BLE_MAC_CONNECTION_FAILED] = "MAC Connection Failed",
  [BLE_COARSE_CLOCK_ADJUSTMENT_REJECTED_BUT_WILL_TRY_TO_ADJUST_USING_CLOCK_DRAGGING] = "Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock Dragging",
};
