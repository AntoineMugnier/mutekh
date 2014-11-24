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

#ifndef BLE_ATT_H
#define BLE_ATT_H

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for Attribute protocol

   @this defines protocol data types for ATT.
 */

#include <hexo/types.h>
#include <hexo/decls.h>

/** Protocol defined ATT opcodes */
enum ble_att_opcode_e {
  BLE_ATT_ERROR_RSP                     = 0x01,
  BLE_ATT_EXCHANGE_MTU_RQT              = 0x02,
  BLE_ATT_EXCHANGE_MTU_RSP              = 0x03,
  BLE_ATT_FIND_INFORMATION_RQT          = 0x04,
  BLE_ATT_FIND_INFORMATION_RSP          = 0x05,
  BLE_ATT_FIND_BY_TYPE_VALUE_RQT        = 0x06,
  BLE_ATT_FIND_BY_TYPE_VALUE_RSP        = 0x07,
  BLE_ATT_READ_BY_TYPE_RQT              = 0x08,
  BLE_ATT_READ_BY_TYPE_RSP              = 0x09,
  BLE_ATT_READ_RQT                      = 0x0A,
  BLE_ATT_READ_RSP                      = 0x0B,
  BLE_ATT_READ_BLOB_RQT                 = 0x0C,
  BLE_ATT_READ_BLOB_RSP                 = 0x0D,
  BLE_ATT_READ_MULTIPLE_RQT             = 0x0E,
  BLE_ATT_READ_MULTIPLE_RSP             = 0x0F,
  BLE_ATT_READ_BY_GROUP_TYPE_RQT        = 0x10,
  BLE_ATT_READ_BY_GROUP_TYPE_RSP        = 0x11,
  BLE_ATT_WRITE_RQT                     = 0x12,
  BLE_ATT_WRITE_RSP                     = 0x13,
  BLE_ATT_WRITE_CMD                     = 0x52,
  BLE_ATT_PREPARE_WRITE_RQT             = 0x16,
  BLE_ATT_PREPARE_WRITE_RSP             = 0x17,
  BLE_ATT_EXECUTE_WRITE_RQT             = 0x18,
  BLE_ATT_EXECUTE_WRITE_RSP             = 0x19,
  BLE_ATT_HANDLE_VALUE_NOTIF            = 0x1B,
  BLE_ATT_HANDLE_VALUE_INDIC            = 0x1D,
  BLE_ATT_HANDLE_VALUE_CONFIRM          = 0x1E,
  BLE_ATT_SIGNED_WRITE_CMD              = 0xD2
};

/**
   @this retrieves basic ATT opcode operation from opcode, ignoring if
   it is signed or unacknowledged.
 */
ALWAYS_INLINE
uint8_t ble_att_opcode_operation(enum ble_att_opcode_e opcode)
{
  return (opcode & 0x3f);
}

/**
   @this retrieves whether ATT opcode is from client to server.
 */
ALWAYS_INLINE
bool_t ble_att_opcode_is_client_to_server(enum ble_att_opcode_e opcode)
{
  return !(opcode & 1);
}

/**
   @this retrieves whether ATT opcode implies a response.
 */
ALWAYS_INLINE
bool_t ble_att_opcode_is_response_expected(enum ble_att_opcode_e opcode)
{
  return (opcode & 0xc0) != 0x40 && opcode != BLE_ATT_HANDLE_VALUE_NOTIF;
}

/**
   @this retrieves whether ATT opcode is signed.
 */
ALWAYS_INLINE
bool_t ble_att_opcode_is_signed(enum ble_att_opcode_e opcode)
{
  return (opcode & 0xc0) == 0xc0;
}

/** Protocol defined ATT error codes */
enum ble_att_error_e {
  BLE_ATT_ERR_NONE                      = 0x00,
  BLE_ATT_ERR_INVALID_HANDLE            = 0x01,
  BLE_ATT_ERR_READ_NOT_PERMITTED        = 0x02,
  BLE_ATT_ERR_WRITE_NOT_PERMITTED       = 0x03,
  BLE_ATT_ERR_INVALID_PDU               = 0x04,
  BLE_ATT_ERR_INSUF_AUTHENTICATION      = 0x05,
  BLE_ATT_ERR_REQUEST_NOT_SUPPORTED     = 0x06,
  BLE_ATT_ERR_INVALID_OFFSET            = 0x07,
  BLE_ATT_ERR_INSUF_AUTHORIZATION       = 0x08,
  BLE_ATT_ERR_PREPARE_QUEUE_FULL        = 0x09,
  BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND       = 0x0A,
  BLE_ATT_ERR_ATTRIBUTE_NOT_LONG        = 0x0B,
  BLE_ATT_ERR_INSUF_ENCRYPT_KEY_SIZE    = 0x0C,
  BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN    = 0x0D,
  BLE_ATT_ERR_UNLIKELY_ERROR            = 0x0E,
  BLE_ATT_ERR_INSUF_ENCRYPTION          = 0x0F,
  BLE_ATT_ERR_UNSUPPORTED_GROUP_TYPE    = 0x10,
  BLE_ATT_ERR_INSUF_RESOURCES           = 0x11
};

/** Protocol defined ATT UUID formats */
enum ble_att_type_fmt_e {
  BLE_ATT_TYPE_UUID16 = 1,
  BLE_ATT_TYPE_UUID128 = 2,
};

#endif
