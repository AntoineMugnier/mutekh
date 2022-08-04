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

#ifndef BLE_PROTOCOL_SIGNALLING_H_
#define BLE_PROTOCOL_SIGNALLING_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for L2CAP Signalling layer
 */

enum ble_signaling_command
{
  BLE_SIGNALLING_COMMAND_REJ = 0x01,
  BLE_SIGNALLING_DISCONNECT_REQ = 0x06,
  BLE_SIGNALLING_DISCONNECT_RSP = 0x07,
  BLE_SIGNALLING_CONN_PARAMS_UPDATE_REQ = 0x12,
  BLE_SIGNALLING_CONN_PARAMS_UPDATE_RSP = 0x13,
  BLE_SIGNALLING_LE_CREDIT_BASED_COMM_REQ = 0x14,
  BLE_SIGNALLING_LE_CREDIT_BASED_COMM_RSP = 0x15,
  BLE_SIGNALLING_LE_FLOW_CONTROL_CREDIT = 0x15,
};

#endif
