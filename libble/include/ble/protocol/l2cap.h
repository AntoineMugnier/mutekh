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

#ifndef BLE_PROTOCOL_L2CAP_H_
#define BLE_PROTOCOL_L2CAP_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for L2CAP
 */

enum ble_l2cap_cid {
  BLE_L2CAP_CID_ATT = 4,
  BLE_L2CAP_CID_SIGNALLING = 5,
  BLE_L2CAP_CID_SM = 6,
};

#endif