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

#ifndef BLE_PROTOCOL_RADIO_H
#define BLE_PROTOCOL_RADIO_H

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for radio features
 */

/*
  Preamble: 1
  Access address: 4
  Opcode, size: 2
  Payload
  CRC: 3
 */
#define BLE_PACKET_SIZE(x) (1 + 4 + 2 + (x) + 3)
#define BLE_PACKET_TIME(x) (BLE_PACKET_SIZE(x) * 8)

#define BLE_T_IFS 150
#define BLE_T_CONN_UNIT 1250
#define BLE_T_ADV_UNIT 625

uint16_t ble_channel_freq_mhz(uint8_t chan);

enum ble_phy_mode_e {
  BLE_PHY_1M,
  BLE_PHY_2M,
  BLE_PHY_CODED2,
  BLE_PHY_CODED8,
};

#endif
