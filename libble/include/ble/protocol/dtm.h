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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2020
*/

#ifndef BLE_PROTOCOL_DTM_H_
#define BLE_PROTOCOL_DTM_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for Advertising channel PDUs

   @this defines protocol data types for advertising.
 */

#include <mutek/buffer_pool.h>
#include <string.h>
#include <ble/protocol/address.h>
#include <ble/protocol/llcp.h>

#define BLE_DTM_AA ~0x8e89bed6
#define BLE_DTM_CRCINIT 0x555555

enum ble_dtm_pattern_e {
    BLE_DTM_PATTERN_PRBS9,
    BLE_DTM_PATTERN_0F,
    BLE_DTM_PATTERN_55,
    BLE_DTM_PATTERN_PRBS15,
    BLE_DTM_PATTERN_FF,
    BLE_DTM_PATTERN_00,
    BLE_DTM_PATTERN_F0,
    BLE_DTM_PATTERN_AA,
};

/**
   @this fills a PDU (including packet type and size header bytes) of
   a DTM packet.  Buffer will be filled with a 2-byte header and a
   payload of @tt{size - 2} bytes of pattern @tt pattern.

   @param pattern Pattend to fill in
   @param buffer Target buffer of at least @tt size bytes
   @param size Buffer size
 */
void ble_dtm_pdu_fill(enum ble_dtm_pattern_e pattern, uint8_t *buffer, size_t size);

/**
   @this fills in a @tt buffer with @tt size bytes of PRBS9, starting with
   state @tt state.
   @returns state for continuation
*/
uint16_t ble_dtm_prbs9_generate(uint16_t state, uint8_t *buffer, size_t size);

/**
   @this fills in a @tt buffer with @tt size bytes of PRBS15, starting with
   state @tt state.
   @returns state for continuation
*/
uint16_t ble_dtm_prbs15_generate(uint16_t state, uint8_t *buffer, size_t size);

#endif
