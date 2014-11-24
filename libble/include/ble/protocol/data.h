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

#ifndef BLE_PROTOCOL_DATA_H_
#define BLE_PROTOCOL_DATA_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Protocol definitions for Data channel PDUs

   @this defines protocol data types for advertising.
 */

#include <mutek/buffer_pool.h>
#include <ble/protocol/address.h>

/** Protocol defined data channel packet types */
enum ble_llid {
  BLE_LL_RESERVED = 0,
  BLE_LL_DATA_CONT = 1,
  BLE_LL_DATA_START = 2,
  BLE_LL_CONTROL = 3,
};

/** @multiple Protocol defined data channel packet flags */
#define BLE_LL_DATA_SN 0x08
#define BLE_LL_DATA_NESN 0x04
#define BLE_LL_DATA_MD 0x10

/** @this printk()s a data channel packet */
void ble_data_packet_dump(const struct buffer_s *p, bool_t m2s);

/**
   @this retrieves data channel packet type
 */
ALWAYS_INLINE
enum ble_llid ble_data_llid_get(const struct buffer_s *p)
{
    return p->data[p->begin] & 0x3;
}

/**
   @this retrieves data channel packet SN
 */
ALWAYS_INLINE bool_t ble_data_sn_get(const struct buffer_s *p)
{
    return !!(p->data[p->begin] & BLE_LL_DATA_SN);
}

/**
   @this retrieves data channel packet NESN
 */
ALWAYS_INLINE bool_t ble_data_nesn_get(const struct buffer_s *p)
{
    return !!(p->data[p->begin] & BLE_LL_DATA_NESN);
}

/**
   @this retrieves data channel packet MD
 */
ALWAYS_INLINE bool_t ble_data_md_get(const struct buffer_s *p)
{
    return !!(p->data[p->begin] & BLE_LL_DATA_MD);
}

/**
   @this checks whether Access Address is valid for 6.B.2.1.2.
 */
bool_t ble_data_aa_is_valid(uint32_t aa);

/** @internal */
struct ble_conn_timing_param_s;

/**
   @this checks whether connection parameters are valid.
 */
error_t ble_conn_timing_validate(const struct ble_conn_timing_param_s *params);

#endif
