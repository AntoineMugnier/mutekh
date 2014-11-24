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

#include <mutek/buffer_pool.h>
#include "address.h"

enum ble_llid {
  BLE_LL_RESERVED = 0,
  BLE_LL_DATA_CONT = 1,
  BLE_LL_DATA_START = 2,
  BLE_LL_CONTROL = 3,
};

#define BLE_LL_DATA_SN 0x08
#define BLE_LL_DATA_NESN 0x04
#define BLE_LL_DATA_MD 0x10

enum ble_version
{
  BLE_LL_VERSION_1_0b = 0,
  BLE_LL_VERSION_1_1 = 1,
  BLE_LL_VERSION_1_2 = 2,
  BLE_LL_VERSION_2_0_EDR = 3,
  BLE_LL_VERSION_2_1_EDR = 4,
  BLE_LL_VERSION_3_0_HS = 5,
  BLE_LL_VERSION_4_0 = 6,
  BLE_LL_VERSION_4_1 = 7,
  BLE_LL_VERSION_4_2 = 8,
};

enum ble_feature
{
  BLE_LL_VERSION_LE_ENCRYPTION = 0,
  BLE_LL_VERSION_CONNECTION_PARAMETERS_REQUEST_PROCEDURE = 1,
  BLE_LL_VERSION_EXTENDED_REJECT_INDICATION = 2,
  BLE_LL_VERSION_SLAVE_INITIATED_FEATURES_EXCHANGE = 3,
  BLE_LL_VERSION_LE_PING = 4,
  BLE_LL_VERSION_LE_DATA_PACKET_LENGTH_EXTENSION = 5,
  BLE_LL_VERSION_LL_PRIVACY = 6,
  BLE_LL_VERSION_EXTENDED_SCANNER_FILTER_POLICIES = 7,
};

enum ble_ll_control_type
{
  BLE_LL_CONNECTION_UPDATE_REQ = 0x00,
  BLE_LL_CHANNEL_MAP_REQ = 0x01,
  BLE_LL_TERMINATE_IND = 0x02,
  BLE_LL_ENC_REQ = 0x03,
  BLE_LL_ENC_RSP = 0x04,
  BLE_LL_START_ENC_REQ = 0x05,
  BLE_LL_START_ENC_RSP = 0x06,
  BLE_LL_UNKNOWN_RSP = 0x07,
  BLE_LL_FEATURE_REQ = 0x08,
  BLE_LL_FEATURE_RSP = 0x09,
  BLE_LL_PAUSE_ENC_REQ = 0x0A,
  BLE_LL_PAUSE_ENC_RSP = 0x0B,
  BLE_LL_VERSION_IND = 0x0C,
  BLE_LL_REJECT_IND = 0x0D,
  BLE_LL_SLAVE_FEATURE_REQ = 0x0E,
  BLE_LL_CONNECTION_PARAM_REQ = 0x0F,
  BLE_LL_CONNECTION_PARAM_RSP = 0x10,
  BLE_LL_REJECT_IND_EXT = 0x11,
  BLE_LL_PING_REQ = 0x12,
  BLE_LL_PING_RSP = 0x13,
  BLE_LL_LENGTH_REQ = 0x14,
  BLE_LL_LENGTH_RSP = 0x15,
};

struct ble_conn_timing_param_s
{
  uint16_t interval;
  uint16_t latency;
  uint16_t timeout;
  uint8_t win_size;
};

struct ble_conn_params_update
{
  struct ble_conn_timing_param_s timing;
  uint16_t win_offset;
  uint16_t instant;
};

void ble_data_packet_dump(const struct buffer_s *p, bool_t m2s);
void ble_control_packet_dump(const struct buffer_s *p);

void ble_data_conn_params_update_parse(const struct buffer_s *p,
                                       struct ble_conn_params_update *cpu);

void ble_conn_channel_mapping_expand(uint8_t *channel_remap, uint64_t channel_map);

ALWAYS_INLINE
enum ble_llid ble_data_llid_get(const struct buffer_s *p)
{
    return p->data[p->begin] & 0x3;
}

ALWAYS_INLINE
enum ble_ll_control_type ble_data_control_get(const struct buffer_s *p)
{
    return p->data[p->begin + 2];
}

ALWAYS_INLINE bool_t ble_data_sn_get(const struct buffer_s *p)
{
    return !!(p->data[p->begin] & BLE_LL_DATA_SN);
}

ALWAYS_INLINE bool_t ble_data_nesn_get(const struct buffer_s *p)
{
    return !!(p->data[p->begin] & BLE_LL_DATA_NESN);
}

ALWAYS_INLINE bool_t ble_data_md_get(const struct buffer_s *p)
{
    return !!(p->data[p->begin] & BLE_LL_DATA_MD);
}

#endif
