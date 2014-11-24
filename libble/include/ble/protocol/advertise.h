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

#ifndef BLE_PROTOCOL_ADVERTISE_H_
#define BLE_PROTOCOL_ADVERTISE_H_

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

#define BLE_ADVERTISE_AA 0x8e89bed6
#define BLE_ADVERTISE_CRCINIT 0x555555

/**
   @this contains all connection request's parameters.
 */
struct ble_adv_connect_s
{
  struct ble_addr_s master;
  struct ble_addr_s slave;
  struct ble_conn_timing_param_s timing;
  uint64_t channel_map;
  uint32_t access_address;
  uint32_t crc_init;
  uint16_t win_offset;
  uint8_t hop;
  uint8_t sca;
  uint8_t win_size;
};

/** Protocol defined advertising packet types */
enum ble_adv_type
{
    BLE_ADV_IND,
    BLE_ADV_DIRECT_IND,
    BLE_ADV_NONCONN_IND,
    BLE_SCAN_REQ,
    BLE_SCAN_RSP,
    BLE_CONNECT_REQ,
    BLE_ADV_SCAN_IND,
    BLE_ADV_TYPE_LAST = BLE_ADV_SCAN_IND,
};

#define BLE_ADV_TXADD 0x40
#define BLE_ADV_RXADD 0x80
#define BLE_ADV_TYPE_MASK 0xf

#define BLE_ADV_TYPE_OFFSET 0
#define BLE_ADV_LEN_OFFSET 1
#define BLE_ADV_TXADDR_OFFSET 2
#define BLE_ADV_RXADDR_OFFSET 8

/** String table matching advertising type for pretty print */
extern const char *ble_adv_type_name[BLE_ADV_TYPE_LAST + 1];

enum ble_sca_e {
  BLE_SCA_251_500_PPM = 0,
  BLE_SCA_151_250_PPM = 1,
  BLE_SCA_101_150_PPM = 2,
  BLE_SCA_76_100_PPM = 3,
  BLE_SCA_51_75_PPM = 4,
  BLE_SCA_31_50_PPM = 5,
  BLE_SCA_21_30_PPM = 6,
  BLE_SCA_0_20_PPM = 7,
};

/** Sleep clock accuracy worst case PPM, indexed by SCA constant */
extern const uint16_t ble_sca_wc[8];
/** Sleep clock accuracy best case PPM, indexed by SCA constant */
extern const uint16_t ble_sca_bc[8];

struct dev_freq_s;

/**
   @this computes a BLE sleep clock accuracy constant from libdevice
   timer accuracy.
 */
enum ble_sca_e ble_sca_from_accuracy(const struct dev_freq_s *freq);

/**
   @this printk() an advertise packet.
*/
void ble_advertise_packet_dump(const struct buffer_s *p);

/**
   @this retrieves advertise packet type from payload.
*/
ALWAYS_INLINE
enum ble_adv_type ble_advertise_packet_type_get(const struct buffer_s *p)
{
    return p->data[p->begin] & BLE_ADV_TYPE_MASK;
}

/**
   @this retrieves advertise source address from payload
*/
ALWAYS_INLINE
void ble_advertise_packet_txaddr_get(const struct buffer_s *p, struct ble_addr_s *addr)
{
  const uint8_t *data = p->data + p->begin;

  addr->type = data[BLE_ADV_TYPE_OFFSET] & BLE_ADV_TXADD ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
  memcpy(addr->addr, &data[BLE_ADV_TXADDR_OFFSET], 6);
}

/**
   @this retrieves advertise target address from payload
*/
ALWAYS_INLINE
void ble_advertise_packet_rxaddr_get(const struct buffer_s *p, struct ble_addr_s *addr)
{
  const uint8_t *data = p->data + p->begin;

  addr->type = data[BLE_ADV_TYPE_OFFSET] & BLE_ADV_RXADD ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
  memcpy(addr->addr, &data[BLE_ADV_RXADDR_OFFSET], 6);
}

/**
   @this sets advertise source address to payload
*/
ALWAYS_INLINE
void ble_advertise_packet_txaddr_set(struct buffer_s *p, const struct ble_addr_s *addr)
{
  uint8_t *data = p->data + p->begin;

  if (addr->type == BLE_ADDR_RANDOM)
    data[BLE_ADV_TYPE_OFFSET] |= BLE_ADV_TXADD;
  else
    data[BLE_ADV_TYPE_OFFSET] &= ~BLE_ADV_TXADD;
  memcpy(&data[BLE_ADV_TXADDR_OFFSET], addr->addr, 6);
}

/**
   @this sets advertise target address to payload
*/
ALWAYS_INLINE
void ble_advertise_packet_rxaddr_set(struct buffer_s *p, const struct ble_addr_s *addr)
{
  uint8_t *data = p->data + p->begin;

  if (addr->type == BLE_ADDR_RANDOM)
    data[BLE_ADV_TYPE_OFFSET] |= BLE_ADV_RXADD;
  else
    data[BLE_ADV_TYPE_OFFSET] &= ~BLE_ADV_RXADD;
  memcpy(&data[BLE_ADV_RXADDR_OFFSET], addr->addr, 6);
}

/**
   @this initializes an ADV_IND packet with no AD PDU.
   @param adva Advertise packet source address
*/
void ble_adv_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva);

/**
   @this initializes an ADV_DIRECT_IND packet.
   @param adva Advertise packet source address
   @param inita Advertise packet target address
*/
void ble_adv_direct_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva,
    const struct ble_addr_s *inita);

/**
   @this initializes an ADV_NONCONN_IND packet with no AD PDU.
   @param adva Advertise packet source address
*/
void ble_adv_nonconn_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva);

/**
   @this initializes an SCAN_REQ packet.
   @param scana Packet source address
   @param adva Packet target address
*/
void ble_adv_scan_req_set(
    struct buffer_s *p,
    const struct ble_addr_s *scana,
    const struct ble_addr_s *adva);

/**
   @this initializes an SCAN_RSP packet with no AD PDU.
   @param adva Packet source address
*/
void ble_adv_scan_rsp_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva);

/**
   @this initializes an SCAN_IND packet.
   @param adva Packet source address
*/
void ble_adv_scan_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva);

/**
   @this initializes a CONNECT_REQ packet.
   @param params Connection request parameters
*/
void ble_adv_connect_set(
    struct buffer_s *p,
    const struct ble_adv_connect_s *params);

/**
   @this appends some AD PDU to ADV_IND or SCAN_RSP packet.
*/
error_t ble_adv_data_append(
    struct buffer_s *p,
    const uint8_t type,
    const void *data, const uint8_t size);

/**
   @this parses a CONNECT_REQ packet.
   @param conn Connection request parameters
*/
error_t ble_adv_connect_parse(
    const struct buffer_s *p,
    struct ble_adv_connect_s *conn);

/**
   @this appends a string PDU to ADV_IND or SCAN_RSP packet.
*/
ALWAYS_INLINE
error_t ble_adv_data_string_append(
    struct buffer_s *p,
    const uint8_t type,
    const char *s)
{
  return ble_adv_data_append(p, type, s, strlen(s));
}

#endif
