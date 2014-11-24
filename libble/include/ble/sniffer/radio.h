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

#ifndef BLE_SNIFFER_H
#define BLE_SNIFFER_H

#include <hexo/types.h>
#include <device/request.h>

struct buffer_s;
struct buffer_pool_s;

struct ble_sniffer;

enum ble_sniffer_status
{
    BLE_SNIFFER_DONE,
    BLE_SNIFFER_INITIAL_TIMEOUT,
    BLE_SNIFFER_WINDOW_TIMEOUT,
    BLE_SNIFFER_IFS_TIMEOUT,
    BLE_SNIFFER_CANCELLED,
};

enum ble_sniffer_chain_mode
{
    BLE_SNIFFER_FREESTANDING = 0,
    BLE_SNIFFER_CHAIN_DONE = (1 << BLE_SNIFFER_DONE),
    BLE_SNIFFER_CHAIN_INITIAL_TIMEOUT = (1 << BLE_SNIFFER_INITIAL_TIMEOUT),
    BLE_SNIFFER_CHAIN_WINDOW_TIMEOUT = (1 << BLE_SNIFFER_WINDOW_TIMEOUT),
    BLE_SNIFFER_CHAIN_IFS_TIMEOUT = (1 << BLE_SNIFFER_IFS_TIMEOUT),
    BLE_SNIFFER_CHAIN_CANCELLED = (1 << BLE_SNIFFER_CANCELLED),
};

enum ble_sniffer_filter
{
    BLE_SNIFFER_FILTER_ADVADDR = 1,
    BLE_SNIFFER_FILTER_ADVIRK = 2,
    BLE_SNIFFER_FILTER_ADVPUBLIC = 4,
    BLE_SNIFFER_FILTER_ADVCONSTANT = 8,
    BLE_SNIFFER_FILTER_DATAPDU = 16,
    BLE_SNIFFER_FILTER_RSSI = 32,
};

struct ble_sniffer_request
{
    /* */
    struct dev_request_s base;

    /* Return values */
    buffer_queue_root_t queue;
    enum ble_sniffer_status status;
    uint8_t packet_count;

    /* Parameters */
    uint64_t start;
    uint64_t initial_timeout;
    uint64_t window;

    uint32_t access;
    uint32_t crc_init;
    uint32_t frequency;
    uint8_t white_iv;

    uint8_t rx_packet_max;
    uint8_t chain_mode;

    uint8_t t_ifs_max;

    /* Options */
    uint8_t filter;
    uint8_t rssi_min;
/*
    struct ble_addr_s filter_addr;
    uint8_t filter_irk[16];
*/
};

STRUCT_INHERIT(ble_sniffer_request, dev_request_s, base);

error_t ble_sniffer_create(size_t priv_size, struct ble_sniffer **radio);
void ble_sniffer_request(struct ble_sniffer *radio, ...);

struct buffer_s *ble_sniffer_packet_alloc(struct ble_sniffer *radio);

struct ble_sniffer *ble_sniffer_from_priv(void *priv);
void *ble_sniffer_to_priv(struct ble_sniffer *radio);

int64_t ble_sniffer_time_get(const struct ble_sniffer *radio);

#endif
