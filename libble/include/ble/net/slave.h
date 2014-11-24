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

#ifndef BLE_NET_SLAVE_H_
#define BLE_NET_SLAVE_H_

/**
   @file
   @module{BLE library}
   @short Slave connection layer

   @section {Description}

   This handles a connection from the slave side.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>
#include <ble/protocol/error.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>

#include <device/class/ble_radio.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>

struct buffer_s;
struct ble_slave_handler_s;
struct ble_peer_s;

#define BLE_LAYER_TYPE_SLAVE NET_LAYER_TYPE('B', 'l', 'e', 'S')

enum ble_slave_key_source_e
{
  BLE_SLAVE_KEY_SOURCE_NONE,
  BLE_SLAVE_KEY_SOURCE_STK,
  BLE_SLAVE_KEY_SOURCE_LTK,
};

/**
 BLE Slave ACL data layer.

 Handles connection data stream.

 Internally, this layer handles ACK-C PDUs (Connection control), it
 requires registration of a L2CAP layer above to handle packet
 fragmentation and L2CAP demux.
 */
struct ble_slave_s
{
  struct net_layer_s layer;

  const struct ble_slave_handler_s *handler;

  struct device_ble_radio_s radio;
  struct device_timer_s timer;
  struct device_crypto_s crypto;

  struct net_layer_s *l2cap;
  struct net_layer_s *gap;
  struct net_layer_s *security;
  struct dev_rng_s *rng;

  struct dev_ble_radio_rq_s ble_rq;
  struct dev_crypto_rq_s crypto_rq;
  struct dev_crypto_context_s ccm_tx_ctx;
  struct dev_crypto_context_s ccm_rx_ctx;
  struct net_task_s *ccm_task;
  net_task_queue_root_t ccm_queue;
  struct buffer_s *ccm_tmp_packet;
  uint8_t crypto_key[16];

  dev_timer_value_t drops_at;

  dev_timer_value_t last_anchor;
  dev_timer_value_t scheduled_anchor;

  dev_timer_delay_t timeout_tk;
  dev_timer_delay_t pending_timeout_tk;

  uint16_t interval;
  uint16_t pending_interval;
  uint16_t pending_win_offset;

  uint16_t last_event_counter;
  uint16_t scheduled_event_counter;
  uint16_t conn_params_update_instant;
  uint16_t channel_map_update_instant;
  uint16_t slave_latency;
  uint16_t pending_slave_latency;
  uint16_t latency_backoff;
  uint16_t since_last_event_unit;
  uint16_t to_next_event_unit;

  uint8_t mapped_channel[37];
  uint8_t pending_mapped_channel[37];
  uint8_t master_sca;
  uint8_t window;
  uint8_t pending_window;
  uint8_t hop;
  uint8_t last_unmapped_channel;
  uint8_t scheduled_unmapped_channel;

  bool_t req_scheduled : 1;
  bool_t channel_map_pending : 1;
  bool_t conn_params_pending : 1;
  bool_t version_sent : 1;
  bool_t tx_encryption : 1;
  bool_t rx_encryption : 1;

  uint8_t reason;

  struct ble_peer_s *peer;
};

struct ble_connection_parameters_s
{
  struct ble_adv_connect_s conn_req;
  dev_timer_value_t connect_packet_timestamp;
};

STRUCT_COMPOSE(ble_slave_s, layer);

struct ble_slave_handler_s
{
  void (*destroyed)(struct ble_slave_s *conn);
};

error_t ble_slave_init(
  struct ble_slave_s *slave,
  const struct ble_slave_handler_s *handler,
  struct net_scheduler_s *scheduler,
  const char *ble,
  const char *crypto,
  struct dev_rng_s *rng,
  const struct ble_connection_parameters_s *conn_params,
  struct ble_peer_s *peer);

#endif
