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

#include <device/class/ble_radio.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>
#include "ble_radio_handler.h"

static
bool_t rssi_radio_params(const struct ble_radio *radio,
                         struct ble_radio_params *params)
{
  return 0;
}

#if defined(CONFIG_BLE_SNIFFER)

static
bool_t sniff_radio_params(const struct ble_radio *radio,
                        struct ble_radio_params *params)
{
  params->channel = radio->current->sniff.channel;
  params->access = radio->current->sniff.access;
  params->crc_init = radio->current->sniff.crc_init;
  params->ifs = 0;
  params->mode = MODE_RX;
  params->rx_rssi = radio->current->sniff.rssi_sample;
  params->tx_power = 0;

  return 1;
}

static
void sniff_payload_received(struct ble_radio *radio,
                          bool_t crc_valid,
                          struct buffer_s *packet)
{
  radio->current->sniff.rssi = radio->last_rx_rssi;
  buffer_queue_pushback(&radio->current->sniff.rx_queue, packet);
}

#endif


#if defined(CONFIG_BLE_PERIPHERAL)

static
void adv_startup(struct ble_radio *radio)
{
  radio->state.adv.state = ADV_IND;
  radio->state.adv.channel = 37;
  radio->current->adv.conn_packet = NULL;
}

static
bool_t adv_radio_params(const struct ble_radio *radio,
                        struct ble_radio_params *params)
{
  params->channel = radio->state.adv.channel;
  params->access = BLE_ADVERTISE_AA;
  params->crc_init = BLE_ADVERTISE_CRCINIT;
  params->ifs = 0;
  params->tx_power = 0;

  switch (radio->state.adv.state) {
  case ADV_IND:
    params->mode = MODE_TX;
    return 1;

  case ADV_SCAN_REQ:
    params->mode = MODE_RX;
    params->ifs = 1;
    return 1;

  case ADV_SCAN_RSP:
    params->mode = MODE_TX;
    params->ifs = 1;
    return 1;

  default:
  case ADV_DONE:
    return 0;
  }
}

static
struct buffer_s *adv_payload_get(const struct ble_radio *radio)
{
  switch (radio->state.adv.state) {
  case ADV_IND:
    return buffer_refinc(radio->current->adv.adv_packet);
  case ADV_SCAN_RSP:
    return buffer_refinc(radio->current->adv.scan_rsp_packet);
  default:
    return NULL;
  }
}

static
void adv_ifs_event(struct ble_radio *radio,
                   bool_t rx_timeout)
{
  switch (radio->state.adv.state) {
  case ADV_SCAN_REQ:
    if (rx_timeout)
      goto channel_next;

    radio->state.adv.state = ADV_SCAN_RSP;
    return;

  case ADV_IND:
    radio->state.adv.state = ADV_SCAN_REQ;
    return;

  case ADV_SCAN_RSP:
    goto channel_next;

  default:
    return;
  }

 channel_next:
  if (radio->state.adv.channel < 39) {
    radio->state.adv.channel++;
    radio->state.adv.state = ADV_IND;
  } else {
    radio->state.adv.state = ADV_DONE;
  }
}

static
void adv_payload_received(struct ble_radio *radio,
                          bool_t crc_valid,
                          struct buffer_s *packet)
{
  // Remember state advanaced because of ifs_event
  assert(radio->state.adv.state == ADV_SCAN_RSP);

  if (!crc_valid)
    goto channel_next;

  const uint8_t size = packet->end - packet->begin;
  struct ble_addr_s adva, expected;

  switch (ble_advertise_packet_type_get(packet)) {
  case BLE_SCAN_REQ:
    if (size != 14)
      goto channel_next;

    ble_advertise_packet_txaddr_get(radio->current->adv.adv_packet, &expected);
    ble_advertise_packet_rxaddr_get(packet, &adva);

    if (ble_addr_cmp(&adva, &expected))
      goto channel_next;

    if (!radio->current->adv.scan_rsp_packet)
      goto channel_next;

    // Do as expected
    return;

  case BLE_CONNECT_REQ:
    if (size != 2 + 12 + 22)
      goto channel_next;

    ble_advertise_packet_txaddr_get(radio->current->adv.adv_packet, &expected);
    ble_advertise_packet_rxaddr_get(packet, &adva);

    if (ble_addr_cmp(&adva, &expected))
      goto channel_next;

    radio->state.adv.state = ADV_DONE;
    radio->current->adv.conn_packet = buffer_refinc(packet);
    radio->current->adv.anchor = radio->address_timestamp;
    return;

  default:
    goto channel_next;
  }

 channel_next:
  if (radio->state.adv.channel < 39) {
    radio->state.adv.channel++;
    radio->state.adv.state = ADV_IND;
  } else {
    radio->state.adv.state = ADV_DONE;
  }
}

#endif


#if defined(CONFIG_BLE_CENTRAL)

static
void adv_listen_startup(struct ble_radio *radio)
{
  buffer_queue_init(&radio->current->adv_listen.received);
}

static
bool_t adv_listen_radio_params(const struct ble_radio *radio,
                               struct ble_radio_params *params)
{
  params->channel = radio->current->adv_listen.channel;
  params->access = BLE_ADVERTISE_AA;
  params->crc_init = BLE_ADVERTISE_CRCINIT;
  params->ifs = 0;
  params->tx_power = 0;

  switch (radio->state.adv.state) {
  case ADV_IND:
  case ADV_SCAN_RSP:
    params->mode = MODE_RX;
    params->ifs = 1;
    return 1;

  case ADV_SCAN_REQ:
    params->mode = MODE_TX;
    return 1;

  default:
  case ADV_DONE:
    return 0;
  }

  return 1;
}

static
struct buffer_s *adv_payload_get(const struct ble_radio *radio)
{
  if (radio->state.adv.state != ADV_SCAN_REQ)
    return NULL;

  struct buffer_s *pkt = nrf5x_ble_radio_packet_alloc(radio);
  ble_adv_scan_req_set(pkt, &radio->current->adv_listen.addr, &radio->state.adv.adva);
  return pkt;
}

static
void adv_ifs_event(struct ble_radio *radio,
                   bool_t rx_timeout)
{
  if (rx_timeout)
    goto next;

  switch (radio->state.adv.state) {
  case ADV_SCAN_REQ:
    radio->state.adv.state = ADV_SCAN_RSP;
    return;

  case ADV_IND:
    if (radio->current->adv_listen.do_scan)
      radio->state.adv.state = ADV_SCAN_REQ;
    return;

  default:
  case ADV_SCAN_RSP:
    goto next;
  }

 next:
  radio->state.adv.state = ADV_IND;
}

static
void adv_listen_payload_received(struct ble_radio *radio,
                                 bool_t crc_valid,
                                 struct buffer_s *packet)
{
  const uint8_t size = packet->end - packet->begin;
  bool_t duplicate = 0;

  if (!crc_valid)
    return;

  switch (ble_advertise_packet_type_get(packet)) {
  case BLE_ADV_DIRECT_IND:
    if (size < 14)
      return;

  case BLE_ADV_IND:
  case BLE_ADV_NONCONN_IND:
  case BLE_ADV_SCAN_IND:
    if (size < 8)
      return;

    ble_advertise_packet_txaddr_get(packet, &radio->state.adv.adva);

    GCT_FOREACH(buffer_list, &radio->current->adv_listen.received, item,
                if (!memcmp(item->data + item->begin,
                            packet->data + packet->begin,
                            8)) {
                  duplicate = true;
                  GCT_FOREACH_BREAK;
                });

    if (!duplicate)
      buffer_queue_pushback(&radio->current->adv_listen.received, packet);
    return;

  default:
    return;
  }
}



static
void adv_scan_startup(struct ble_radio *radio)
{
  radio->state.adv.state = ADV_IND;
  radio->current->adv_scan.adv_packet = NULL;
  radio->current->adv_scan.scan_rsp_packet = NULL;
  assert(radio->current->adv_scan.scan_packet);
}

static
bool_t adv_scan_radio_params(const struct ble_radio *radio,
                             struct ble_radio_params *params)
{
  params->access = BLE_ADVERTISE_AA;
  params->crc_init = BLE_ADVERTISE_CRCINIT;
  params->channel = radio->current->adv_scan.channel;
  params->ifs = 0;
  params->rx_rssi = 0;
  params->tx_power = 0;

  switch (radio->state.adv.state) {
  case ADV_SCAN_RSP:
    params->ifs = 1;
    params->mode = MODE_RX;
    return 1;

  case ADV_IND:
    params->mode = MODE_RX;
    params->rx_rssi = 1;
    return 1;

  case ADV_SCAN_REQ:
    params->mode = MODE_TX;
    params->ifs = 1;
    return 1;

  case ADV_DONE:
  default:
    return 0;
  }
}

static
struct buffer_s *adv_scan_payload_get(const struct ble_radio *radio)
{
  return buffer_refinc(radio->current->adv_scan.scan_packet);
}

static
void adv_scan_ifs_event(struct ble_radio *radio,
                    bool_t rx_timeout)
{
  if (rx_timeout)
    radio->state.adv.state = ADV_IND;
  else
    radio->state.adv.state++;
}

static
void adv_scan_payload_received(struct ble_radio *radio,
                               bool_t crc_valid,
                               struct buffer_s *packet)
{
  assert(radio->state.adv.state == ADV_SCAN_REQ || radio->state.adv.state == ADV_DONE);

  const uint8_t size = packet->end - packet->begin;
  struct ble_addr_s expected, adva;

  if (!crc_valid)
    goto restart;

  switch (ble_advertise_packet_type_get(packet)) {
  case BLE_ADV_IND:
    if (size < 8)
      goto restart;

    ble_advertise_packet_rxaddr_get(radio->current->adv_scan.scan_packet, &expected);
    ble_advertise_packet_txaddr_get(packet, &adva);

    radio->current->adv_scan.rssi = radio->last_rx_rssi;

    if (!ble_addr_cmp(&expected, &adva)) {
      radio->current->adv_scan.adv_packet = buffer_refinc(packet);
      goto scan;
    }

    goto restart;

  case BLE_SCAN_RSP:
    if (size < 8)
      goto restart;

    ble_advertise_packet_rxaddr_get(radio->current->adv_scan.scan_packet, &expected);
    ble_advertise_packet_txaddr_get(packet, &adva);

    if (!ble_addr_cmp(&expected, &adva)) {
      radio->current->adv_scan.scan_rsp_packet = buffer_refinc(packet);
      goto done;
    }

    goto restart;

  default:
    goto restart;
  }

 restart:
  radio->state.adv.state = ADV_IND;

  if (radio->current->adv_scan.adv_packet)
    buffer_refdec(radio->current->adv_scan.adv_packet);

  return;

 scan:
  assert(radio->state.adv.state == ADV_SCAN_REQ);
  return;

 done:
  assert(radio->state.adv.state == ADV_DONE);
  return;
}



static
void connect_startup(struct ble_radio *radio)
{
  radio->state.conn.state = CONN_ADV;
}

static
bool_t connect_radio_params(const struct ble_radio *radio,
                            struct ble_radio_params *params)
{
  params->access = BLE_ADVERTISE_AA;
  params->crc_init = BLE_ADVERTISE_CRCINIT;
  params->channel = radio->current->connect.channel;
  params->ifs = 0;
  params->tx_power = 0;

  switch (radio->state.conn.state) {
  case CONN_SEND:
    params->mode = MODE_TX;
    params->ifs = 1;
    return 1;

  case CONN_DONE:
    return 0;

  default:
    params->mode = MODE_RX;
    return 1;
  }
}

static
struct buffer_s *connect_payload_get(const struct ble_radio *radio)
{
  return buffer_refinc(radio->current->connect.conn_packet);
}

static
void connect_ifs_event(struct ble_radio *radio,
                       bool_t rx_timeout)
{
  assert(radio->state.conn.state != CONN_DONE);

  if (rx_timeout)
    radio->state.conn.state = CONN_ADV;
  else
    radio->state.conn.state++;
}

static
void connect_payload_received(struct ble_radio *radio,
                              bool_t crc_valid,
                              struct buffer_s *packet)
{
  // State changed through ifs_event before
  assert(radio->state.conn.state == CONN_SEND);

  const uint8_t size = packet->end - packet->begin;
  struct ble_addr_s expected, adva;

  if (size < 8 || !crc_valid)
    goto restart;

  switch (ble_advertise_packet_type_get(packet)) {
  case BLE_ADV_IND:
  case BLE_ADV_DIRECT_IND:
    ble_advertise_packet_txaddr_get(packet, &adva);
    ble_advertise_packet_rxaddr_get(radio->current->connect.conn_packet, &expected);

    // Calculate anchor point from receive time and tx packet size
    radio->current->connect.anchor = radio->address_timestamp;

    if (!ble_addr_cmp(&expected, &adva))
      goto conn;

  default:
    goto restart;
  }

 restart:
  radio->state.conn.state = CONN_ADV;
  return;

 conn:
  // Do as expected
  return;
}

#endif

#if defined(CONFIG_BLE_CENTRAL) || defined(CONFIG_BLE_PERIPHERAL)

static
void data_startup(struct ble_radio *radio)
{
  radio->state.data.done = 0;
  radio->state.data.last_rx_ok = 0;

  radio->current->data.packet_count = 0;
  radio->current->data.tx_count = 0;
  radio->current->data.tx_acked_count = 0;
  radio->current->data.tx_empty_count = 0;
  radio->current->data.rx_ok_count = 0;
  radio->current->data.rx_invalid_count = 0;
  radio->current->data.rx_repeated_count = 0;
  radio->current->data.rx_empty_count = 0;

  radio->current->data.anchor = radio->current->not_before;
}

#if defined(CONFIG_BLE_CENTRAL)

static
bool_t master_radio_params(const struct ble_radio *radio,
                           struct ble_radio_params *params)
{
  if (radio->state.data.done)
    return 0;

  params->access = radio->current->data.access;
  params->crc_init = radio->current->data.crc_init;
  params->channel = radio->current->data.channel;
  params->tx_power = radio->current->data.tx_power;

  params->mode = (radio->current->data.packet_count & 1) ? MODE_RX : MODE_TX;
  params->ifs = radio->current->data.packet_count != 0;

  params->rx_rssi = radio->current->data.rssi_sample
    && radio->current->data.packet_count == 1;

  return radio->current->data.packet_count < 2
    || !buffer_queue_isempty(&radio->current->data.tx_queue)
    || radio->current->data.md;
}

#else

static
bool_t slave_radio_params(const struct ble_radio *radio,
                            struct ble_radio_params *params)
{
  if (radio->state.data.done)
    return 0;

  params->access = radio->current->data.access;
  params->crc_init = radio->current->data.crc_init;
  params->channel = radio->current->data.channel;
  params->tx_power = radio->current->data.tx_power;

  params->mode = (radio->current->data.packet_count & 1) ? MODE_TX : MODE_RX;
  params->ifs = radio->current->data.packet_count != 0;

  params->rx_rssi = radio->current->data.rssi_sample
    && radio->current->data.packet_count == 0;

  return !buffer_queue_isempty(&radio->current->data.tx_queue)
    || radio->current->data.md
    // If last received packet has data (or got received broken),
    // force another roundtrip.
    || (((radio->current->data.packet_count & 1) == 1)
        && (!radio->state.data.last_rx_ok || !radio->state.data.last_empty))
    || radio->current->data.md
    //|| (radio->current->data.packet_count % 2)
    || (radio->current->data.packet_count < 2
        && !radio->current->data.latency_permitted);
}

#endif

static
struct buffer_s *data_payload_get(const struct ble_radio *radio)
{
  struct buffer_s *packet, *next;

  if (buffer_queue_isempty(&radio->current->data.tx_queue)) {
    // Tx queue is empty, we have to create an empty packet to piggyback
    // handshaking.  Cannot borrow reference to a common packet as it is
    // chained in tx queue.
    packet = nrf5x_ble_radio_packet_alloc(radio);

    packet->data[packet->begin + 0] = BLE_LL_DATA_CONT;
    packet->data[packet->begin + 1] = 0;
    packet->end = packet->begin + 2;

    radio->current->data.tx_empty_count++;

    buffer_queue_pushback(&radio->current->data.tx_queue, packet);
  } else {
    packet = buffer_queue_head(&radio->current->data.tx_queue);
  }

  radio->current->data.tx_count++;

  next = buffer_queue_next(&radio->current->data.tx_queue, packet);

  packet->data[packet->begin] = (packet->data[packet->begin] & 0x3)
    // If head packet has data in it, force master to ack it.
    | (packet->data[packet->begin + 1] ? BLE_LL_DATA_MD : 0)
    // Normal MD semantics
    | (next ? BLE_LL_DATA_MD : 0)
    | (radio->current->data.nesn ? BLE_LL_DATA_NESN : 0)
    | (radio->current->data.sn ? BLE_LL_DATA_SN : 0);

  if (next)
    buffer_refdec(next);

  return packet;
}

static
void data_ifs_event(struct ble_radio *radio,
                    bool_t rx_timeout)
{
  if (rx_timeout)
    radio->state.data.done = 1;
  else
    radio->current->data.packet_count++;
}

static
void data_payload_received(struct ble_radio *radio,
                           bool_t crc_valid,
                           struct buffer_s *packet)
{
  const uint8_t size = packet->end - packet->begin;

  if (radio->current->data.packet_count < 2) {
    radio->current->data.anchor = radio->address_timestamp;
    radio->current->data.rssi = radio->last_rx_rssi;
  }

  /* Dont handle invalid packets (handshaking is lost as well) */
  if (size < 2 || !crc_valid) {
    radio->state.data.last_empty = 0;
    radio->state.data.last_rx_ok = 0;
    radio->current->data.rx_invalid_count++;
    return;
  }

  radio->state.data.last_rx_ok = 1;
  radio->current->data.rx_ok_count++;

  bool_t sn = ble_data_sn_get(packet);
  bool_t nesn = ble_data_nesn_get(packet);
  bool_t md = ble_data_md_get(packet);
  bool_t empty = size == 2 && ble_data_llid_get(packet) == BLE_LL_DATA_CONT;

  radio->state.data.last_empty = empty;

  if (radio->current->data.nesn != sn)
    radio->current->data.rx_repeated_count++;
  else if (empty)
    radio->current->data.rx_empty_count++;
  else
    buffer_queue_pushback(&radio->current->data.rx_queue, packet);

  radio->current->data.md = md;
  radio->current->data.nesn = !sn;

  if (nesn != radio->current->data.sn) {
    radio->current->data.sn = !radio->current->data.sn;
    radio->current->data.tx_acked_count++;
    buffer_refdec(buffer_queue_pop(&radio->current->data.tx_queue));
  }
}

#endif

const struct ble_radio_request_handler
request_handler[DEVICE_BLE_RADIO_COMMAND_MAX + 1] = {
  [DEVICE_BLE_RADIO_RSSI_SAMPLE] = {
    .radio_params = rssi_radio_params,
  },

#if defined(CONFIG_BLE_SNIFFER)
  [DEVICE_BLE_RADIO_SNIFF] = {
    .radio_params = sniff_radio_params,
    .payload_received = sniff_payload_received,
  },
#endif

#if defined(CONFIG_BLE_PERIPHERAL)
  [DEVICE_BLE_RADIO_ADV] = {
    .startup = adv_startup,
    .radio_params = adv_radio_params,
    .payload_get = adv_payload_get,
    .ifs_event = adv_ifs_event,
    .payload_received = adv_payload_received,
  },

  [DEVICE_BLE_RADIO_DATA_SLAVE] = {
    .startup = data_startup,
    .radio_params = slave_radio_params,
    .payload_get = data_payload_get,
    .ifs_event = data_ifs_event,
    .payload_received = data_payload_received,
  },
#endif

#if defined(CONFIG_BLE_CENTRAL)
  [DEVICE_BLE_RADIO_ADV_LISTEN] = {
    .startup = adv_listen_startup,
    .radio_params = adv_listen_radio_params,
    .payload_get = adv_listen_payload_get,
    .ifs_event = adv_listen_ifs_event,
    .payload_received = adv_listen_payload_received,
  },

  [DEVICE_BLE_RADIO_ADV_SCAN] = {
    .startup = adv_scan_startup,
    .radio_params = adv_scan_radio_params,
    .payload_get = adv_scan_payload_get,
    .ifs_event = adv_scan_ifs_event,
    .payload_received = adv_scan_payload_received,
  },

  [DEVICE_BLE_RADIO_CONNECT] = {
    .startup = connect_startup,
    .radio_params = connect_radio_params,
    .payload_get = connect_payload_get,
    .ifs_event = connect_ifs_event,
    .payload_received = connect_payload_received,
  },

  [DEVICE_BLE_RADIO_DATA_MASTER] = {
    .startup = data_startup,
    .radio_params = master_radio_params,
    .payload_get = data_payload_get,
    .ifs_event = data_ifs_event,
    .payload_received = data_payload_received,
  },
#endif
};
