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

#include <hexo/bit.h>

#include <ble/protocol/radio.h>
#include <ble/protocol/advertise.h>
#include <ble/net/adv.h>

#include <net/scheduler.h>
#include <net/layer.h>
#include <net/task.h>

#include "ble.h"

#define dprintk(...) do{}while(0)

static
error_t adv_param_update(struct net_layer_s *layer, const struct ble_advertiser_param_s *params);

static const struct ble_advertiser_handler_s ble_advertiser_layer_handler;
static const struct nrf5x_ble_context_handler_s ble_advertiser_ctx_handler;

enum adv_state_e {
  ADV_IND,
  SCAN_REQ,
  SCAN_RSP,
  ADV_DONE,
};

struct nrf5x_ble_advertiser_s
{
  struct nrf5x_ble_context_s context;
  struct net_layer_s layer;

  uint32_t interval_tk;
  uint32_t delay_max_tk;

  struct ble_addr_s local_addr;

  struct buffer_s *adv_packet;
  struct buffer_s *scan_response;
  struct buffer_s *rx_buffer;
  struct buffer_s *conn_packet;
  dev_timer_value_t conn_ts;

  enum ble_phy_mode_e phy;
  enum adv_state_e state;
  uint8_t channel;

  uint8_t ad[62];
  size_t ad_len;
  
  bool_t connectable;
};

STRUCT_COMPOSE(nrf5x_ble_advertiser_s, context);
STRUCT_COMPOSE(nrf5x_ble_advertiser_s, layer);

static void advertiser_schedule(struct nrf5x_ble_advertiser_s *adv)
{
  dev_timer_value_t now, begin;

  //printk("%s %d\n", __FUNCTION__, net_layer_refcount(&adv->layer));
  
  if (!net_layer_refcount(&adv->layer))
    return;

  now = nrf5x_ble_rtc_value_get(adv->context.pv);
  begin = now + adv->interval_tk + (adv->delay_max_tk & rand());

  nrf5x_ble_context_schedule(&adv->context, begin, 0, 0, 0, 100000);
}

static bool_t advertiser_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_advertiser_s *adv = nrf5x_ble_advertiser_s_from_context(context);

  //printk("%s %d\n", __FUNCTION__, net_layer_refcount(&adv->layer));

  if (!net_layer_refcount(&adv->layer))
    return 0;

  adv->channel = 37;
  adv->state = ADV_IND;

  return 1;
}

static void advertiser_ctx_event_closed(struct nrf5x_ble_context_s *context,
                                        enum event_status_e status)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);
  const struct ble_advertiser_delegate_vtable_s *vtable
    = const_ble_advertiser_delegate_vtable_s_from_base(adv->layer.delegate_vtable);
  bool_t reschedule = 1;

  //printk("%s %d\n", __FUNCTION__, net_layer_refcount(&adv->layer));

  //printk("%s %d %p\n", __FUNCTION__, status, adv->conn_packet);

  if (adv->conn_packet) {
    struct ble_adv_connect_s cp;
    error_t err = ble_adv_connect_parse(adv->conn_packet, &cp);

    if (!err)
      reschedule = vtable->connection_requested(
          adv->layer.delegate, &adv->layer,
          &cp, adv->conn_ts);

    buffer_refdec(adv->conn_packet);
    adv->conn_packet = NULL;
  }

  if (!adv->rx_buffer)
    adv->rx_buffer = net_layer_packet_alloc(&adv->layer, 0, 0);
  
  if (reschedule)
    advertiser_schedule(adv);
}

static bool_t advertiser_ctx_radio_params(struct nrf5x_ble_context_s *context,
                                          struct nrf5x_ble_params_s *params)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  if (adv->conn_packet)
    return 0;

  params->channel = adv->channel;
  params->access = BLE_ADVERTISE_AA;
  params->crc_init = BLE_ADVERTISE_CRCINIT;
  params->tx_power = 0;
  params->rx_rssi = 0;
  params->whitening = 1;
  params->phy = adv->phy;

  switch (adv->state) {
  case ADV_IND:
  case SCAN_RSP:
    params->mode = MODE_TX;
    return 1;

  case SCAN_REQ:
    params->mode = MODE_RX;
    return 1;

  case ADV_DONE:
    return 0;
  }

  return 0;
}

static uint8_t *advertiser_ctx_payload_get(struct nrf5x_ble_context_s *context,
                                           enum nrf5x_ble_transfer_e mode)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  switch (adv->state) {
  case ADV_IND:
    assert(mode == MODE_TX);
    return adv->adv_packet->data + adv->adv_packet->begin;
  case SCAN_RSP:
    assert(mode == MODE_TX);
    return adv->scan_response->data + adv->scan_response->begin;
  case SCAN_REQ:
    assert(mode == MODE_RX);
    if (adv->rx_buffer)
      return adv->rx_buffer->data + adv->rx_buffer->begin;
  default:
    return NULL;
  }
}

static void advertiser_ctx_ifs_event(struct nrf5x_ble_context_s *context,
                                     bool_t timeout)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  if (!timeout) {
    switch (adv->state) {
    case ADV_IND:
      adv->state = SCAN_REQ;
      return;

    case SCAN_REQ:
      adv->state = SCAN_RSP;
      return;

    default:
      adv->state = ADV_DONE;
      return;

    case SCAN_RSP:
      // Go to next adv channel, like for timeouts
      break;
    }
  }

  if (adv->channel == 39) {
    adv->state = ADV_DONE;
  } else {
    adv->channel++;
    adv->state = ADV_IND;
  }
}

static void advertiser_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                            dev_timer_value_t ts,
                                            int16_t rssi,
                                            bool_t crc_valid)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  // Remember state advanaced because of ifs_event
  assert(adv->state == SCAN_RSP);

  if (!crc_valid)
    goto channel_next;

  const uint8_t size = __MIN(CONFIG_BLE_PACKET_SIZE,
                             adv->rx_buffer->data[adv->rx_buffer->begin + 1] + 2);
  struct ble_addr_s adva;

  adv->rx_buffer->end = adv->rx_buffer->begin + size;
  switch (ble_advertise_packet_type_get(adv->rx_buffer)) {
  case BLE_SCAN_REQ:
    if (size != 14)
      goto channel_next;

    ble_advertise_packet_rxaddr_get(adv->rx_buffer, &adva);

    if (ble_addr_cmp(&adva, &adv->local_addr) || !adv->scan_response)
      goto channel_next;

    // Do as expected
    return;

  case BLE_CONNECT_REQ:
    if (size != 2 + 12 + 22 || !adv->connectable)
      goto channel_next;

    ble_advertise_packet_rxaddr_get(adv->rx_buffer, &adva);

    if (ble_addr_cmp(&adva, &adv->local_addr))
      goto channel_next;

    adv->state = ADV_DONE;
    adv->conn_packet = adv->rx_buffer;
    adv->rx_buffer = NULL;
    adv->conn_ts = ts;
    return;

  default:
  channel_next:
    if (adv->channel < 39) {
      adv->channel++;
      adv->state = ADV_IND;
      break;
    }

    adv->state = ADV_DONE;
  }
}

static
void advertiser_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_advertiser_s *adv = nrf5x_ble_advertiser_s_from_layer(layer);

  nrf5x_ble_context_cleanup(&adv->context);
  if (adv->adv_packet)
    buffer_refdec(adv->adv_packet);
  if (adv->scan_response)
    buffer_refdec(adv->scan_response);
  if (adv->rx_buffer)
    buffer_refdec(adv->rx_buffer);
  if (adv->conn_packet)
    buffer_refdec(adv->conn_packet);

  mem_free(adv);
}

error_t nrf5x_ble_advertiser_create(struct net_scheduler_s *scheduler,
                              struct nrf5x_ble_private_s *priv,
                              const void *params_,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer)
{
  struct nrf5x_ble_advertiser_s *adv = mem_alloc(sizeof(*adv), mem_scope_sys);
  error_t err;

  if (!adv)
    return -ENOMEM;

  memset(adv, 0, sizeof(*adv));

  dprintk("Advertiser init start\n");

  err = net_layer_init(&adv->layer, &ble_advertiser_layer_handler.base, scheduler, delegate, delegate_vtable);
  if (err)
    goto err_out;

  nrf5x_ble_context_init(priv, &adv->context, &ble_advertiser_ctx_handler);

  assert(CONFIG_BLE_PACKET_SIZE - 1 >= 2 + 6 + 31);

  adv->adv_packet = net_layer_packet_alloc(&adv->layer, 0, 0);
  adv->scan_response = net_layer_packet_alloc(&adv->layer, 0, 0);

  adv_param_update(&adv->layer, params_);

  adv->rx_buffer = net_layer_packet_alloc(&adv->layer, 0, 0);

  advertiser_schedule(adv);

  dprintk("Advertiser init done\n");

  *layer = &adv->layer;

  return 0;

 err_out:
  mem_free(adv);

  return err;
}

static
error_t adv_param_update(struct net_layer_s *layer, const struct ble_advertiser_param_s *params)
{
  struct nrf5x_ble_advertiser_s *adv = nrf5x_ble_advertiser_s_from_layer(layer);
  const uint8_t *ad = params->ad;
  const uint8_t *end = params->ad + params->ad_len;

  if (!nrf5x_ble_phy_is_supported(params->phy))
    return -ENOTSUP;
  
  adv->phy = params->phy;
  if (params->connectable)
    ble_adv_ind_set(adv->adv_packet, &params->local_addr);
  else
    ble_adv_nonconn_ind_set(adv->adv_packet, &params->local_addr);
  ble_adv_scan_rsp_set(adv->scan_response, &params->local_addr);

  adv->adv_packet->end = adv->adv_packet->begin + 8;
  adv->adv_packet->data[adv->adv_packet->begin + 1] = 6;

  adv->scan_response->end = adv->scan_response->begin + 8;
  adv->scan_response->data[adv->scan_response->begin + 1] = 6;

  while (ad < end) {
    uint8_t length = *ad;
    error_t err = ble_adv_data_append(adv->adv_packet, ad[1], ad + 2, length - 1);

    if (err)
      break;

    ad += length + 1;
  }

  while (ad < end) {
    uint8_t length = *ad;
    error_t err = ble_adv_data_append(adv->scan_response, ad[1], ad + 2, length - 1);

    if (err)
      break;

    ad += length + 1;
  }

  adv->local_addr = params->local_addr;
  adv->interval_tk = params->interval_ms * 32768 / 1000;
  adv->delay_max_tk = pow2_m1_up(params->delay_max_ms * 32768 / 1000);
  adv->connectable = params->connectable;

  if (!adv->delay_max_tk)
    adv->delay_max_tk = 3;

  dprintk("ADV interval: %d, delay_max: %d\n", adv->interval_tk, adv->delay_max_tk);
  dprintk("ADV packet: %P\n",
         adv->adv_packet->data + adv->adv_packet->begin,
         adv->adv_packet->end - adv->adv_packet->begin);
  dprintk("Scan RSP packet: %P\n",
         adv->scan_response->data + adv->scan_response->begin,
         adv->scan_response->end - adv->scan_response->begin);

  return 0;
}

static const struct ble_advertiser_handler_s ble_advertiser_layer_handler = {
  .params_update = adv_param_update,
  .base.destroyed = advertiser_layer_destroyed,
};

static const struct nrf5x_ble_context_handler_s ble_advertiser_ctx_handler = {
  .event_opened = advertiser_ctx_event_opened,
  .event_closed = advertiser_ctx_event_closed,
  .radio_params = advertiser_ctx_radio_params,
  .payload_get = advertiser_ctx_payload_get,
  .ifs_event = advertiser_ctx_ifs_event,
  .payload_received = advertiser_ctx_payload_received,
};
