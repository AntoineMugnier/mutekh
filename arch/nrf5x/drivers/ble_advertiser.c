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

#include <ble/protocol/advertise.h>
#include <ble/net/adv.h>
#include <ble/net/layer.h>

#include "ble.h"
#include "ble_advertiser.h"

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

  uint32_t interval_tk;
  uint32_t delay_max_tk;

  struct ble_addr_s local_addr;

  struct buffer_s *adv_packet;
  struct buffer_s *scan_response;

  struct buffer_s *conn_packet;
  dev_timer_value_t conn_ts;

  enum adv_state_e state;
  uint8_t channel;

  uint8_t ad[62];
  size_t ad_len;
  
  bool_t connectable;
};

STRUCT_COMPOSE(nrf5x_ble_advertiser_s, context);

static void advertiser_schedule(struct nrf5x_ble_advertiser_s *adv)
{
  dev_timer_value_t now, begin;

  now = nrf5x_ble_rtc_value_get(adv->context.pv);
  begin = now + adv->interval_tk + (adv->delay_max_tk & rand());

  nrf5x_ble_context_schedule(&adv->context, begin, 0, 0, 100000);
}

static void advertiser_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_advertiser_s *adv = nrf5x_ble_advertiser_s_from_context(context);

  adv->channel = 37;
  adv->state = ADV_IND;
}

static void advertiser_ctx_event_closed(struct nrf5x_ble_context_s *context,
                                        enum event_status_e status)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);
  const struct ble_advertiser_delegate_vtable_s *vtable
    = const_ble_advertiser_delegate_vtable_s_from_base(adv->context.layer.delegate_vtable);
  bool_t reschedule = 1;

  //printk("%s %d %p\n", __FUNCTION__, status, adv->conn_packet);

  if (adv->conn_packet) {
    struct ble_adv_connect_s conn;
    error_t err = ble_adv_connect_parse(adv->conn_packet, &conn);

    if (!err)
      reschedule = vtable->connection_requested(
          adv->context.layer.delegate, &adv->context.layer,
          &conn, adv->conn_ts);

    buffer_refdec(adv->conn_packet);
    adv->conn_packet = NULL;
  }

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

  switch (adv->state) {
  case ADV_IND:
  case SCAN_RSP:
    params->ifs_timeout = 0;
    params->mode = MODE_TX;
    return 1;

  case SCAN_REQ:
    params->mode = MODE_RX;
    params->ifs_timeout = 1;
    return 1;

  case ADV_DONE:
    return 0;
  }

  return 0;
}

static struct buffer_s *advertiser_ctx_payload_get(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  switch (adv->state) {
  case ADV_IND:
    return buffer_refinc(adv->adv_packet);
  case SCAN_RSP:
    return buffer_refinc(adv->scan_response);
  default:
    return NULL;
  }
}

static void advertiser_ctx_ifs_event(struct nrf5x_ble_context_s *context,
                                     bool_t ifs_timeout)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  switch (adv->state) {
  case ADV_IND:
    adv->state = SCAN_REQ;
    break;

  case SCAN_REQ:
    if (!ifs_timeout) {
      adv->state = SCAN_RSP;
      break;
    }
    // Fallthrough

  case SCAN_RSP:
    if (adv->channel == 39) {
      adv->state = ADV_DONE;
      break;
    }

    adv->channel++;
    adv->state = ADV_IND;
    //    printk("%s %d\n", __FUNCTION__, adv->channel);
    break;

  default:
    break;
  }
}

static void advertiser_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                            dev_timer_value_t ts,
                                            bool_t crc_valid,
                                            struct buffer_s *packet)
{
  struct nrf5x_ble_advertiser_s *adv
    = nrf5x_ble_advertiser_s_from_context(context);

  // Remember state advanaced because of ifs_event
  assert(adv->state == SCAN_RSP);

  if (!crc_valid)
    goto channel_next;

  const uint8_t size = packet->end - packet->begin;
  struct ble_addr_s adva;

  switch (ble_advertise_packet_type_get(packet)) {
  case BLE_SCAN_REQ:
    if (size != 14)
      goto channel_next;

    ble_advertise_packet_rxaddr_get(packet, &adva);

    if (ble_addr_cmp(&adva, &adv->local_addr) || !adv->scan_response)
      goto channel_next;

    // Do as expected
    return;

  case BLE_CONNECT_REQ:
    if (size != 2 + 12 + 22 || !adv->connectable)
      goto channel_next;

    ble_advertise_packet_rxaddr_get(packet, &adva);

    if (ble_addr_cmp(&adva, &adv->local_addr))
      goto channel_next;

    adv->state = ADV_DONE;
    adv->conn_packet = buffer_refinc(packet);
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
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_advertiser_s *adv = nrf5x_ble_advertiser_s_from_context(ctx);

  nrf5x_ble_context_cleanup(&adv->context);
  buffer_refdec(adv->adv_packet);
  buffer_refdec(adv->scan_response);

  mem_free(adv);
}

error_t nrf5x_ble_advertiser_create(struct net_scheduler_s *scheduler,
                              struct nrf5x_ble_private_s *priv,
                              const void *params_,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable_,
                              struct net_layer_s **layer)
{
  struct nrf5x_ble_advertiser_s *adv = mem_alloc(sizeof(*adv), mem_scope_sys);
  error_t err;
  const struct ble_advertiser_delegate_vtable_s *delegate_vtable
    = const_ble_advertiser_delegate_vtable_s_from_base(delegate_vtable_);

  if (!adv)
    return -ENOMEM;

  memset(adv, 0, sizeof(*adv));

  err = nrf5x_ble_context_init(&adv->context,
                               scheduler,
                               &ble_advertiser_layer_handler.base,
                               priv, 
                               &ble_advertiser_ctx_handler,
                               delegate, &delegate_vtable->base);
  if (err)
    goto err_out;

  adv->adv_packet = net_layer_packet_alloc(&adv->context.layer, 0, 0);
  adv->scan_response = net_layer_packet_alloc(&adv->context.layer, 0, 0);

  adv_param_update(&adv->context.layer, params_);

  advertiser_schedule(adv);

  printk("Advertiser init done\n");

  *layer = &adv->context.layer;

  return 0;

 err_out:
  mem_free(adv);

  return err;
}

static
error_t adv_param_update(struct net_layer_s *layer, const struct ble_advertiser_param_s *params)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_advertiser_s *adv = nrf5x_ble_advertiser_s_from_context(ctx);
  const uint8_t *ad = params->ad;
  const uint8_t *end = params->ad + params->ad_len;

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
  adv->delay_max_tk = POW2_M1_CONSTANT_UP(params->delay_max_ms * 32768 / 1000);
  adv->connectable = params->connectable;

  if (!adv->delay_max_tk)
    adv->delay_max_tk = 3;

  printk("ADV interval: %d, delay_max: %d\n", adv->interval_tk, adv->delay_max_tk);
  printk("ADV packet: %P\n",
         adv->adv_packet->data + adv->adv_packet->begin,
         adv->adv_packet->end - adv->adv_packet->begin);
  printk("Scan RSP packet: %P\n",
         adv->scan_response->data + adv->scan_response->begin,
         adv->scan_response->end - adv->scan_response->begin);

  return 0;
}

static const struct ble_advertiser_handler_s ble_advertiser_layer_handler = {
  .params_update = adv_param_update,
  .base.destroyed = advertiser_layer_destroyed,
  .base.type = BLE_NET_LAYER_ADV,
};

static const struct nrf5x_ble_context_handler_s ble_advertiser_ctx_handler = {
  .event_opened = advertiser_ctx_event_opened,
  .event_closed = advertiser_ctx_event_closed,
  .radio_params = advertiser_ctx_radio_params,
  .payload_get = advertiser_ctx_payload_get,
  .ifs_event = advertiser_ctx_ifs_event,
  .payload_received = advertiser_ctx_payload_received,
};
