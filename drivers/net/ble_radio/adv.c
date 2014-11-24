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

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/layer.h>
#include <ble/net/adv.h>
#include <ble/net/gap.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/advertise.h>

#include <device/class/ble_radio.h>
#include <device/class/timer.h>
#include <device/request.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include "ble_radio_private.h"

#define dprintk printk

struct ble_advertiser_s
{
  struct net_layer_s layer;

  struct ble_private_s *pv;

  struct dev_ble_radio_rq_s ble_rq;

  uint32_t interval_tk;
  uint32_t delay_max_tk;

  dev_timer_value_t last_adv_ts;

  struct buffer_s *adv_packet;
  struct buffer_s *scan_response;
};

STRUCT_COMPOSE(ble_advertiser_s, layer);

static const struct ble_advertiser_handler_s advertiser_handler;

static void advertiser_request_schedule(struct ble_advertiser_s *adv)
{
  int32_t delay;
  error_t err;

 again:
  delay = adv->delay_max_tk & rand();
  delay += adv->interval_tk;

  adv->last_adv_ts = adv->ble_rq.not_before = adv->last_adv_ts + delay;

  err = DEVICE_OP(&adv->pv->radio, request, &adv->ble_rq);
  if (err == -ETIMEDOUT) {
    DEVICE_OP(&adv->pv->timer, get_value, &adv->last_adv_ts, 0);
    goto again;
  } else if (err) {
    net_layer_refdec(&adv->layer);
  }
}

static KROUTINE_EXEC(advertiser_rq_done)
{
  struct ble_advertiser_s *adv = KROUTINE_CONTAINER(kr, *adv, ble_rq.base.kr);
  bool_t schedule_again = 1;

  if (net_layer_refcount(&adv->layer) == 1) {
    net_layer_refdec(&adv->layer);
    return;
  }

  if (adv->ble_rq.adv.conn_packet) {
    if (adv->layer.delegate) {
      const struct ble_advertiser_delegate_vtable_s *vtable
        = const_ble_advertiser_delegate_vtable_s_from_base(adv->layer.delegate_vtable);
      struct ble_adv_connect_s conn;

      error_t err = ble_adv_connect_parse(adv->ble_rq.adv.conn_packet, &conn);

      if (!err)
        schedule_again = vtable->connection_requested(
          adv->layer.delegate, &adv->layer,
          &conn, adv->ble_rq.adv.anchor);
    }

    buffer_refdec(adv->ble_rq.adv.conn_packet);
    adv->ble_rq.adv.conn_packet = NULL;
  }

  if (schedule_again)
    advertiser_request_schedule(adv);
  else
    net_layer_refdec(&adv->layer);
}

static
void ble_advertiser_destroyed(struct net_layer_s *layer)
{
  struct ble_advertiser_s *adv = ble_advertiser_s_from_layer(layer);

  dprintk("Advertiser %p destroyed\n", adv);

  buffer_refdec(adv->adv_packet);
  buffer_refdec(adv->scan_response);

  device_stop(&adv->pv->radio);

  mem_free(adv);
}

static
error_t adv_param_update(struct net_layer_s *layer, const struct ble_advertiser_param_s *params)
{
  struct ble_advertiser_s *adv = ble_advertiser_s_from_layer(layer);
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

  dev_timer_init_sec(&adv->pv->timer, &adv->interval_tk, 0, params->interval_ms, 1000);
  dev_timer_init_sec(&adv->pv->timer, &adv->delay_max_tk, 0, params->delay_max_ms, 1000);

  adv->delay_max_tk = POW2_M1_CONSTANT_UP(adv->delay_max_tk);

  printk("ADV interval: %d, delay_max: %d\n", adv->interval_tk, adv->delay_max_tk);
  printk("ADV packet: %P\n",
         adv->adv_packet->data + adv->adv_packet->begin,
         adv->adv_packet->end - adv->adv_packet->begin);
  printk("Scan RSP packet: %P\n",
         adv->scan_response->data + adv->scan_response->begin,
         adv->scan_response->end - adv->scan_response->begin);

  return 0;
}

static const struct ble_advertiser_handler_s advertiser_handler = {
  .params_update = adv_param_update,
  .base.destroyed = ble_advertiser_destroyed,
  .base.type = BLE_NET_LAYER_ADV,
};

error_t ble_advertiser_create(struct net_scheduler_s *scheduler,
                              struct ble_private_s *priv,
                              const void *params_,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable_,
                              struct net_layer_s **layer)
{
  struct ble_advertiser_s *adv = mem_alloc(sizeof(*adv), mem_scope_sys);
  error_t err;
  struct ble_radio_info_s ble_info;
  const struct ble_advertiser_delegate_vtable_s *delegate_vtable
    = const_ble_advertiser_delegate_vtable_s_from_base(delegate_vtable_);

  if (!adv)
    return -ENOMEM;

  memset(adv, 0, sizeof(*adv));

  adv->pv = priv;

  err = net_layer_init(&adv->layer, &advertiser_handler.base, scheduler, delegate,
                       &delegate_vtable->base);
  if (err)
    return err;

  device_start(&adv->pv->radio);

  DEVICE_OP(&adv->pv->timer, get_value, &adv->last_adv_ts, 0);
  DEVICE_OP(&adv->pv->radio, get_info, &ble_info);

  adv->adv_packet = net_layer_packet_alloc(&adv->layer, ble_info.prefix_size, 8);
  adv->scan_response = net_layer_packet_alloc(&adv->layer, ble_info.prefix_size, 8);

  adv_param_update(&adv->layer, params_);

  adv->ble_rq.not_after = 0;
  adv->ble_rq.max_duration = 0;
  adv->ble_rq.type = DEVICE_BLE_RADIO_ADV;
  adv->ble_rq.packet_pool = scheduler->packet_pool;
  adv->ble_rq.adv.adv_packet = adv->adv_packet;
  adv->ble_rq.adv.scan_rsp_packet = adv->scan_response;
  adv->ble_rq.adv.conn_packet = NULL;
  kroutine_init(&adv->ble_rq.base.kr, advertiser_rq_done, KROUTINE_INTERRUPTIBLE);

  // We keep our own ref, return another one to caller
  *layer = net_layer_refinc(&adv->layer);

  advertiser_request_schedule(adv);

  dprintk("Advertiser init done\n");

  return 0;

 err_out:
  mem_free(adv);

  return err;
}
