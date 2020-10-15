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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2020
*/

#define LOGK_MODULE_ID "ndtm"

#include <mutek/printk.h>
#include <hexo/bit.h>

#include <ble/protocol/advertise.h>
#include <ble/protocol/radio.h>
#include <ble/net/dtm_tx.h>

#include <net/scheduler.h>
#include <net/layer.h>
#include <net/task.h>

#include "ble.h"

static
error_t dtm_tx_param_update(struct net_layer_s *layer, const struct ble_dtm_tx_param_s *params);

static const struct ble_dtm_tx_handler_s ble_dtm_tx_layer_handler;
static const struct nrf5x_ble_context_handler_s ble_dtm_tx_ctx_handler;

struct nrf5x_ble_dtm_tx_s
{
  struct nrf5x_ble_context_s context;
  struct net_layer_s layer;

  uint64_t channel_map;
  uint64_t channel_todo;

  uint32_t interval_tk;
  uint32_t delay_max_tk;
  uint16_t tx_power;
  int8_t channel;
  uint8_t pdu[256];
};

STRUCT_COMPOSE(nrf5x_ble_dtm_tx_s, context);
STRUCT_COMPOSE(nrf5x_ble_dtm_tx_s, layer);

static
void dtm_tx_schedule(struct nrf5x_ble_dtm_tx_s *dtm)
{
  dev_timer_value_t now, begin;
  
  if (!net_layer_refcount(&dtm->layer))
    return;

  now = nrf5x_ble_rtc_value_get(dtm->context.pv);
  begin = now + dtm->interval_tk + (dtm->delay_max_tk & rand());

  nrf5x_ble_context_schedule(&dtm->context, begin, 0, 0, 0, 100000);
}

static
void channel_set_next(struct nrf5x_ble_dtm_tx_s *dtm)
{
  if (dtm->channel_todo) {
    dtm->channel = __builtin_ctzll(dtm->channel_todo);
    dtm->channel_todo &= ~bit(dtm->channel);
  } else
    dtm->channel = -1;
}

static
bool_t dtm_tx_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_dtm_tx_s *dtm = nrf5x_ble_dtm_tx_s_from_context(context);

  if (!net_layer_refcount(&dtm->layer))
    return 0;

  logk_trace("op");
  dtm->channel_todo = dtm->channel_map;
  channel_set_next(dtm);

  return 1;
}

static
void dtm_tx_ctx_event_closed(struct nrf5x_ble_context_s *context,
                             enum event_status_e status)
{
  struct nrf5x_ble_dtm_tx_s *dtm
    = nrf5x_ble_dtm_tx_s_from_context(context);

  logk_trace("cl");
  dtm_tx_schedule(dtm);
}

static
bool_t dtm_tx_ctx_radio_params(struct nrf5x_ble_context_s *context,
                               struct nrf5x_ble_params_s *params)
{
  struct nrf5x_ble_dtm_tx_s *dtm
    = nrf5x_ble_dtm_tx_s_from_context(context);

  if (dtm->channel < 0)
    return 0;

  params->channel = dtm->channel;
  params->access = BLE_DTM_AA;
  params->crc_init = BLE_DTM_CRCINIT;
  params->tx_power = dtm->tx_power;
  params->mode = MODE_TX;
  params->whitening = 0;
  params->rx_rssi = 0;

  return 1;
}

static
uint8_t *dtm_tx_ctx_payload_get(struct nrf5x_ble_context_s *context,
                                enum nrf5x_ble_transfer_e mode)
{
  struct nrf5x_ble_dtm_tx_s *dtm
    = nrf5x_ble_dtm_tx_s_from_context(context);

  channel_set_next(dtm);

  return dtm->pdu;
}

static
void dtm_tx_ctx_ifs_event(struct nrf5x_ble_context_s *context,
                          bool_t timeout)
{
  struct nrf5x_ble_dtm_tx_s *dtm
    = nrf5x_ble_dtm_tx_s_from_context(context);

  (void)dtm;
}

static
void dtm_tx_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                 dev_timer_value_t ts,
                                 int16_t rssi,
                                 bool_t crc_valid)
{
  struct nrf5x_ble_dtm_tx_s *dtm
    = nrf5x_ble_dtm_tx_s_from_context(context);

  (void)dtm;

  return;
}

static
void dtm_tx_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_dtm_tx_s *dtm = nrf5x_ble_dtm_tx_s_from_layer(layer);

  nrf5x_ble_context_cleanup(&dtm->context);

  mem_free(dtm);
}

error_t nrf5x_ble_dtm_tx_create(struct net_scheduler_s *scheduler,
                                struct nrf5x_ble_private_s *priv,
                                const void *params_,
                                void *delegate,
                                const struct net_layer_delegate_vtable_s *delegate_vtable,
                                struct net_layer_s **layer)
{
  const struct ble_dtm_tx_param_s *params = params_;

  if (params->phy != BLE_PHY_1M)
    return -ENOTSUP;

  struct nrf5x_ble_dtm_tx_s *dtm = mem_alloc(sizeof(*dtm), mem_scope_sys);
  error_t err;
  
  if (!dtm)
    return -ENOMEM;

  memset(dtm, 0, sizeof(*dtm));

  logk_trace("Init start");

  err = net_layer_init(&dtm->layer, &ble_dtm_tx_layer_handler.base, scheduler, NULL, NULL);
  if (err)
    goto err_out;

  nrf5x_ble_context_init(priv, &dtm->context, &ble_dtm_tx_ctx_handler);

  dtm_tx_param_update(&dtm->layer, params);

  logk_trace("Init done");

  *layer = &dtm->layer;

  dtm_tx_schedule(dtm);

  return 0;

 err_out:
  mem_free(dtm);

  return err;
}

static
error_t dtm_tx_param_update(struct net_layer_s *layer, const struct ble_dtm_tx_param_s *params)
{
  struct nrf5x_ble_dtm_tx_s *dtm = nrf5x_ble_dtm_tx_s_from_layer(layer);
  uint8_t size = params->size;

  ble_dtm_pdu_fill(params->pattern, dtm->pdu, size);

  dtm->tx_power = params->tx_power;
  dtm->interval_tk = params->interval_ms * 32768 / 1000;
  dtm->delay_max_tk = pow2_m1_up(params->delay_max_ms * 32768 / 1000);
  dtm->channel_map = params->channel_map;

  return 0;
}

static const struct ble_dtm_tx_handler_s ble_dtm_tx_layer_handler = {
  .params_update = dtm_tx_param_update,
  .base.destroyed = dtm_tx_layer_destroyed,
};

static const struct nrf5x_ble_context_handler_s ble_dtm_tx_ctx_handler = {
  .event_opened = dtm_tx_ctx_event_opened,
  .event_closed = dtm_tx_ctx_event_closed,
  .radio_params = dtm_tx_ctx_radio_params,
  .payload_get = dtm_tx_ctx_payload_get,
  .ifs_event = dtm_tx_ctx_ifs_event,
  .payload_received = dtm_tx_ctx_payload_received,
};
