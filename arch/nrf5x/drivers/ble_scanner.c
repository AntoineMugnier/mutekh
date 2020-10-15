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
#include <ble/net/scanner.h>

#include <net/scheduler.h>
#include <net/layer.h>
#include <net/task.h>

#include "ble.h"

#define dprintk(...) do{}while(0)
//#define dprintk printk

static
error_t scan_param_update(struct net_layer_s *layer, const struct ble_scanner_param_s *params);

static const struct ble_scanner_handler_s ble_scanner_layer_handler;
static const struct nrf5x_ble_context_handler_s ble_scanner_ctx_handler;

enum scan_state_e {
  SCAN_IND,
  SCAN_REQ,
  SCAN_RSP,
  SCAN_CONNECT,
  SCAN_DONE,
};

struct nrf5x_ble_scanner_s
{
  struct nrf5x_ble_context_s context;
  struct net_layer_s layer;

  dev_timer_value_t last_start;

  uint32_t interval_tk;
  uint32_t duration_tk;

  struct net_layer_s *client;

  struct buffer_s *rx_buffer;
  struct buffer_s *tx_buffer;
  struct ble_adv_connect_s conn_params;
  dev_timer_value_t conn_ts;

  struct ble_gap_preferred_conn_params_s timing;

  enum scan_state_e state;
  uint8_t channel;

  enum ble_scanner_policy_e default_policy;

  size_t target_count;
  struct ble_scanner_target_s target[BLE_SCANNER_TARGET_MAXCOUNT];
};

STRUCT_COMPOSE(nrf5x_ble_scanner_s, context);
STRUCT_COMPOSE(nrf5x_ble_scanner_s, layer);

static void scanner_schedule(struct nrf5x_ble_scanner_s *scan)
{
  dev_timer_value_t now, begin, end;

  if (!net_layer_refcount(&scan->layer)) {
    dprintk("%s Scanner unreffed\n", __FUNCTION__);
    return;
  }

  now = nrf5x_ble_rtc_value_get(scan->context.pv);

  if (now < scan->last_start + scan->duration_tk) {
    begin = now;
    end = scan->last_start + scan->duration_tk;
  } else {
    if (scan->last_start + scan->interval_tk < now) {
      begin = now;
      end = now + scan->duration_tk;
    } else {
      begin = scan->last_start + scan->interval_tk;
      end = begin + scan->duration_tk;
    }

    scan->last_start = begin;

    scan->channel++;
    if (scan->channel > 39)
      scan->channel = 37;
  }

  nrf5x_ble_context_schedule(&scan->context, begin, end, 0, 0, 10);
}

static bool_t scanner_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_scanner_s *scan = nrf5x_ble_scanner_s_from_context(context);

  if (!net_layer_refcount(&scan->layer)) {
    dprintk("%s Scanner unreffed\n", __FUNCTION__);
    return 0;
  }

  scan->state = SCAN_IND;

  /* TODO: Do something useful with timings depending on other layers
     alive in the radio
  */

  scan->conn_params.timing.interval = scan->timing.interval_min;
  scan->conn_params.timing.latency = scan->timing.latency;
  // Convert from conn units (1.25ms) to timeout units (10ms) == 8x more
  scan->conn_params.timing.timeout = __MAX(
    (scan->conn_params.timing.interval + 7) / 8 * 2 * (scan->timing.latency + 1),
    scan->timing.timeout);
  // Should probably share channel quality measurements...
  scan->conn_params.channel_map = (1ull << 37) - 1;
  scan->conn_params.hop = 16;
  // 
  scan->conn_params.sca = scan->context.pv->sca;
  // Should schedule where not colliding
  scan->conn_params.win_size = 3;
  scan->conn_params.win_offset = 5;

  return 1;
}

static void scanner_ctx_event_closed(struct nrf5x_ble_context_s *context,
                                        enum event_status_e status)
{
  struct nrf5x_ble_scanner_s *scan
    = nrf5x_ble_scanner_s_from_context(context);
  const struct ble_scanner_delegate_vtable_s *vtable
    = const_ble_scanner_delegate_vtable_s_from_base(scan->layer.delegate_vtable);
  bool_t reschedule = 1;

  if (scan->conn_ts) {
    reschedule = vtable->connection_requested(scan->layer.delegate,
                                              &scan->layer,
                                              &scan->conn_params,
                                              scan->conn_ts);
    scan->conn_ts = 0;
  }

  if (!scan->rx_buffer)
    scan->rx_buffer = net_layer_packet_alloc(&scan->layer, 0, 0);

  if (reschedule)
    scanner_schedule(scan);
}

static bool_t scanner_ctx_radio_params(struct nrf5x_ble_context_s *context,
                                          struct nrf5x_ble_params_s *params)
{
  struct nrf5x_ble_scanner_s *scan
    = nrf5x_ble_scanner_s_from_context(context);

  if (scan->conn_ts)
    return 0;

  params->channel = scan->channel;
  params->access = BLE_ADVERTISE_AA;
  params->crc_init = BLE_ADVERTISE_CRCINIT;
  params->tx_power = 0;
  params->rx_rssi = 0;

  switch (scan->state) {
  case SCAN_IND:
    params->mode = MODE_RX;
    params->rx_rssi = 1;
    return 1;

  case SCAN_RSP:
    params->mode = MODE_RX;
    params->rx_rssi = 1;
    return 1;

  case SCAN_REQ:
  case SCAN_CONNECT:
    params->mode = MODE_TX;
    return 1;

  case SCAN_DONE:
    return 0;
  }

  return 0;
}

static uint8_t *scanner_ctx_payload_get(struct nrf5x_ble_context_s *context,
                                           enum nrf5x_ble_transfer_e mode)
{
  struct nrf5x_ble_scanner_s *scan
    = nrf5x_ble_scanner_s_from_context(context);

  switch (scan->state) {
  case SCAN_IND:
  case SCAN_RSP:
    assert(mode == MODE_RX);
    if (!scan->rx_buffer)
      return NULL;
    return scan->rx_buffer->data + scan->rx_buffer->begin;
  case SCAN_REQ:
    assert(mode == MODE_TX);

    if (!scan->tx_buffer)
      return NULL;

    ble_adv_scan_req_set(scan->tx_buffer, &scan->conn_params.master, &scan->conn_params.slave);

    return scan->tx_buffer->data + scan->tx_buffer->begin;

  case SCAN_CONNECT:
    assert(mode == MODE_TX);

    if (!scan->tx_buffer)
      return NULL;

    ble_adv_connect_set(scan->tx_buffer, &scan->conn_params);

    return scan->tx_buffer->data + scan->tx_buffer->begin;

  default:
    return NULL;
  }
}

static void scanner_ctx_ifs_event(struct nrf5x_ble_context_s *context,
                                  bool_t timeout)
{
  struct nrf5x_ble_scanner_s *scan
    = nrf5x_ble_scanner_s_from_context(context);

  if (timeout) {
    scan->state = SCAN_IND;
    return;
  }

  switch (scan->state) {
  case SCAN_IND:
    scan->state = SCAN_REQ;
    break;

  case SCAN_REQ:
    scan->state = SCAN_RSP;
    break;

  case SCAN_RSP:
    scan->state = SCAN_IND;
    break;

  case SCAN_CONNECT:
    scan->state = SCAN_DONE;
    scan->conn_ts = nrf5x_ble_rtc_value_get(context->pv);
    break;

  default:
    scan->state = SCAN_DONE;
    break;
  }
}

static enum ble_scanner_policy_e scanner_policy_get(struct nrf5x_ble_scanner_s *scan, const struct ble_addr_s *addr)
{
  for (size_t i = 0; i < scan->target_count; ++i) {
    /* printk("Policy ? "BLE_ADDR_FMT" - "BLE_ADDR_FMT"\n", */
    /*        BLE_ADDR_ARG(addr), BLE_ADDR_ARG(&scan->target[i].addr)); */
    if (!ble_addr_cmp(addr, &scan->target[i].addr))
      return scan->target[i].policy;
  }

  return scan->default_policy;
}

static void scanner_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                         dev_timer_value_t ts,
                                            int16_t rssi,
                                         bool_t crc_valid)
{
  struct nrf5x_ble_scanner_s *scan
    = nrf5x_ble_scanner_s_from_context(context);
  enum ble_scanner_policy_e policy = BLE_SCANNER_IGNORE;
  struct ble_addr_s adva;
  const uint8_t size = __MIN(CONFIG_BLE_PACKET_SIZE,
                             scan->rx_buffer->data[scan->rx_buffer->begin + 1] + 2);

  if (!crc_valid || size < 8) {
    scan->state = SCAN_IND;
    return;
  }

  scan->rx_buffer->end = scan->rx_buffer->begin + size;
  ble_advertise_packet_txaddr_get(scan->rx_buffer, &adva);

  switch (ble_advertise_packet_type_get(scan->rx_buffer)) {
  case BLE_ADV_IND:
    policy = scanner_policy_get(scan, &adva);
    break;

  case BLE_SCAN_RSP:
    policy = scanner_policy_get(scan, &adva) & ~BLE_SCANNER_SCAN;
    break;

  case BLE_ADV_DIRECT_IND:
    policy = scanner_policy_get(scan, &adva) & BLE_SCANNER_CONNECT;
    break;

  default:
    policy = 0;
    break;
  }

  if (policy & BLE_SCANNER_CONNECT) {
    scan->conn_params.slave = adva;
    scan->state = SCAN_CONNECT;
    return;
  }

  if (policy & BLE_SCANNER_SCAN) {
    scan->conn_params.slave = adva;
    scan->state = SCAN_REQ;
    return;
  }

  if (scan->client) {
    struct buffer_s *buffer;
    struct net_task_s *task;
    struct net_addr_s src = {
      .rssi = rssi,
    };

    buffer = net_layer_packet_alloc(&scan->layer, 1, 0);
    if (buffer) {
      task = net_scheduler_task_alloc(scan->layer.scheduler);
      if (task) {
        net_task_inbound_push(task, scan->client, &scan->layer,
                              ts, &src, NULL, scan->rx_buffer);
        buffer_refdec(scan->rx_buffer);
        scan->rx_buffer = buffer;
      } else {
        buffer_refdec(buffer);
      }
    }
  }

  scan->state = SCAN_IND;
}

static
void scanner_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_scanner_s *scan = nrf5x_ble_scanner_s_from_layer(layer);

  dprintk("%s\n", __FUNCTION__);

  nrf5x_ble_context_cleanup(&scan->context);
  if (scan->rx_buffer)
    buffer_refdec(scan->rx_buffer);
  if (scan->tx_buffer)
    buffer_refdec(scan->tx_buffer);

  mem_free(scan);
}

error_t nrf5x_ble_scanner_create(struct net_scheduler_s *scheduler,
                              struct nrf5x_ble_private_s *priv,
                              const void *params_,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable_,
                              struct net_layer_s **layer)
{
  const struct ble_scanner_param_s *params = params_;
  struct nrf5x_ble_scanner_s *scan = mem_alloc(sizeof(*scan), mem_scope_sys);
  error_t err;

  if (params->phy != BLE_PHY_1M)
    return -EINVAL;

  if (!scan)
    return -ENOMEM;

  memset(scan, 0, sizeof(*scan));

  dprintk("%s Scanner init start\n", __FUNCTION__);

  err = net_layer_init(&scan->layer, &ble_scanner_layer_handler.base, scheduler, delegate, delegate_vtable_);
  if (err)
    goto err_out;

  nrf5x_ble_context_init(priv, &scan->context, &ble_scanner_ctx_handler);

  scan->tx_buffer = net_layer_packet_alloc(&scan->layer, 0, 0);
  scan->rx_buffer = net_layer_packet_alloc(&scan->layer, 0, 0);
  scan->last_start = 0;
  scan->channel = 37;

  scan_param_update(&scan->layer, params);

  scanner_schedule(scan);

  dprintk("%s Scanner init done\n", __FUNCTION__);

  *layer = &scan->layer;

  return 0;

 err_out:
  mem_free(scan);

  return err;
}

static
error_t scan_param_update(struct net_layer_s *layer, const struct ble_scanner_param_s *params)
{
  struct nrf5x_ble_scanner_s *scan = nrf5x_ble_scanner_s_from_layer(layer);

  if (params->phy != BLE_PHY_1M)
    return -ENOTSUP;
  
  scan->interval_tk = params->interval_ms * 32768 / 1000;
  scan->duration_tk = params->duration_ms * 32768 / 1000;
  scan->default_policy = params->default_policy;
  scan->target_count = __MIN(params->target_count, BLE_SCANNER_TARGET_MAXCOUNT);
  for (size_t i = 0; i < scan->target_count; ++i) {
    scan->target[i].addr = params->target[i].addr;
    scan->target[i].policy = params->target[i].policy;
  }

  scan->timing = params->timing;

  scan->conn_params.master = params->local_addr;
  scan->conn_params.access_address = params->access_address;
  scan->conn_params.crc_init = params->crc_init;

  dprintk("%s scan interval: %d, duration: %d\n",
          __FUNCTION__,
          scan->interval_tk, scan->duration_tk);

  return 0;
}

static
void scan_layer_unbound(struct net_layer_s *layer,
                         struct net_layer_s *child)
{
  struct nrf5x_ble_scanner_s *scan = nrf5x_ble_scanner_s_from_layer(layer);

  if (child != scan->client)
    return;

  scan->client = NULL;
}

static error_t scan_layer_bound(struct net_layer_s *layer,
                                 void *addr,
                                 struct net_layer_s *child)
{
  struct nrf5x_ble_scanner_s *scan = nrf5x_ble_scanner_s_from_layer(layer);

  if (scan->client)
    return -EBUSY;

  scan->client = child;

  return 0;
}

static const struct ble_scanner_handler_s ble_scanner_layer_handler = {
  .params_update = scan_param_update,
  .base.bound = scan_layer_bound,
  .base.unbound = scan_layer_unbound,
  .base.destroyed = scanner_layer_destroyed,
};

static const struct nrf5x_ble_context_handler_s ble_scanner_ctx_handler = {
  .event_opened = scanner_ctx_event_opened,
  .event_closed = scanner_ctx_event_closed,
  .radio_params = scanner_ctx_radio_params,
  .payload_get = scanner_ctx_payload_get,
  .ifs_event = scanner_ctx_ifs_event,
  .payload_received = scanner_ctx_payload_received,
};
