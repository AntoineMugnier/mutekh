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

#include <ble/util/channel_mapper.h>
#include <ble/util/timing_mapper.h>
#include <ble/protocol/error.h>
#include <ble/net/slave.h>
#include <ble/net/layer.h>
#include <ble/net/llcp.h>
#include <ble/net/link.h>
#include <ble/peer.h>
#include "ble.h"
#include "ble_slave.h"

#include <net/scheduler.h>
#include <net/layer.h>
#include <net/task.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define cprintk(...) do{}while(0)
//#define cprintk printk

#define SUPPORTED_FEATURES (0                                  \
                            | (_CONFIG_BLE_CRYPTO << BLE_LL_FEATURE_LE_ENCRYPTION) \
                            | (1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE) \
                            | (1 << BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION) \
                            | (1 << BLE_LL_FEATURE_LE_PING)             \
                            )

#define SLAVE_CONTEXT_UPDATED 1

struct nrf5x_ble_slave_s
{
  struct nrf5x_ble_context_s context;

  const struct ble_slave_handler_s *handler;

  struct net_layer_s *ll;

  struct ble_channel_mapper_s channel_mapper;
  struct ble_timing_mapper_s timing;

  struct net_task_s *conn_params_rq_pending;

  buffer_queue_root_t tx_queue;

  uint8_t tx_queue_count;

  struct net_addr_s src, dst;

  dev_timer_value_t anchor;

  uint32_t access_address;
  uint32_t crc_init;
  int16_t packet_per_event_lp;

  uint16_t last_event_counter;
  uint16_t scheduled_event_counter;

  uint16_t latency_backoff;
  uint16_t since_last_event_intervals;
  uint16_t since_last_event_unit;
  uint16_t to_next_event_unit;

  uint16_t features;

  uint8_t missed_event_count;
  uint8_t event_acked_count;
  uint8_t event_rx_count;
  uint8_t event_tx_count;
  uint8_t tx_per_event_lp;
  uint8_t event_packet_count;
  uint8_t event_crc_error;
  uint8_t event_channel;
  uint8_t reason;

  bool_t established : 1;
  bool_t version_sent : 1;
  bool_t features_sent : 1;
  bool_t peer_md : 1;
  bool_t latency_permitted : 1;
  bool_t event_done : 1;
  bool_t nesn : 1;
  bool_t sn : 1;
  bool_t opened : 1;
  bool_t flow_updating : 1;

  struct ble_link_flow_update_s flow_update;
};

STRUCT_COMPOSE(nrf5x_ble_slave_s, context);
STRUCT_COMPOSE(nrf5x_ble_slave_s, flow_update);

static void nrf5x_ble_slave_flow_updated(void *ptr)
{
  struct net_task_s *task = ptr;
  struct nrf5x_ble_slave_s *slave
    = nrf5x_ble_slave_s_from_flow_update(
        ble_link_flow_update_s_from_task(task));

  slave->flow_updating = 0;
}

static
void nrf5x_ble_slave_error(struct nrf5x_ble_slave_s *slave, uint8_t reason)
{
  const struct ble_slave_delegate_vtable_s *vtable
    = const_ble_slave_delegate_vtable_s_from_base(slave->context.layer.delegate_vtable);

  cprintk("Slave error, reason %d", reason);

  if (slave->reason)
    return;

  slave->reason = reason;

  vtable->connection_lost(slave->context.layer.delegate,
                          &slave->context.layer,
                          reason);
}

static void slave_schedule(struct nrf5x_ble_slave_s *slave)
{
  if (slave->opened)
    return;

  uint32_t event_advance;
  dev_timer_value_t now;
  int16_t until_forced = slave->timing.current.latency + 1 - slave->since_last_event_intervals;
  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;

  slave->event_done = 0;

  event_advance = __MIN(__MAX(until_forced, 1),
                        __MIN(slave->timing.current.latency + 1, slave->latency_backoff));

  if (!slave->established
      || slave->missed_event_count
      || slave->reason
      || slave->peer_md
      || slave->tx_queue_count)
    event_advance = 1;
  else if (slave->timing.update_pending) {
    int16_t to_event = slave->timing.update_instant - slave->last_event_counter;
    if (to_event < event_advance)
      event_advance = to_event;
  }

  cprintk("Slave schedule %d/%d, %03d/%03d/%03d, reason %d",
         slave->last_event_counter, slave->scheduled_event_counter,
         event_advance, slave->latency_backoff, slave->timing.current.latency,
         slave->reason);

  if (slave->scheduled_event_counter - slave->last_event_counter == event_advance) {
    cprintk(" is next\n");
    return;
  }

  assert(event_advance);

 deadline_missed:
  now = nrf5x_ble_rtc_value_get(slave->context.pv);

  if (slave->reason && (!slave->tx_queue_count
                        || slave->since_last_event_intervals + event_advance > 6))
    return;

  slave->scheduled_event_counter = slave->last_event_counter + event_advance;

  cprintk(" event %d (%d + %d)\n", slave->scheduled_event_counter,
         slave->last_event_counter, event_advance);

  ble_timing_mapper_window_slave_get(&slave->timing,
                                     slave->scheduled_event_counter,
                                     &event_begin,
                                     &event_end,
                                     &event_max_duration);

  if (event_begin > slave->timing.drops_at) {
    cprintk(" too late, drops at %lld, scheduled at %lld, %lld too late\n",
            slave->timing.drops_at, event_begin,
            event_begin - slave->timing.drops_at);
    nrf5x_ble_slave_error(slave, BLE_CONNECTION_TIMEOUT);
    return;
  }

  if (event_begin < now) {
    event_advance++;
    cprintk(" deadline_missed %lld < %lld\n", event_begin, now);
    goto deadline_missed;
  }

  slave->latency_permitted = slave->established
    && ((slave->since_last_event_intervals + event_advance) < slave->latency_backoff);

  cprintk(" scheduled event %d at %lld\n",
          slave->scheduled_event_counter,
          event_begin);

  nrf5x_ble_context_schedule(&slave->context, event_begin, event_end, event_max_duration, 0);
}

static const struct nrf5x_ble_context_handler_s ble_slave_ctx_handler;
static const struct net_layer_handler_s ble_slave_layer_handler;

error_t nrf5x_ble_slave_create(struct net_scheduler_s *scheduler,
                               struct nrf5x_ble_private_s *priv,
                               const void *params_,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable,
                               struct net_layer_s **layer)
{
  error_t err;
  struct nrf5x_ble_slave_s *slave;
  const struct ble_slave_param_s *params = params_;
  struct device_timer_s self_as_timer;

  slave = mem_alloc(sizeof(*slave), mem_scope_sys);
  if (!slave)
    return -ENOMEM;

  memset(slave, 0, sizeof(*slave));

  err = device_get_accessor(&self_as_timer, priv->dev, DRIVER_CLASS_TIMER, 0);
  if (err)
    return err;

  err = nrf5x_ble_context_init(&slave->context,
                               scheduler,
                               &ble_slave_layer_handler,
                               priv, 
                               &ble_slave_ctx_handler,
                               delegate, delegate_vtable);
  if (err)
    goto err_put_timer;

  assert(CONFIG_BLE_PACKET_SIZE - 1 >= 33);

  slave->context.layer.context.prefix_size = 1;
  // Default MTU: header (2), l2cap (4), att (23), mic (4)
  // May be negociated with packet length extension.
  slave->context.layer.context.mtu = 33;

  slave->access_address = params->conn_req.access_address;
  slave->crc_init = params->conn_req.crc_init;

  ble_addr_net_set(&params->conn_req.master, &slave->src);
  ble_addr_net_set(&params->conn_req.slave, &slave->dst);

  slave->last_event_counter = -1;
  slave->scheduled_event_counter = -1;
  slave->latency_backoff = 1;

  slave->flow_update.task.destroy_func = nrf5x_ble_slave_flow_updated;

  buffer_queue_init(&slave->tx_queue);
  slave->tx_queue_count = 0;

  ble_channel_mapper_init(&slave->channel_mapper,
                          params->conn_req.channel_map,
                          params->conn_req.hop);

  ble_timing_mapper_init(&slave->timing,
                         &self_as_timer,
                         &params->conn_req,
                         params->connect_packet_timestamp);

  slave->since_last_event_intervals = 0;

  slave_schedule(slave);

  *layer = &slave->context.layer;

 err_put_timer:
  device_put_accessor(&self_as_timer);

  return err;
}

static
void slave_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  buffer_queue_destroy(&slave->tx_queue);
  nrf5x_ble_context_cleanup(&slave->context);
  ble_channel_mapper_cleanup(&slave->channel_mapper);
  ble_timing_mapper_cleanup(&slave->timing);

  mem_free(layer);
}

static void slave_query_task_handle(struct nrf5x_ble_slave_s *slave,
                                    struct net_task_s *task)
{
  switch (task->query.opcode) {
  case BLE_LLCP_CONNECTION_PARAMETERS_UPDATE: {
    struct ble_llcp_connection_parameters_update_s *up
      = ble_llcp_connection_parameters_update_s_from_task(task);

    cprintk("Slave conn params update on event %d\n", up->update.instant);

    net_task_query_respond_push(
      task, ble_timing_mapper_update_push(&slave->timing, &up->update));
    return;
  }

  case BLE_LLCP_CHANNEL_MAP_UPDATE: {
    struct ble_llcp_channel_map_update_s *up
      = ble_llcp_channel_map_update_s_from_task(task);

    cprintk("Slave channel map update on event %d\n", up->instant);

    net_task_query_respond_push(task, ble_channel_mapper_update_push(
      &slave->channel_mapper, up->instant, up->channel_map));
    return;
  }

  default:
    net_task_query_respond_push(task, -ENOTSUP);
    return;
  }
}

static void slave_link_flow_update(struct nrf5x_ble_slave_s *slave)
{
  if (!slave->flow_updating && slave->ll) {
    slave->flow_update.accepted_count
      = (slave->packet_per_event_lp >> 4)
      - slave->tx_queue_count
      + 4;

    slave->flow_updating = 1;
    net_task_notification_push(&slave->flow_update.task, slave->ll, &slave->context.layer,
                               BLE_LINK_FLOW_UPDATE);
  }
}

static
void slave_layer_task_handle(struct net_layer_s *layer,
                             struct net_task_s *task)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  dprintk("%s in %p %p -> %p", __FUNCTION__, slave, task->source, task->target);

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    dprintk(" outbound [%P]",
            task->packet.buffer->data + task->packet.buffer->begin,
            task->packet.buffer->end - task->packet.buffer->begin);

    // We are closing connection, dont enqueue anything
    if (slave->reason)
      break;

    buffer_queue_pushback(&slave->tx_queue, task->packet.buffer);
    slave->tx_queue_count++;
    slave_schedule(slave);
    slave_link_flow_update(slave);
    break;

  case NET_TASK_QUERY:
    dprintk("Slave query, %S\n", &task->query.opcode, 4);

    slave_query_task_handle(slave, task);
    return;

  default:
    dprintk(" other\n");
    break;
  }

  net_task_destroy(task);
}

static
void slave_layer_unbound(struct net_layer_s *layer,
                         struct net_layer_s *child)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  if (child != slave->ll)
    return;

  slave->ll = NULL;

  if (!slave->reason) {
    dprintk("Slave unbound without connection closed\n");

    nrf5x_ble_slave_error(slave, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
  }
}

static error_t slave_layer_bound(struct net_layer_s *layer,
                                 void *addr,
                                 struct net_layer_s *child)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  if (slave->ll)
    return -EBUSY;

  slave->ll = child;

  return 0;
}

static void slave_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  slave->opened = 1;
  slave->event_acked_count = 0;
  slave->event_tx_count = 0;
  slave->event_rx_count = 0;
  slave->event_crc_error = 0;
  slave->event_packet_count = 0;
  slave->event_channel = ble_channel_mapper_chan_get(&slave->channel_mapper,
                                                     slave->scheduled_event_counter);
}

static
void slave_ctx_event_closed(struct nrf5x_ble_context_s *context,
                            enum event_status_e status)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  cprintk("Event %d done: %d, %d pkts, latency %s\n   (%d rx, %d tx, %d acked, %d crc err)\n",
          slave->scheduled_event_counter, status,
          slave->event_packet_count, slave->latency_permitted ? "permitted" : "forbidden",
          slave->event_rx_count,
          slave->event_tx_count, slave->event_acked_count, slave->event_crc_error);

  slave->established |= !!slave->event_acked_count;
  slave->tx_per_event_lp += slave->event_tx_count - (slave->tx_per_event_lp >> 4);

  slave->since_last_event_intervals += (int16_t)(slave->scheduled_event_counter - slave->last_event_counter);
  slave->last_event_counter = slave->scheduled_event_counter;

  slave->packet_per_event_lp += slave->event_acked_count - (slave->packet_per_event_lp >> 4);

  switch (slave->event_packet_count) {
  default:
    // Last time we reached master
    slave->since_last_event_intervals = 0;
    slave->latency_backoff = 2 * slave->latency_backoff;

    goto sync;
  case 1:
  sync:
    ble_channel_mapper_event_set(&slave->channel_mapper, slave->last_event_counter);
    ble_timing_mapper_event_set(&slave->timing, slave->last_event_counter,
                                slave->anchor);

    // In any case, update missed event count, for not dropping the
    // connection too early.
    slave->missed_event_count = 0;
    break;

  case 0:
    slave->missed_event_count++;
    break;
  }

  if (slave->latency_backoff > slave->timing.current.latency)
    slave->latency_backoff = slave->timing.current.latency + 1;

  cprintk(" latency %d, backoff %d\n",
         slave->timing.current.latency, slave->latency_backoff);

  slave->opened = 0;

  slave_link_flow_update(slave);
  slave_schedule(slave);
}

static
bool_t slave_ctx_radio_params(struct nrf5x_ble_context_s *context,
                              struct nrf5x_ble_params_s *params)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  if (slave->event_done || slave->event_crc_error > 2)
    return 0;

  params->access = slave->access_address;
  params->crc_init = slave->crc_init;
  params->channel = slave->event_channel;
  params->tx_power = 0;
  params->mode = (slave->event_packet_count & 1) ? MODE_TX : MODE_RX;
  params->ifs_timeout = slave->event_packet_count != 0;
  params->rx_rssi = 0;

  return (slave->event_packet_count < 2 && !slave->latency_permitted)
    || slave->tx_queue_count
    || slave->peer_md
    || (slave->event_packet_count >= 2 && (slave->event_packet_count & 1));
}

static
struct buffer_s *slave_ctx_payload_get(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);
  struct buffer_s *packet;

  if (slave->tx_queue_count == 0) {
    // Tx queue is empty, we have to create an empty packet to piggyback
    // handshaking.  Cannot borrow reference to a common packet as it is
    // chained in tx queue.
    packet = net_layer_packet_alloc(&slave->context.layer,
                                    slave->context.layer.context.prefix_size, 2);

    if (!packet)
      return NULL;

    packet->data[packet->begin + 0] = BLE_LL_DATA_CONT;
    packet->data[packet->begin + 1] = 0;

    buffer_queue_pushback(&slave->tx_queue, packet);
    slave->tx_queue_count++;
  } else {
    packet = buffer_queue_head(&slave->tx_queue);
  }

  packet->data[packet->begin] = (packet->data[packet->begin] & 0x3)
    // If head packet has data in it, force master to ack it.
    | (packet->data[packet->begin + 1] ? BLE_LL_DATA_MD : 0)
    // Normal MD semantics
    | ((slave->tx_queue_count > 1) ? BLE_LL_DATA_MD : 0)
    | (slave->nesn ? BLE_LL_DATA_NESN : 0)
    | (slave->sn ? BLE_LL_DATA_SN : 0);

  slave->event_tx_count++;

  return packet;
}

static
void slave_ctx_ifs_event(struct nrf5x_ble_context_s *context, bool_t rx_timeout)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  if (rx_timeout)
    slave->event_done = 1;
  else
    slave->event_packet_count++;
}

static
void slave_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                dev_timer_value_t timestamp,
                                bool_t crc_valid,
                                struct buffer_s *packet)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);
  const uint8_t size = packet->end - packet->begin;

  slave->event_rx_count++;

  if (slave->event_packet_count == 1)
    slave->anchor = timestamp;

  /* Dont handle invalid packets (handshaking is lost as well) */
  if (size < 2 || !crc_valid) {
    slave->event_crc_error++;
    return;
  }

  bool_t sn = ble_data_sn_get(packet);
  bool_t nesn = ble_data_nesn_get(packet);
  bool_t md = ble_data_md_get(packet);
  bool_t empty = size == 2 && ble_data_llid_get(packet) == BLE_LL_DATA_CONT;

  if (slave->nesn == sn) {
    if (!empty && slave->ll) {
      struct net_task_s *task = net_scheduler_task_alloc(slave->context.layer.scheduler);
      if (task) {
        net_task_inbound_push(task, slave->ll, &slave->context.layer,
                              timestamp, &slave->src, &slave->dst, packet);
        slave->nesn = !sn;
      }
    } else {
      slave->nesn = !sn;
    }
  }

  slave->peer_md = md;

  if (nesn != slave->sn) {
    slave->event_acked_count++;
    slave->sn = nesn;

    struct buffer_s *packet = buffer_queue_pop(&slave->tx_queue);

    if (packet) {
      slave->tx_queue_count--;
      buffer_refdec(packet);
    }
  }
}

static const struct net_layer_handler_s ble_slave_layer_handler = {
  .destroyed = slave_layer_destroyed,
  .task_handle = slave_layer_task_handle,
  .bound = slave_layer_bound,
  .unbound = slave_layer_unbound,
  .type = BLE_NET_LAYER_SLAVE,
};

static const struct nrf5x_ble_context_handler_s ble_slave_ctx_handler = {
  .event_opened = slave_ctx_event_opened,
  .event_closed = slave_ctx_event_closed,
  .radio_params = slave_ctx_radio_params,
  .payload_get = slave_ctx_payload_get,
  .ifs_event = slave_ctx_ifs_event,
  .payload_received = slave_ctx_payload_received,
};
