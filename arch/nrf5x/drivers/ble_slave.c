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
#include <ble/net/phy.h>
#include <ble/net/llcp.h>
#include <ble/net/link.h>
#include <ble/peer.h>
#include "ble.h"

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
#define STUCK_EVENTS_MAX 100

struct nrf5x_ble_slave_s
{
  struct nrf5x_ble_context_s context;
  struct net_layer_s layer;

  buffer_queue_root_t tx_queue;

  struct ble_channel_mapper_s channel_mapper;
  struct ble_timing_mapper_s timing;
  struct net_addr_s src, dst;

  struct net_layer_s *ll;

  struct buffer_s *rx_buffer;

  dev_timer_value_t anchor;
  net_task_queue_root_t tx_pending;

  uint32_t access_address;
  uint32_t crc_init;

  int16_t packet_per_event_lp;
  uint16_t last_event_counter;
  uint16_t scheduled_event_counter;
  uint16_t latency_backoff;
  uint16_t since_last_event_intervals;

  uint8_t tx_queue_count;
  uint8_t missed_event_count;
  uint8_t event_acked_count;
  uint8_t event_rx_count;
  uint8_t event_rx_data_acked_count;
  uint8_t event_tx_count;
  uint8_t event_packet_count;
  uint8_t event_crc_error;
  uint8_t event_channel;
  uint8_t reason;
  uint8_t stuck_events_left;

  uint8_t empty[2];

  bool_t established : 1;
  bool_t peer_md : 1;
  bool_t latency_permitted : 1;
  bool_t event_done : 1;
  bool_t nesn : 1;
  bool_t sn : 1;
  bool_t opened : 1;
  bool_t flow_updating : 1;
  bool_t empty_first : 1;
  bool_t close_after_flush : 1;

  struct ble_link_flow_update_s flow_update;
};

STRUCT_COMPOSE(nrf5x_ble_slave_s, context);
STRUCT_COMPOSE(nrf5x_ble_slave_s, flow_update);
STRUCT_COMPOSE(nrf5x_ble_slave_s, layer);

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
  const struct ble_phy_delegate_vtable_s *vtable
    = const_ble_phy_delegate_vtable_s_from_base(slave->layer.delegate_vtable);

  dprintk("Slave error, reason %d\n", reason);

  if (slave->reason)
    return;

  slave->reason = reason;

  vtable->connection_lost(slave->layer.delegate,
                          &slave->layer,
                          reason);
}

static void slave_schedule(struct nrf5x_ble_slave_s *slave)
{
  if (slave->opened)
    return;

  dprintk("%s %d\n", __FUNCTION__, net_layer_refcount(&slave->layer));

  if (!net_layer_refcount(&slave->layer))
    return;

  uint32_t event_advance;
  dev_timer_value_t now;
  int16_t until_forced = slave->timing.current.latency + 1 - slave->since_last_event_intervals;
  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;

  slave->event_done = 0;

  if (slave->tx_queue_count)
    slave->latency_backoff = 1;

  event_advance = __MIN(__MAX(until_forced, 1),
                        __MIN(slave->timing.current.latency + 1, slave->latency_backoff));

  if (!slave->latency_permitted
      || !slave->established
      || !slave->event_acked_count
      || slave->missed_event_count
      || slave->empty_first
      || slave->reason
      || slave->peer_md)
    event_advance = 1;
  else if (slave->timing.update_pending) {
    int16_t to_event = slave->timing.update_instant - slave->last_event_counter;
    if (to_event < event_advance)
      event_advance = to_event;
  }

  cprintk("Slave schedule %d/%d, %d %03d/%03d/%03d, reason %d",
         slave->last_event_counter, slave->scheduled_event_counter,
          until_forced,
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
    && !slave->tx_queue_count
    && !slave->peer_md
    && ((slave->since_last_event_intervals + event_advance) < slave->latency_backoff);

  cprintk(" slave latency permit: %s, txq: %d, md: %d, %d + %d < %d: %d\n",
         slave->established ? "established" : "not yet",
          slave->tx_queue_count,
          slave->peer_md,
         slave->since_last_event_intervals,
         event_advance,
         slave->latency_backoff,
         slave->latency_permitted);

  cprintk(" scheduled event %d at %lld\n",
          slave->scheduled_event_counter,
          event_begin);

  nrf5x_ble_context_schedule(&slave->context, event_begin, event_end, event_max_duration, 1, 0);
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
  const struct ble_phy_params_s *params = params_;
  struct device_timer_s self_as_timer;

  slave = mem_alloc(sizeof(*slave), mem_scope_sys);
  if (!slave)
    return -ENOMEM;

  memset(slave, 0, sizeof(*slave));

  err = device_get_accessor(&self_as_timer.base, priv->dev, DRIVER_CLASS_TIMER, 0);
  if (err)
    return err;

  err = ble_channel_mapper_init(&slave->channel_mapper,
                                params->conn_req.channel_map,
                                params->conn_req.hop);
  if (err)
    goto err_out;

  err = ble_timing_mapper_slave_init(&slave->timing,
                                     &self_as_timer,
                                     &params->conn_req,
                                     params->connect_packet_timestamp);

  if (err)
    goto err_channel_mapper;

  err = net_layer_init(&slave->layer, &ble_slave_layer_handler, scheduler, delegate, delegate_vtable);
  if (err)
    goto err_timing_mapper;

  device_put_accessor(&self_as_timer.base);

  nrf5x_ble_context_init(priv, &slave->context, &ble_slave_ctx_handler);

  assert(CONFIG_BLE_PACKET_SIZE - 1 >= 33);

  slave->layer.context.prefix_size = 1;
  // Default MTU: header (2), l2cap (4), att (23), mic (4)
  // May be negociated with packet length extension.
  slave->layer.context.mtu = 33;
  ble_addr_net_set(&params->conn_req.master, &slave->layer.context.addr);
  slave->layer.context.addr.master = 0;

  slave->access_address = params->conn_req.access_address;
  slave->crc_init = params->conn_req.crc_init;

  ble_addr_net_set(&params->conn_req.master, &slave->src);
  ble_addr_net_set(&params->conn_req.slave, &slave->dst);

  slave->last_event_counter = -1;
  slave->scheduled_event_counter = -1;
  slave->latency_backoff = 1;

  slave->flow_update.task.destroy_func = nrf5x_ble_slave_flow_updated;
  slave->stuck_events_left = 6;

  net_task_queue_init(&slave->tx_pending);
  buffer_queue_init(&slave->tx_queue);
  slave->tx_queue_count = 0;

  slave->since_last_event_intervals = 0;

  slave_schedule(slave);

  slave->rx_buffer = net_layer_packet_alloc(&slave->layer, 1, 0);

  *layer = &slave->layer;

  return 0;

 err_timing_mapper:
  ble_timing_mapper_cleanup(&slave->timing);
 err_channel_mapper:
  ble_channel_mapper_cleanup(&slave->channel_mapper);
 err_out:
  device_put_accessor(&self_as_timer.base);
  free(slave);

  return err;
}

static
void slave_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_layer(layer);

  dprintk("%s\n", __FUNCTION__);

  while (slave->tx_queue_count < 8) {
    struct net_task_s *task = net_task_queue_pop(&slave->tx_pending);

    if (!task)
      break;

    net_task_destroy(task);
  }

  if (slave->rx_buffer)
    buffer_refdec(slave->rx_buffer);
  buffer_queue_destroy(&slave->tx_queue);
  net_task_queue_destroy(&slave->tx_pending);
  nrf5x_ble_context_cleanup(&slave->context);
  ble_channel_mapper_cleanup(&slave->channel_mapper);
  ble_timing_mapper_cleanup(&slave->timing);

  mem_free(slave);
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
    net_task_notification_push(&slave->flow_update.task, slave->ll, &slave->layer,
                               BLE_LINK_FLOW_UPDATE);
  }
}

static
void slave_layer_task_handle(struct net_layer_s *layer,
                             struct net_task_s *task)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_layer(layer);
  bool_t kept = 0;

  dprintk("%s in %p %p -> %p", __FUNCTION__, slave, task->source, task->target);

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    dprintk(" outbound [%P]",
            task->packet.buffer->data + task->packet.buffer->begin,
            task->packet.buffer->end - task->packet.buffer->begin);

    // We are closing connection, dont enqueue anything
    if (slave->reason)
      break;

    if (task->packet.dst_addr.fatal)
      slave->close_after_flush = 1;

    CPU_INTERRUPT_SAVESTATE_DISABLE;

    if (slave->tx_queue_count > 8) {
      net_task_queue_pushback(&slave->tx_pending, task);
      kept = 1;
    } else {
      buffer_queue_pushback(&slave->tx_queue, task->packet.buffer);
      slave->tx_queue_count++;
    }
    CPU_INTERRUPT_RESTORESTATE;

    slave_schedule(slave);
    if (kept) {
      // Dont destroy task
      return;
    }
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
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_layer(layer);

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
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_layer(layer);

  if (slave->ll)
    return -EBUSY;

  slave->ll = child;

  return 0;
}

static bool_t slave_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  if (!net_layer_refcount(&slave->layer))
    return 0;

  dprintk("%s\n", __FUNCTION__);

  slave->opened = 1;
  slave->event_acked_count = 0;
  slave->event_tx_count = 0;
  slave->event_rx_count = 0;
  slave->event_rx_data_acked_count = 0;
  slave->event_crc_error = 0;
  slave->event_packet_count = 0;
  slave->event_channel = ble_channel_mapper_chan_get(&slave->channel_mapper,
                                                     slave->scheduled_event_counter);

  return 1;
}

static
void slave_ctx_event_closed(struct nrf5x_ble_context_s *context,
                            enum event_status_e status)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  dprintk("%s\n", __FUNCTION__);

  cprintk("Event %d done: %d, %d pkts, latency %s\n   (%d rx, %d data, %d tx, %d acked, %d crc err)\n",
          slave->scheduled_event_counter, status,
          slave->event_packet_count, slave->latency_permitted ? "permitted" : "forbidden",
          slave->event_rx_count, slave->event_rx_data_acked_count,
          slave->event_tx_count, slave->event_acked_count, slave->event_crc_error);

  assert(slave->tx_queue_count == buffer_queue_count(&slave->tx_queue));

  while (slave->tx_queue_count < 8) {
    struct net_task_s *task = net_task_queue_pop(&slave->tx_pending);

    if (!task)
      break;

    buffer_queue_pushback(&slave->tx_queue, task->packet.buffer);
    slave->tx_queue_count++;
    net_task_destroy(task);
  }
  
  if (slave->event_packet_count && !slave->event_acked_count)
    slave->stuck_events_left--;
  if (slave->stuck_events_left == 0)
    nrf5x_ble_slave_error(slave, BLE_RESPONSE_TIMEOUT);

  if (slave->tx_queue_count == 0 && slave->close_after_flush)
    nrf5x_ble_slave_error(slave, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);

  slave->established |= !!slave->event_acked_count;

  slave->since_last_event_intervals += (int16_t)(slave->scheduled_event_counter - slave->last_event_counter);
  slave->last_event_counter = slave->scheduled_event_counter;

  slave->packet_per_event_lp += slave->event_acked_count - (slave->packet_per_event_lp >> 4);

  if (!slave->rx_buffer)
    slave->rx_buffer = net_layer_packet_alloc(&slave->layer, 1, 0);

  switch (slave->event_packet_count) {
  default:
    // Last time we reached master
    if (slave->event_rx_data_acked_count)
      slave->latency_backoff = 1;
    else
      slave->latency_backoff = 2 * slave->latency_backoff;

    if (slave->event_rx_count > slave->event_crc_error)
      slave->since_last_event_intervals = 0;

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
  params->rx_rssi = 0;

  return !(slave->event_rx_count - slave->event_crc_error < 1 && slave->latency_permitted)
    || slave->tx_queue_count
    || slave->empty_first
    || slave->peer_md
    || (slave->event_packet_count >= 2 && (slave->event_packet_count & 1));
}

static
uint8_t *slave_ctx_payload_get(struct nrf5x_ble_context_s *context,
                               enum nrf5x_ble_transfer_e mode)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);
  struct buffer_s *packet;

  assert(!cpu_is_interruptible());

  if (mode == MODE_RX) {
    if (!slave->rx_buffer)
      return NULL;
    packet = slave->rx_buffer;

#if defined(CONFIG_COMPILE_DEBUG)
    memset(packet->data, 0xaa, 8);
#endif

    return packet->data + packet->begin;
  }

  uint8_t *data;
  bool_t md;

  if (slave->latency_permitted
      && slave->tx_queue_count == 0
      && !slave->empty_first
      && !slave->peer_md) {
    slave->event_done = 1;
    return NULL;
  }

  if (slave->tx_queue_count == 0 || slave->empty_first) {
    // Tx queue is empty, we have to emulate an empty packet to
    // piggyback handshaking.

    slave->empty_first = 1;
    slave->empty[0] = BLE_LL_DATA_CONT;
    slave->empty[1] = 0;
    data = slave->empty;
    md = !!slave->tx_queue_count;
    assert(data[1] == 0);
  } else {
    packet = buffer_queue_head(&slave->tx_queue);
    assert(packet);
    data = packet->data + packet->begin;
    assert(data[1] + 2 == packet->end - packet->begin);
    buffer_refdec(packet);
    // Lie about MD: also tell we have more data to send when only one
    // packet is in queue, this way, master acknowledges early, and we
    // can apply slave latency on subsequent event.
    md = (slave->tx_queue_count && slave->rx_buffer) || !net_task_queue_isempty(&slave->tx_pending);
  }

  data[0] = (data[0] & 0x3)
    | (md ? BLE_LL_DATA_MD : 0)
    | (slave->nesn ? BLE_LL_DATA_NESN : 0)
    | (slave->sn ? BLE_LL_DATA_SN : 0);

  assert((data[0] & 3) != 0);

  slave->event_tx_count++;

  return data;
}

static
void slave_ctx_ifs_event(struct nrf5x_ble_context_s *context, bool_t timeout)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);

  if (timeout)
    slave->event_done = 1;
  else
    slave->event_packet_count++;
}

static
void slave_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                dev_timer_value_t timestamp,
                                            int16_t rssi,
                                bool_t crc_valid)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);
  struct buffer_s *packet = slave->rx_buffer;
  uint16_t size = __MIN(CONFIG_BLE_PACKET_SIZE,
                        packet->data[packet->begin + 1] + 2);
  struct buffer_s *next_rx_packet;
  struct net_task_s *task;

  assert(!cpu_is_interruptible());

  packet->end = packet->begin + size;
  slave->event_rx_count++;

  if (slave->event_packet_count == 1)
    slave->anchor = timestamp;

  /* Dont handle invalid packets (handshaking is lost as well) */
  if (!crc_valid) {
    slave->event_crc_error++;
    return;
  }

  bool_t sn = ble_data_sn_get(packet);
  bool_t nesn = ble_data_nesn_get(packet);
  bool_t empty = size == 2 && ble_data_llid_get(packet) == BLE_LL_DATA_CONT;
  slave->peer_md = ble_data_md_get(packet);

  if (nesn != slave->sn) {
    slave->event_acked_count++;
    slave->sn = nesn;
    slave->stuck_events_left = STUCK_EVENTS_MAX;

    if (slave->empty_first) {
      slave->empty_first = 0;
    } else {
      struct buffer_s *packet = buffer_queue_pop(&slave->tx_queue);

      if (packet) {
        slave->tx_queue_count--;
        buffer_refdec(packet);
      }
    }
  }

  if (slave->nesn != sn)
    return;

  if (empty || !slave->ll)
    goto accept;

  next_rx_packet = net_layer_packet_alloc(&slave->layer, 1, 0);
  if (!next_rx_packet)
    return;

  task = net_scheduler_task_alloc(slave->layer.scheduler);
  if (!task) {
    buffer_refdec(next_rx_packet);
    return;
  }

  net_task_inbound_push(task, slave->ll, &slave->layer,
                        timestamp, &slave->src, &slave->dst, packet);
  buffer_refdec(packet);
  slave->rx_buffer = next_rx_packet;
  slave->event_rx_data_acked_count++;

 accept:
  slave->nesn = !sn;
}

static const struct net_layer_handler_s ble_slave_layer_handler = {
  .destroyed = slave_layer_destroyed,
  .task_handle = slave_layer_task_handle,
  .bound = slave_layer_bound,
  .unbound = slave_layer_unbound,
};

static const struct nrf5x_ble_context_handler_s ble_slave_ctx_handler = {
  .event_opened = slave_ctx_event_opened,
  .event_closed = slave_ctx_event_closed,
  .radio_params = slave_ctx_radio_params,
  .payload_get = slave_ctx_payload_get,
  .ifs_event = slave_ctx_ifs_event,
  .payload_received = slave_ctx_payload_received,
};
