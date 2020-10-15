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
#include <ble/net/link.h>
#include <ble/net/llcp.h>
#include <net/layer.h>
#include <net/scheduler.h>
#include <ble/peer.h>
#include "ble.h"

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define cprintk(...) do{}while(0)
//#define cprintk printk

#define MASTER_CONTEXT_UPDATED 1
#define STUCK_EVENTS_MAX 100

struct nrf5x_ble_master_s
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

  uint32_t access_address;
  uint32_t crc_init;

  int16_t packet_per_event_lp;
  uint16_t last_event_counter;
  uint16_t scheduled_event_counter;
  uint16_t since_last_event_intervals;

  uint8_t tx_queue_count;
  uint8_t missed_event_count;
  uint8_t event_acked_count;
  uint8_t event_rx_count;
  uint8_t event_tx_count;
  uint8_t event_packet_count;
  uint8_t event_crc_error;
  uint8_t event_channel;
  uint8_t reason;
  uint8_t stuck_events_left;

  uint8_t empty[2];

  enum ble_phy_mode_e phy:8;
  bool_t established : 1;
  bool_t peer_md : 1;
  bool_t event_done : 1;
  bool_t nesn : 1;
  bool_t sn : 1;
  bool_t opened : 1;
  bool_t flow_updating : 1;
  bool_t empty_first : 1;
  bool_t close_after_flush : 1;

  struct ble_link_flow_update_s flow_update;
};

STRUCT_COMPOSE(nrf5x_ble_master_s, context);
STRUCT_COMPOSE(nrf5x_ble_master_s, flow_update);
STRUCT_COMPOSE(nrf5x_ble_master_s, layer);

static void nrf5x_ble_master_flow_updated(void *ptr)
{
  struct net_task_s *task = ptr;
  struct nrf5x_ble_master_s *master
    = nrf5x_ble_master_s_from_flow_update(
        ble_link_flow_update_s_from_task(task));

  master->flow_updating = 0;
}

static
void nrf5x_ble_master_error(struct nrf5x_ble_master_s *master, uint8_t reason)
{
  const struct ble_phy_delegate_vtable_s *vtable
    = const_ble_phy_delegate_vtable_s_from_base(master->layer.delegate_vtable);

  cprintk("Master error, reason %d", reason);

  if (master->reason)
    return;

  master->reason = reason;

  vtable->connection_lost(master->layer.delegate,
                          &master->layer,
                          reason);
}

static void master_schedule(struct nrf5x_ble_master_s *master)
{
  if (master->opened)
    return;

  if (!net_layer_refcount(&master->layer))
    return;

  uint32_t event_advance;
  dev_timer_value_t now;
  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;

  master->event_done = 0;

  event_advance = 1;

  if (!master->established
      || master->missed_event_count
      || master->reason
      || master->peer_md
      || master->tx_queue_count)
    event_advance = 1;
  else if (master->timing.update_pending) {
    int16_t to_event = master->timing.update_instant - master->last_event_counter;
    if (to_event < event_advance)
      event_advance = to_event;
  }

  cprintk("Master schedule %d/%d, %03d, reason %d",
         master->last_event_counter, master->scheduled_event_counter,
         event_advance,
         master->reason);

  if (master->scheduled_event_counter - master->last_event_counter == event_advance) {
    cprintk(" is next\n");
    return;
  }

  assert(event_advance);

 deadline_missed:
  now = nrf5x_ble_rtc_value_get(master->context.pv);

  if (master->reason && (!master->tx_queue_count
                        || master->since_last_event_intervals + event_advance > 6))
    return;

  master->scheduled_event_counter = master->last_event_counter + event_advance;

  cprintk(" event %d (%d + %d)\n", master->scheduled_event_counter,
         master->last_event_counter, event_advance);

  ble_timing_mapper_window_master_get(&master->timing,
                                     master->scheduled_event_counter,
                                     &event_begin,
                                     &event_end,
                                     &event_max_duration);

  if (event_begin > master->timing.drops_at) {
    cprintk(" too late, drops at %lld, scheduled at %lld, %lld too late\n",
            master->timing.drops_at, event_begin,
            event_begin - master->timing.drops_at);
    nrf5x_ble_master_error(master, BLE_CONNECTION_TIMEOUT);
    return;
  }

  if (event_begin < now) {
    event_advance++;
    cprintk(" deadline_missed %lld < %lld\n", event_begin, now);
    goto deadline_missed;
  }

  cprintk(" scheduled event %d at %lld\n",
          master->scheduled_event_counter,
          event_begin);

  master->anchor = event_begin;

  nrf5x_ble_context_schedule(&master->context, event_begin, event_end, event_max_duration, 1, 0);
}

static const struct nrf5x_ble_context_handler_s ble_master_ctx_handler;
static const struct net_layer_handler_s ble_master_layer_handler;

error_t nrf5x_ble_master_create(struct net_scheduler_s *scheduler,
                               struct nrf5x_ble_private_s *priv,
                               const void *params_,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable,
                               struct net_layer_s **layer)
{
  error_t err;
  struct nrf5x_ble_master_s *master;
  const struct ble_phy_params_s *params = params_;
  struct device_timer_s self_as_timer;

  if (!nrf5x_ble_phy_is_supported(params->phy))
    return -ENOTSUP;
  
  master = mem_alloc(sizeof(*master), mem_scope_sys);
  if (!master)
    return -ENOMEM;

  memset(master, 0, sizeof(*master));

  err = device_get_accessor(&self_as_timer.base, priv->dev, DRIVER_CLASS_TIMER, 0);
  if (err)
    return err;

  err = ble_channel_mapper_init(&master->channel_mapper,
                                params->conn_req.channel_map,
                                params->conn_req.hop);
  if (err)
    goto err_out;

  err = ble_timing_mapper_master_init(&master->timing,
                                      &self_as_timer,
                                      &params->conn_req,
                                      params->connect_packet_timestamp);

  if (err)
    goto err_channel_mapper;

  err = net_layer_init(&master->layer, &ble_master_layer_handler, scheduler, delegate, delegate_vtable);
  if (err)
    goto err_timing_mapper;

  device_put_accessor(&self_as_timer.base);

  nrf5x_ble_context_init(priv, &master->context, &ble_master_ctx_handler);

  assert(CONFIG_BLE_PACKET_SIZE - 1 >= 33);

  master->phy = params->phy;
  master->layer.context.prefix_size = 1;
  // Default MTU: header (2), l2cap (4), att (23), mic (4)
  // May be negociated with packet length extension.
  master->layer.context.mtu = 33;
  ble_addr_net_set(&params->conn_req.master, &master->layer.context.addr);
  master->layer.context.addr.master = 1;

  master->access_address = params->conn_req.access_address;
  master->crc_init = params->conn_req.crc_init;

  ble_addr_net_set(&params->conn_req.master, &master->src);
  ble_addr_net_set(&params->conn_req.master, &master->dst);

  master->last_event_counter = -1;
  master->scheduled_event_counter = -1;

  master->flow_update.task.destroy_func = nrf5x_ble_master_flow_updated;
  master->stuck_events_left = 6;

  buffer_queue_init(&master->tx_queue);
  master->tx_queue_count = 0;

  master->since_last_event_intervals = 0;

  master_schedule(master);

  master->rx_buffer = net_layer_packet_alloc(&master->layer, 1, 0);

  *layer = &master->layer;

  return 0;

 err_timing_mapper:
  ble_timing_mapper_cleanup(&master->timing);
 err_channel_mapper:
  ble_channel_mapper_cleanup(&master->channel_mapper);
 err_out:
  device_put_accessor(&self_as_timer.base);
  free(master);

  return err;
}

static
void master_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_layer(layer);

  if (master->rx_buffer)
    buffer_refdec(master->rx_buffer);
  buffer_queue_destroy(&master->tx_queue);
  nrf5x_ble_context_cleanup(&master->context);
  ble_channel_mapper_cleanup(&master->channel_mapper);
  ble_timing_mapper_cleanup(&master->timing);

  mem_free(master);
}

static void master_query_task_handle(struct nrf5x_ble_master_s *master,
                                    struct net_task_s *task)
{
  switch (task->query.opcode) {
  case BLE_LLCP_CONNECTION_PARAMETERS_UPDATE: {
    struct ble_llcp_connection_parameters_update_s *up
      = ble_llcp_connection_parameters_update_s_from_task(task);

    cprintk("Master conn params update on event %d\n", up->update.instant);

    net_task_query_respond_push(
      task, ble_timing_mapper_update_push(&master->timing, &up->update));
    return;
  }

  case BLE_LLCP_CHANNEL_MAP_UPDATE: {
    struct ble_llcp_channel_map_update_s *up
      = ble_llcp_channel_map_update_s_from_task(task);

    cprintk("Master channel map update on event %d\n", up->instant);

    net_task_query_respond_push(task, ble_channel_mapper_update_push(
      &master->channel_mapper, up->instant, up->channel_map));
    return;
  }

  default:
    net_task_query_respond_push(task, -ENOTSUP);
    return;
  }
}

static void master_link_flow_update(struct nrf5x_ble_master_s *master)
{
  if (!master->flow_updating && master->ll) {
    master->flow_update.accepted_count
      = (master->packet_per_event_lp >> 4)
      - master->tx_queue_count
      + 4;

    master->flow_updating = 1;
    net_task_notification_push(&master->flow_update.task, master->ll, &master->layer,
                               BLE_LINK_FLOW_UPDATE);
  }
}

static
void master_layer_task_handle(struct net_layer_s *layer,
                             struct net_task_s *task)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_layer(layer);

  dprintk("%s in %p %p -> %p", __FUNCTION__, master, task->source, task->target);

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    dprintk(" outbound [%P]",
            task->packet.buffer->data + task->packet.buffer->begin,
            task->packet.buffer->end - task->packet.buffer->begin);

    // We are closing connection, dont enqueue anything
    if (master->reason)
      break;

    if (task->packet.dst_addr.fatal)
      master->close_after_flush = 1;

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    buffer_queue_pushback(&master->tx_queue, task->packet.buffer);
    master->tx_queue_count++;

    assert(master->tx_queue_count == buffer_queue_count(&master->tx_queue));

    CPU_INTERRUPT_RESTORESTATE;
    master_schedule(master);
    //    master_link_flow_update(master);
    break;

  case NET_TASK_QUERY:
    dprintk("Master query, %S\n", &task->query.opcode, 4);

    master_query_task_handle(master, task);
    return;

  default:
    dprintk(" other\n");
    break;
  }

  net_task_destroy(task);
}

static
void master_layer_unbound(struct net_layer_s *layer,
                         struct net_layer_s *child)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_layer(layer);

  if (child != master->ll)
    return;

  master->ll = NULL;

  if (!master->reason) {
    dprintk("Master unbound without connection closed\n");

    nrf5x_ble_master_error(master, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
  }
}

static error_t master_layer_bound(struct net_layer_s *layer,
                                 void *addr,
                                 struct net_layer_s *child)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_layer(layer);

  if (master->ll)
    return -EBUSY;

  master->ll = child;

  return 0;
}

static bool_t master_ctx_event_opened(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_context(context);

  if (!net_layer_refcount(&master->layer))
    return 0;

  master->opened = 1;
  master->event_acked_count = 0;
  master->event_tx_count = 0;
  master->event_rx_count = 0;
  master->event_crc_error = 0;
  master->event_packet_count = 0;
  master->event_channel = ble_channel_mapper_chan_get(&master->channel_mapper,
                                                     master->scheduled_event_counter);

  return 1;
}

static
void master_ctx_event_closed(struct nrf5x_ble_context_s *context,
                            enum event_status_e status)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_context(context);

  cprintk("Event %d done: %d, %d pkts\n   (%d rx, %d tx, %d acked, %d crc err)\n",
          master->scheduled_event_counter, status,
          master->event_packet_count,
          master->event_rx_count,
          master->event_tx_count, master->event_acked_count, master->event_crc_error);

  if (master->event_rx_count && !master->event_acked_count)
    master->stuck_events_left--;
  if (master->stuck_events_left == 0)
    nrf5x_ble_master_error(master, BLE_CONNECTION_TIMEOUT);

  if (master->tx_queue_count == 0 && master->close_after_flush)
    nrf5x_ble_master_error(master, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);

  master->established |= master->event_acked_count > 1;

  master->since_last_event_intervals += (int16_t)(master->scheduled_event_counter - master->last_event_counter);
  master->last_event_counter = master->scheduled_event_counter;

  master->packet_per_event_lp += master->event_acked_count - (master->packet_per_event_lp >> 4);

  ble_channel_mapper_event_set(&master->channel_mapper, master->last_event_counter);
  if (master->event_packet_count > 1)
    ble_timing_mapper_event_set(&master->timing,
                                master->last_event_counter,
                                master->anchor);

  if (!master->rx_buffer)
    master->rx_buffer = net_layer_packet_alloc(&master->layer, 1, 0);

  switch (master->event_packet_count) {
  default:
    // Last time we reached master
    master->since_last_event_intervals = 0;
    master->missed_event_count = 0;
    break;

  case 0:
  case 1:
    master->missed_event_count++;
    break;
  }

  master->opened = 0;

  master_link_flow_update(master);
  master_schedule(master);
}

static
bool_t master_ctx_radio_params(struct nrf5x_ble_context_s *context,
                              struct nrf5x_ble_params_s *params)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_context(context);

  if (master->event_done || master->event_crc_error > 2)
    return 0;

  params->phy = master->phy;
  params->access = master->access_address;
  params->crc_init = master->crc_init;
  params->channel = master->event_channel;
  params->tx_power = 0;
  params->mode = (master->event_packet_count & 1) ? MODE_RX : MODE_TX;
  params->rx_rssi = 0;
  params->whitening = 1;

  return master->event_packet_count < 2
    || master->tx_queue_count
    || master->empty_first
    || master->peer_md;
}

static
uint8_t *master_ctx_payload_get(struct nrf5x_ble_context_s *context,
                               enum nrf5x_ble_transfer_e mode)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_context(context);
  struct buffer_s *packet;

  assert(!cpu_is_interruptible());

  if (mode == MODE_RX) {
    if (!master->rx_buffer)
      return NULL;
    packet = master->rx_buffer;

#if defined(CONFIG_COMPILE_DEBUG)
    memset(packet->data, 0xaa, 8);
#endif

    return packet->data + packet->begin;
  }

  uint8_t *data;
  bool_t md;

  if (master->tx_queue_count == 0 || master->empty_first) {
    // Tx queue is empty, we have to emulate an empty packet to
    // piggyback handshaking.

    master->empty_first = 1;
    master->empty[0] = BLE_LL_DATA_CONT;
    master->empty[1] = 0;
    data = master->empty;
    md = !!master->tx_queue_count;
  } else {
    packet = buffer_queue_head(&master->tx_queue);
    data = packet->data + packet->begin;
    buffer_refdec(packet);
    md = master->tx_queue_count > 1;
  }

  data[0] = (data[0] & 0x3)
    | (md ? BLE_LL_DATA_MD : 0)
    | (master->nesn ? BLE_LL_DATA_NESN : 0)
    | (master->sn ? BLE_LL_DATA_SN : 0);

  master->event_tx_count++;

  return data;
}

static
void master_ctx_ifs_event(struct nrf5x_ble_context_s *context, bool_t timeout)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_context(context);

  if (timeout)
    master->event_done = 1;
  else
    master->event_packet_count++;
}

static
void master_ctx_payload_received(struct nrf5x_ble_context_s *context,
                                dev_timer_value_t timestamp,
                                 int16_t rssi,
                                bool_t crc_valid)
{
  struct nrf5x_ble_master_s *master = nrf5x_ble_master_s_from_context(context);
  struct buffer_s *packet = master->rx_buffer;
  uint16_t size = __MIN(CONFIG_BLE_PACKET_SIZE,
                        packet->data[packet->begin + 1] + 2);
  struct buffer_s *next_rx_packet;
  struct net_task_s *task;

  assert(!cpu_is_interruptible());

  packet->end = packet->begin + size;
  master->event_rx_count++;

  /* Dont handle invalid packets (handshaking is lost as well) */
  if (size < 2 || !crc_valid) {
    master->event_crc_error++;
    return;
  }

  bool_t sn = ble_data_sn_get(packet);
  bool_t nesn = ble_data_nesn_get(packet);
  bool_t empty = size == 2 && ble_data_llid_get(packet) == BLE_LL_DATA_CONT;
  master->peer_md = ble_data_md_get(packet);

  if (nesn != master->sn) {
    master->event_acked_count++;
    master->sn = nesn;
    master->stuck_events_left = STUCK_EVENTS_MAX;

    if (master->empty_first) {
      master->empty_first = 0;
    } else {
      struct buffer_s *packet = buffer_queue_pop(&master->tx_queue);

      if (packet) {
        master->tx_queue_count--;
        buffer_refdec(packet);
      }
    }
  }

  if (master->nesn != sn)
    return;

  if (empty || !master->ll)
    goto accept;

  next_rx_packet = net_layer_packet_alloc(&master->layer, 1, 0);
  if (!next_rx_packet)
    return;

  task = net_scheduler_task_alloc(master->layer.scheduler);
  if (!task) {
    buffer_refdec(next_rx_packet);
    return;
  }

  net_task_inbound_push(task, master->ll, &master->layer,
                        timestamp, &master->src, &master->dst, packet);
  buffer_refdec(packet);
  master->rx_buffer = next_rx_packet;

 accept:
  master->nesn = !sn;
}

static const struct net_layer_handler_s ble_master_layer_handler = {
  .destroyed = master_layer_destroyed,
  .task_handle = master_layer_task_handle,
  .bound = master_layer_bound,
  .unbound = master_layer_unbound,
};

static const struct nrf5x_ble_context_handler_s ble_master_ctx_handler = {
  .event_opened = master_ctx_event_opened,
  .event_closed = master_ctx_event_closed,
  .radio_params = master_ctx_radio_params,
  .payload_get = master_ctx_payload_get,
  .ifs_event = master_ctx_ifs_event,
  .payload_received = master_ctx_payload_received,
};
