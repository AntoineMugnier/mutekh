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

#include <ble/net/master.h>
#include <ble/net/l2cap.h>
#include <ble/net/gap.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/data.h>

#define F_CHANNEL_MAP_PENDING 1
#define F_CONN_PARAMS_PENDING 2
#define F_VERSION_SENT 4

static const struct net_layer_handler_s master_handler;
static KROUTINE_EXEC(master_rq_done);
static void master_packet_push(struct ble_master_s *master,
                               struct buffer_s *buffer,
                               bool_t reliable);

#define dprintk(...) do{}while(0)
//#define dprintk printk

static uint32_t cu_tk(struct ble_master_s *master, uint16_t units)
{
#if defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
// Do a round calculation
# define TK_PER_CU ((CONFIG_BLE_SLEEP_CLOCK_HZ + 400) / 800)
  return units * TK_PER_CU;
#else
  dev_timer_delay_t d;
  dev_timer_init_sec(&master->timer, &d, NULL, units, 800);
  return d;
#endif
}

static void master_enqueue_tx(struct ble_master_s *master)
{
  size_t txq_size = buffer_queue_count(&master->request.data.tx_queue);
  size_t txq_max = master->window * 4;

  while (txq_size < txq_max) {
    struct buffer_s *tx = buffer_queue_pop(&master->reliable_queue);
    if (!tx)
      break;
    buffer_queue_pushback(&master->request.data.tx_queue, tx);
    buffer_refdec(tx);
    txq_size++;
  }

  while (txq_size < txq_max) {
    struct buffer_s *tx = buffer_queue_pop(&master->bulk_queue);
    if (!tx)
      break;
    buffer_queue_pushback(&master->request.data.tx_queue, tx);
    buffer_refdec(tx);
    txq_size++;
  }
}

static uint8_t mod37(uint32_t x)
{
  uint32_t d = 37 << 10;

  while (d >= 37) {
    if (x >= d)
      x -= d;

    d >>= 1;
  }

  return x;
}

static void request_schedule(struct ble_master_s *master)
{
  uint16_t event_advance = 0;
  int16_t after_instant;
  error_t err;
  uint32_t to_next_event_unit;
  uint32_t ws_tk;

 again:
  event_advance++;

  master->scheduled_event_counter = master->last_event_counter + event_advance;

  after_instant = master->scheduled_event_counter
    - master->conn_params_update_instant;

  // Connection parameters related
  if ((master->flags & F_CONN_PARAMS_PENDING) && after_instant >= 0) {
    to_next_event_unit = (event_advance - after_instant) * master->interval;
    to_next_event_unit += master->pending_win_offset;
    to_next_event_unit += after_instant * master->pending_interval;
    ws_tk = cu_tk(master, master->pending_window);
  } else {
    to_next_event_unit = event_advance * master->interval;
    ws_tk = cu_tk(master, master->window);
  }

  // Channel map related
  master->scheduled_unmapped_channel = mod37(master->last_unmapped_channel + event_advance * master->hop);
  if (master->flags & F_CHANNEL_MAP_PENDING
      && (int16_t)(master->scheduled_event_counter - master->channel_map_update_instant) >= 0) {
    master->request.data.channel = master->pending_mapped_channel[master->scheduled_unmapped_channel];
  } else {
    master->request.data.channel = master->mapped_channel[master->scheduled_unmapped_channel];
  }

  master->request.not_before = master->last_anchor + cu_tk(master, to_next_event_unit);
  master->request.not_after = master->request.not_before + ws_tk;
  master->request.max_duration = 0;
  master->request.data.latency_permitted = 0;

  if (master->request.not_before > master->drops_at)
    master->reason = BLE_CONNECTION_TIMEOUT;

  if (master->reason) {
    dprintk("Master disconnect reason: %d\n", master->reason);
    net_layer_refdec(&master->layer);
    return;
  }

  kroutine_init(&master->request.base.kr, master_rq_done, KROUTINE_INTERRUPTIBLE);

  master_enqueue_tx(master);

  err = DEVICE_OP(&master->radio, request, &master->request);
  if (err) {
    dprintk("Sched error: %d\n", err);
    event_advance++;
    goto again;
  }
}

static void timing_params_apply(
  struct ble_master_s *master,
  const struct ble_conn_timing_param_s *timing)
{
  master->interval = timing->interval;
  master->window = timing->win_size;

  dev_timer_init_sec_round(&master->timer, &master->timeout_tk,
                           NULL, timing->timeout, 100);
}

static void event_reference_setup(
  struct ble_master_s *master,
  dev_timer_value_t reference,
  uint16_t win_offset)
{
  int32_t connect_to_first_ref, connect_to_first_ref_tk;

  /**
     Because of reference packet length, we do this calculation with a
     1us precision, this is the only one.
  */
  connect_to_first_ref = BLE_PACKET_TIME(34)
    + BLE_T_CONN_UNIT * (win_offset + 1 - master->interval);

  dprintk("packet time: %d, wo: %d, int: %d\n", BLE_PACKET_TIME(34),
         win_offset, master->interval);

  dprintk("connect_to_first_ref: %ld\n", connect_to_first_ref);

  if (connect_to_first_ref >= 0) {
    dev_timer_delay_t d;
    dev_timer_init_sec(&master->timer, &d,
                       NULL, connect_to_first_ref, 1000000);
    connect_to_first_ref_tk = d;
  } else {
    dev_timer_delay_t d;
    dev_timer_init_sec_ceil(&master->timer, &d,
                            NULL, -connect_to_first_ref, 1000000);
    connect_to_first_ref_tk = -d;
  }

  master->last_anchor = reference + connect_to_first_ref_tk;
  master->last_event_counter = -1;
  master->last_unmapped_channel = 0;
  master->drops_at = master->last_anchor + cu_tk(master, master->interval * 6);;

  dprintk("Connection packet at %llu, base ref %ld ticks after\n",
         reference, connect_to_first_ref_tk);
}

error_t ble_master_init(
  struct ble_master_s *master,
  const struct ble_master_handler_s *handler,
  struct net_scheduler_s *scheduler,
  const char *dev,
  const struct ble_connection_parameters_s *conn_params)
{
  error_t err;
  struct ble_radio_info_s info;

  err = device_get_accessor_by_path(&master->radio, NULL, dev, DRIVER_CLASS_BLE_RADIO);
  if (err)
    goto err_out;

  err = device_get_accessor_by_path(&master->timer, NULL, dev, DRIVER_CLASS_TIMER);
  if (err)
    goto err_put_radio;

  err = net_layer_init(&master->layer, &master_handler, scheduler,
                       BLE_LAYER_TYPE_MASTER, 2);
  if (err)
    goto err_put_devices;

  DEVICE_OP(&master->radio, get_info, &info);

  master->layer.payload_offset = info.data_prepend_size + 2;

  master->flags = 0;

  master->handler = handler;

  device_start(&master->radio);

  buffer_queue_init(&master->reliable_queue);
  buffer_queue_init(&master->bulk_queue);
  buffer_queue_init(&master->request.data.tx_queue);
  buffer_queue_init(&master->request.data.rx_queue);
  master->request.type = DEVICE_BLE_RADIO_DATA_MASTER;
  master->request.packet_pool = scheduler->packet_pool;
  master->request.data.access = conn_params->conn_req.access_address;
  master->request.data.crc_init = conn_params->conn_req.crc_init;

  ble_conn_channel_mapping_expand(master->mapped_channel, conn_params->conn_req.channel_map);
  master->hop = conn_params->conn_req.hop;
  timing_params_apply(master, &conn_params->conn_req.timing);
  event_reference_setup(master, conn_params->connect_packet_timestamp,
                        conn_params->conn_req.win_offset);

  /** Our own reference as long as driver schedules the requests */
  net_layer_refinc(&master->layer);

  request_schedule(master);

  dprintk("master init done\n");

  return 0;

 err_put_devices:
  device_put_accessor(&master->timer);

 err_put_radio:
  device_put_accessor(&master->radio);

 err_out:
  return err;
}

static KROUTINE_EXEC(master_rq_done)
{
  struct ble_master_s *master = KROUTINE_CONTAINER(kr, *master, request.base.kr);

  if (master->reason) {
    dprintk("Master disconnect reason: %d\n", master->reason);
    net_layer_refdec(&master->layer);
    return;
  }

  dprintk("Event %d done: %d, %d pkts, anchor %lld\n  (%d tx, %d acked, %d empty, %d rxok, %d invalid, %d repeated, %d empty)\n",
         master->scheduled_event_counter,
         master->request.status, master->request.data.packet_count,
         master->request.data.anchor,
         master->request.data.tx_count,
         master->request.data.tx_acked_count,
         master->request.data.tx_empty_count,
         master->request.data.rx_ok_count,
         master->request.data.rx_invalid_count,
         master->request.data.rx_repeated_count,
         master->request.data.rx_empty_count);

  master->last_event_counter = master->scheduled_event_counter;
  master->last_unmapped_channel = master->scheduled_unmapped_channel;

  if (master->flags & F_CONN_PARAMS_PENDING
      && (int16_t)(master->last_event_counter - master->conn_params_update_instant) >= 0) {
    master->flags &= ~F_CONN_PARAMS_PENDING;
    master->timeout_tk = master->pending_timeout_tk;
    master->interval = master->pending_interval;
    master->window = master->pending_window;
    printk("Conn params update: int %d (%llx)\n",
           master->pending_interval,
           master->last_anchor);
  }

  if (master->flags & F_CHANNEL_MAP_PENDING
      && (int16_t)(master->last_event_counter - master->channel_map_update_instant) >= 0) {
    master->flags &= ~F_CHANNEL_MAP_PENDING;
    memcpy(master->mapped_channel, master->pending_mapped_channel, 37);
    printk("Channel map update\n");
  }

  master->last_anchor = __MAX(master->scheduled_anchor, master->request.data.anchor);

  if (master->request.data.packet_count > 1)
    master->drops_at = master->request.data.anchor + master->timeout_tk;

  /**
     TODO: Enqueue packets and do crypto on them
   */

  request_schedule(master);

  struct buffer_s *p;
  while ((p = buffer_queue_pop(&master->request.data.rx_queue))) {
    struct net_task_s *task = net_scheduler_task_alloc(master->layer.scheduler);
    struct net_addr_s dst;

    dst.llid = p->data[p->begin] & 0x3;
    p->begin += 2;

    dprintk("Packet from master: %p ll %d [%P]\n", p, dst.llid, p->data + p->begin, p->end - p->begin);

    net_task_inbound_push(task, &master->layer, &master->layer,
                          master->request.data.anchor, NULL, &dst, p);
    buffer_refdec(p);
  }
}

static
void llcp_handle(struct ble_master_s *master, struct buffer_s *p)
{
  const uint8_t *args = &p->data[p->begin + 1];

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ:
  case BLE_LL_CHANNEL_MAP_REQ:
  case BLE_LL_ENC_REQ:
    // Invalid
    break;

  case BLE_LL_TERMINATE_IND:
    master->reason = *args;

  case BLE_LL_PING_REQ: {
    struct buffer_s *rsp = net_layer_packet_alloc(&master->layer, master->layer.context.prefix_size, 3);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_PING_RSP;

    master_packet_push(master, rsp, 1);

    buffer_refdec(rsp);
    return;
  }

  case BLE_LL_VERSION_IND: {
    if (master->flags & F_VERSION_SENT)
      return;

    master->flags |= F_VERSION_SENT;

    struct buffer_s *rsp = net_layer_packet_alloc(&master->layer, master->layer.context.prefix_size, 8);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 6;
    rsp->data[rsp->begin + 2] = BLE_LL_VERSION_IND;
    rsp->data[rsp->begin + 3] = BLE_LL_VERSION_4_0;
    endian_le16_na_store(&rsp->data[rsp->begin + 4], 0xffff);
    endian_le16_na_store(&rsp->data[rsp->begin + 6], 0);

    master_packet_push(master, rsp, 1);

    buffer_refdec(rsp);
    return;
  }

  case BLE_LL_ENC_RSP:
  case BLE_LL_START_ENC_REQ:
  case BLE_LL_START_ENC_RSP:
  case BLE_LL_PAUSE_ENC_REQ:
  case BLE_LL_PAUSE_ENC_RSP:
    // TODO
    break;

  case BLE_LL_UNKNOWN_RSP:
  case BLE_LL_PING_RSP:
    // OSEF
    break;

  case BLE_LL_FEATURE_REQ:
  case BLE_LL_FEATURE_RSP:
  case BLE_LL_SLAVE_FEATURE_REQ:
    // TODO
    break;

  case BLE_LL_CONNECTION_PARAM_REQ:
  case BLE_LL_CONNECTION_PARAM_RSP:
    // Who cares ?
    break;

  case BLE_LL_REJECT_IND:
  case BLE_LL_REJECT_IND_EXT:
    // WTF
    break;

  case BLE_LL_LENGTH_REQ:
  case BLE_LL_LENGTH_RSP:
    // Maybe
    break;
  }
}

static
void ble_master_destroyed(struct net_layer_s *layer)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);

  device_stop(&master->radio);
  device_put_accessor(&master->timer);
  device_put_accessor(&master->radio);

  master->handler->destroyed(master);
}

static
void ble_master_task_handle(struct net_layer_s *layer,
                            struct net_task_header_s *header)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  switch (task->header.type) {
  case NET_TASK_INBOUND:
    if (task->header.source == &master->layer) {
      dprintk("Master packet from ll: %d\n", task->inbound.dst_addr.llid);

      switch (task->inbound.dst_addr.llid) {
      case BLE_LL_RESERVED:
        master->reason = BLE_COMMAND_DISALLOWED;
        break;

      case BLE_LL_DATA_CONT:
      case BLE_LL_DATA_START:
        if (master->l2cap) {
          net_task_inbound_forward(task, master->l2cap);
          return;
        }

      case BLE_LL_CONTROL:
        llcp_handle(master, task->inbound.buffer);
        break;
      }
    } else if (task->header.source == master->l2cap) {
      uint8_t header[] = {task->inbound.dst_addr.llid,
                          task->inbound.buffer->end - task->inbound.buffer->begin};
      buffer_prepend(task->inbound.buffer, header, 2);

      master_packet_push(master, task->inbound.buffer, task->inbound.dst_addr.reliable);
    }
    break;

  default:
    break;
  }

  net_task_cleanup(task);
}

static
error_t ble_master_bound(struct net_layer_s *layer,
                         void *addr,
                         struct net_layer_s *child)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);

  // Child list takes the reference. Dont double it.
  switch (layer->type) {
  case BLE_LAYER_TYPE_L2CAP:
    master->l2cap = child;
    return 0;

//  case BLE_LAYER_TYPE_SECURITY:
//    master->security = child;
//    return 0;

  case BLE_LAYER_TYPE_GAP:
    master->gap = child;
    return 0;

  default:
    return -ENOTSUP;
  }
}

static
void ble_master_unbound(struct net_layer_s *layer,
                              struct net_layer_s *child)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);

  if (child == master->security)
    master->security = NULL;

  if (child == master->gap)
    master->gap = NULL;

  if (child == master->l2cap) {
    master->l2cap = NULL;

    if (!master->reason) {
      dprintk("Master l2cap unbound without connection closed\n");
      buffer_queue_clear(&master->reliable_queue);
      buffer_queue_clear(&master->bulk_queue);

      struct buffer_s *term = net_layer_packet_alloc(&master->layer, master->layer.context.prefix_size, 4);
      term->data[term->begin + 0] = BLE_LL_CONTROL;
      term->data[term->begin + 1] = 1;
      term->data[term->begin + 2] = BLE_LL_TERMINATE_IND;
      term->data[term->begin + 3] = BLE_REMOTE_USER_TERMINATED_CONNECTION;
      master_packet_push(master, term, 1);
      buffer_refdec(term);

      master->reason = BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST;
    }
  }
}

static const struct net_layer_handler_s master_handler = {
  .destroyed = ble_master_destroyed,
  .task_handle = ble_master_task_handle,
  .bound = ble_master_bound,
  .unbound = ble_master_unbound,
};

static void master_packet_push(struct ble_master_s *master, struct buffer_s *buffer, bool_t reliable)
{
  if (master->reason)
    return;

  buffer_queue_pushback(reliable ? &master->reliable_queue : &master->bulk_queue,
                        buffer);

  master_enqueue_tx(master);
}
