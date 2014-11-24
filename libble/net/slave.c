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

#include <ble/net/slave.h>
#include <ble/net/l2cap.h>
#include <ble/net/gap.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/data.h>
#include <ble/security_db.h>

#define SLAVE_CONTEXT_UPDATED 1

static const struct net_layer_handler_s slave_handler;
static KROUTINE_EXEC(slave_rq_done);

#define dprintk(...) do{}while(0)
//#define dprintk printk

static uint8_t mod37(uint32_t x);
static uint32_t cu_tk(struct ble_slave_s *slave, uint32_t units);
static uint32_t cu_ww_tk(struct ble_slave_s *slave, uint32_t units);

static void slave_request_schedule(struct ble_slave_s *slave);
static void slave_crypto_next(struct ble_slave_s *slave);
static void slave_llcp_push(struct ble_slave_s *slave, struct buffer_s *buffer);

/*
  Packet handling in slave:

                                From other layer
                                       | (task)
                                       v                (task)
                   +------------------>+<------------------+
               [7] ^ (task)        [5] | (task)            ^
                   |                   |                   |
  Internal /       |               [4] v (crypto_queue)    | (kr) [1]
 Notification <-> LLCP                CCM                Radio <-> antenna
  / Commands       ^               [3] | (kr->task)        ^
                   |                   |                   |
               [6] |                   v                   |
                   +<-------------[Dispatch]-------------->+ [2]
                                       |                (task)
                                       v
                                 To other layer

   1: slave_rq_done
   2: slave_tx_enqueue
   3: slave_crypto_done
   4: slave_crypto_next
   5: slave_ccm_enqueue
   6: slave_llcp_task_handle
   7: slave_llcp_push
 */

static void slave_tx_enqueue(struct ble_slave_s *slave,
                             struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;

  buffer_queue_pushback(&slave->ble_rq.data.tx_queue,
                        task->inbound.buffer);

  if ((p->data[p->begin] & 0x3) == BLE_LL_CONTROL) {
    switch (p->data[p->begin + 2]) {
    case BLE_LL_START_ENC_REQ:
      slave->tx_encryption = 1;
      slave->rx_encryption = 1;
      break;
    case BLE_LL_PAUSE_ENC_RSP:
      slave->tx_encryption = 0;
      break;
    }
  }

  dprintk("%s: @%lld %p ll %d [%P]\n", __FUNCTION__, task->inbound.timestamp,
         p, p->data[p->begin] & 0x3,
         p->data + p->begin,
         p->end - p->begin);

  slave_request_schedule(slave);

  net_task_cleanup(task);
}

static void slave_rx_enqueue(struct ble_slave_s *slave,
                             struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;

  if ((p->data[p->begin] & 0x3) == BLE_LL_CONTROL) {
    switch (p->data[p->begin + 2]) {
    case BLE_LL_PAUSE_ENC_REQ:
      slave->rx_encryption = 0;
      break;
    }
  }

  task->inbound.dst_addr.llid = p->data[p->begin] & 0x3;
  p->begin += 2;

  dprintk("%s %lld: %p ll %d [%P]\n", __FUNCTION__, task->inbound.timestamp,
          p, task->inbound.dst_addr.llid,
          p->data + p->begin,
          p->end - p->begin);

  net_task_inbound_forward(task, &slave->layer);
}

static KROUTINE_EXEC(slave_crypto_done)
{
  struct ble_slave_s *slave = KROUTINE_CONTAINER(kr, *slave, crypto_rq.rq.kr);
  struct net_task_s *task = slave->ccm_task;

  dprintk("%s %p %p\n", __FUNCTION__, slave, task);

  if (!task)
    return;

  // Replace packet in task
  
  buffer_refdec(task->inbound.buffer);
  task->inbound.buffer = slave->ccm_tmp_packet;
  slave->ccm_tmp_packet = NULL;
  slave->ccm_task = NULL;

  if (slave->crypto_rq.err) {
    slave->reason = BLE_AUTHENTICATION_FAILURE;
    net_task_cleanup(task);
    return;
  }

  if (!task->inbound.timestamp) {
    // Outbound
    dprintk("CRY < %P\n",
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    slave_tx_enqueue(slave, task);
  } else {
    // Inbound
    dprintk("CLR > %P\n",
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    task->inbound.src_addr.secure = 1;
    slave_rx_enqueue(slave, task);
  }

  slave_crypto_next(slave);
}

static void slave_crypto_next(struct ble_slave_s *slave)
{
  struct buffer_s *in, *out;
  struct net_task_s *task;
  uint16_t out_size = 0;

  if (slave->ccm_task)
    return;

  dprintk("%s\n", __FUNCTION__);

 again:
  task = net_task_s_from_header(net_task_queue_pop(&slave->ccm_queue));

  if (!task)
    return;

  in = task->inbound.buffer;

  if (!task->inbound.timestamp) {
    // Outbound
    dprintk("CLR < %P\n",
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    if (!slave->tx_encryption) {
      slave_tx_enqueue(slave, task);
      goto again;
    }

    slave->crypto_rq.op = DEV_CRYPTO_FINALIZE;
    slave->crypto_rq.ctx = &slave->ccm_tx_ctx;
    out_size = task->inbound.buffer->end - task->inbound.buffer->begin + 4;
  } else {
    // Inbound

    if (!slave->rx_encryption) {
      dprintk("CLR > %P\n",
             task->inbound.buffer->data + task->inbound.buffer->begin,
             task->inbound.buffer->end - task->inbound.buffer->begin);

      slave_rx_enqueue(slave, task);
      goto again;
    }

    if (task->inbound.buffer->end - task->inbound.buffer->begin < 7) {
      slave->reason = BLE_AUTHENTICATION_FAILURE;
      net_task_cleanup(task);
      goto again;
    }

    dprintk("CRY > %P\n",
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    slave->crypto_rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
    slave->crypto_rq.ctx = &slave->ccm_rx_ctx;
    out_size = task->inbound.buffer->end - task->inbound.buffer->begin - 4;
  }

  slave->ccm_task = task;

  out = net_layer_packet_alloc(
    &slave->layer, slave->layer.context.prefix_size,
    out_size);

  slave->ccm_tmp_packet = out;
  slave->crypto_rq.ad_len = 0;
  slave->crypto_rq.ad = 0;
  slave->crypto_rq.out = out->data + out->begin;
  slave->crypto_rq.in = in->data + in->begin;
  slave->crypto_rq.len = in->end - in->begin;

  kroutine_init(&slave->crypto_rq.rq.kr, slave_crypto_done, KROUTINE_INTERRUPTIBLE);
  DEVICE_OP(&slave->crypto, request, &slave->crypto_rq);
}

static void slave_ccm_enqueue(struct ble_slave_s *slave,
                              struct net_task_s *task)
{
  net_task_queue_pushback(&slave->ccm_queue, &task->header);
  slave_crypto_next(slave);
}

static void slave_llcp_push(struct ble_slave_s *slave,
                            struct buffer_s *buffer)
{
  struct net_task_s *task = net_scheduler_task_alloc(slave->layer.scheduler);
  net_task_inbound_push(task, &slave->layer, &slave->layer, 0, NULL, NULL, buffer);
}

static void slave_request_schedule(struct ble_slave_s *slave)
{
  dev_timer_delay_t to_event_tk, ws_tk, ww_tk;
  uint32_t event_advance;
  dev_timer_value_t now;
  int16_t after_instant;
  error_t err;

  event_advance = __MIN(slave->slave_latency + 1, slave->latency_backoff);

  if (!buffer_queue_isempty(&slave->ble_rq.data.tx_queue)
      || slave->ble_rq.data.md)
    event_advance = 1;

 again2:
  DEVICE_OP(&slave->timer, get_value, &now, 0);

  dprintk("Slave schedule %d, %d/%d/%d %s\n", slave->last_event_counter,
         event_advance, slave->latency_backoff, slave->slave_latency,
         slave->req_scheduled ? "scheduled" : "idle");

  if (now > slave->drops_at)
    slave->reason = BLE_CONNECTION_TIMEOUT;

  if (slave->req_scheduled) {
    if (slave->scheduled_event_counter == slave->last_event_counter + event_advance) {
      dprintk(" is next\n");
      return;
    }

    if (slave->req_scheduled
        && DEVICE_OP(&slave->radio, cancel, &slave->ble_rq) != 0) {
      dprintk(" cancel failed\n");
      return;
    }

    slave->req_scheduled = 0;
  }

  if (slave->reason) {
    dprintk("Slave disconnect reason: %d\n", slave->reason);
    net_layer_refdec(&slave->layer);
    return;
  }

  /*
    For updates:

    last_event_counter     --v
    event_advance            |<---------------->|
    time             --------+----------+-------+-------->
    instant                          ---^       |
    scheduled_event_counter                  ---^

    We cannot update last_event_counter before successful event.
    We may cancel event and reschedule it asap.
  */

 again:
  slave->scheduled_event_counter = slave->last_event_counter + event_advance;
  after_instant = slave->scheduled_event_counter - slave->conn_params_update_instant;

  // Connection parameters related
  if ((slave->conn_params_pending) && after_instant >= 0) {
    slave->to_next_event_unit = (event_advance - after_instant) * slave->interval;
    slave->to_next_event_unit += slave->pending_win_offset;
    slave->to_next_event_unit += after_instant * slave->pending_interval;
    ws_tk = cu_tk(slave, slave->pending_window);
  } else {
    slave->to_next_event_unit = event_advance * slave->interval;
    ws_tk = cu_tk(slave, slave->window);
  }

  // Our stuff
  to_event_tk = cu_tk(slave, slave->to_next_event_unit);
  ww_tk = cu_ww_tk(slave, slave->to_next_event_unit + slave->since_last_event_unit);

  slave->scheduled_anchor = slave->last_anchor + to_event_tk;

  if (slave->scheduled_anchor < now + cu_tk(slave, 1)) {
    event_advance++;
    dprintk(" again\n");
    goto again;
  }

  dprintk(" event %d (%d + %d)\n", slave->scheduled_event_counter,
         slave->last_event_counter, event_advance);

  // Channel map related
  slave->scheduled_unmapped_channel = mod37(slave->last_unmapped_channel + event_advance * slave->hop);
  if (slave->channel_map_pending
      && (int16_t)(slave->last_event_counter + event_advance
                   - slave->channel_map_update_instant) >= 0) {
    slave->ble_rq.data.channel = slave->pending_mapped_channel[slave->scheduled_unmapped_channel];
    dprintk(" channel map: %P (pending)\n", slave->pending_mapped_channel, 37);
  } else {
    slave->ble_rq.data.channel = slave->mapped_channel[slave->scheduled_unmapped_channel];
    dprintk(" channel map: %P\n", slave->mapped_channel, 37);
  }

  dprintk(" channel %d [%d] (%d + %d * %d)\n",
          slave->scheduled_unmapped_channel, slave->ble_rq.data.channel,
          slave->last_unmapped_channel, event_advance,
          slave->hop);

  dprintk(" anchor %llu, ww %d, ws %d\n",
         slave->scheduled_anchor, ww_tk, ws_tk);

  slave->ble_rq.not_before = slave->scheduled_anchor - ww_tk - 1;
  slave->ble_rq.not_after = slave->scheduled_anchor + ws_tk + ww_tk * 2;
  slave->ble_rq.data.latency_permitted = event_advance < slave->latency_backoff;
  slave->ble_rq.max_duration = 0;

  dprintk(" scheduling at %llu-%llu on channel %d\n",
         slave->ble_rq.not_before, slave->ble_rq.not_after,
         slave->ble_rq.data.channel);

  kroutine_init(&slave->ble_rq.base.kr, slave_rq_done, KROUTINE_INTERRUPTIBLE);

  dprintk("slave %p, dev %p, req %p\n", slave, slave->radio.dev, slave->radio.api->f_request);

  assert(slave->radio.dev);

  err = DEVICE_OP(&slave->radio, request, &slave->ble_rq);
  if (err) {
    dprintk("Sched error: %d\n", err);
    event_advance++;
    goto again2;
  }

  slave->req_scheduled = 1;
}

static void timing_params_apply(
  struct ble_slave_s *slave,
  const struct ble_conn_timing_param_s *timing)
{
  slave->interval = timing->interval;
  slave->window = timing->win_size;

  dev_timer_init_sec_round(&slave->timer, &slave->timeout_tk,
                           NULL, timing->timeout, 100);

  slave->slave_latency = timing->latency;
  slave->latency_backoff = 1;
}

static void event_reference_setup(
  struct ble_slave_s *slave,
  dev_timer_value_t reference,
  uint16_t win_offset)
{
  int32_t connect_to_first_ref, connect_to_first_ref_tk;

  /**
     Because of reference packet length, we do this calculation with a
     1us precision, this is the only one.
  */
  connect_to_first_ref = BLE_PACKET_TIME(34)
    + BLE_T_CONN_UNIT * (win_offset + 1 - slave->interval);

  dprintk("packet time: %d, wo: %d, int: %d\n", BLE_PACKET_TIME(34),
         win_offset, slave->interval);

  dprintk("connect_to_first_ref: %ld\n", connect_to_first_ref);

  if (connect_to_first_ref >= 0) {
    dev_timer_delay_t d;
    dev_timer_init_sec(&slave->timer, &d,
                       NULL, connect_to_first_ref, 1000000);
    connect_to_first_ref_tk = d;
  } else {
    dev_timer_delay_t d;
    dev_timer_init_sec_ceil(&slave->timer, &d,
                            NULL, -connect_to_first_ref, 1000000);
    connect_to_first_ref_tk = -d;
  }

  slave->last_anchor = reference + connect_to_first_ref_tk;
  slave->last_event_counter = -1;
  slave->last_unmapped_channel = 0;
  slave->drops_at = slave->last_anchor + cu_tk(slave, slave->interval * 6);;

  dprintk("Connection packet at %llu, base ref %ld ticks after\n",
         reference, connect_to_first_ref_tk);
}

error_t ble_slave_init(
  struct ble_slave_s *slave,
  const struct ble_slave_handler_s *handler,
  struct net_scheduler_s *scheduler,
  const char *ble,
  const char *crypto,
  struct dev_rng_s *rng,
  const struct ble_connection_parameters_s *conn_params,
  struct ble_peer_s *peer)
{
  error_t err;
  struct ble_radio_info_s ble_info;
  struct dev_crypto_info_s crypto_info;

  memset(slave, 0, sizeof(*slave));

  err = device_get_accessor_by_path(&slave->radio, NULL, ble, DRIVER_CLASS_BLE_RADIO);
  if (err)
    goto err_out;

  err = device_get_accessor_by_path(&slave->timer, NULL, ble, DRIVER_CLASS_TIMER);
  if (err)
    goto err_put_radio;

  err = device_get_accessor_by_path(&slave->crypto, NULL, crypto, DRIVER_CLASS_CRYPTO);
  if (err)
    goto err_put_timer;

  err = net_layer_init(&slave->layer, &slave_handler, scheduler);
  if (err)
    goto err_put_devices;

  DEVICE_OP(&slave->radio, get_info, &ble_info);
  DEVICE_OP(&slave->crypto, info, &crypto_info);

  slave->layer.context.prefix_size = ble_info.prefix_size + 2;
  slave->layer.context.mtu = ble_info.mtu - 6;

  slave->handler = handler;
  slave->peer = peer;
  slave->rng = rng;

  device_start(&slave->radio);

  net_task_queue_init(&slave->ccm_queue);

  slave->ccm_tx_ctx.mode = DEV_CRYPTO_MODE_BLE_CCM;
  slave->ccm_tx_ctx.state_data = mem_alloc(crypto_info.state_size, mem_scope_sys);
  slave->ccm_tx_ctx.key_data = slave->crypto_key;
  slave->ccm_tx_ctx.key_len = 16;
  slave->ccm_tx_ctx.iv_len = 8;
  slave->ccm_tx_ctx.auth_len = 2;
  slave->ccm_tx_ctx.encrypt_only = 1;

  slave->ccm_rx_ctx.mode = DEV_CRYPTO_MODE_BLE_CCM;
  slave->ccm_rx_ctx.state_data = mem_alloc(crypto_info.state_size, mem_scope_sys);
  slave->ccm_rx_ctx.key_data = slave->crypto_key;
  slave->ccm_rx_ctx.key_len = 16;
  slave->ccm_rx_ctx.iv_len = 8;
  slave->ccm_rx_ctx.auth_len = 2;
  slave->ccm_rx_ctx.decrypt_only = 1;

  buffer_queue_init(&slave->ble_rq.data.tx_queue);
  buffer_queue_init(&slave->ble_rq.data.rx_queue);
  slave->ble_rq.type = DEVICE_BLE_RADIO_DATA_SLAVE;
  slave->ble_rq.packet_pool = scheduler->packet_pool;
  slave->ble_rq.data.access = conn_params->conn_req.access_address;
  slave->ble_rq.data.crc_init = conn_params->conn_req.crc_init;

  ble_conn_channel_mapping_expand(slave->mapped_channel, conn_params->conn_req.channel_map);
  slave->hop = conn_params->conn_req.hop;
  slave->master_sca = conn_params->conn_req.sca;
  timing_params_apply(slave, &conn_params->conn_req.timing);
  event_reference_setup(slave, conn_params->connect_packet_timestamp,
                        conn_params->conn_req.win_offset);

  slave->latency_backoff = 1;

  /** Our own reference as long as driver schedules the requests */
  net_layer_refinc(&slave->layer);

  slave_request_schedule(slave);

  dprintk("slave init done\n");

  return 0;

 err_put_devices:
  device_put_accessor(&slave->crypto);

 err_put_timer:
  device_put_accessor(&slave->timer);

 err_put_radio:
  device_put_accessor(&slave->radio);

 err_out:
  return err;
}

static KROUTINE_EXEC(slave_rq_done)
{
  struct ble_slave_s *slave = KROUTINE_CONTAINER(kr, *slave, ble_rq.base.kr);

  slave->req_scheduled = 0;

  if (slave->reason) {
    dprintk("Slave disconnect reason: %d\n", slave->reason);
    net_layer_refdec(&slave->layer);
    return;
  }

  dprintk("Event %d done: %d, %d pkts, anchor %lld, lat perm %d\n  (%d tx, %d acked, %d empty, %d rxok, %d invalid, %d repeated, %d empty)\n",
         slave->scheduled_event_counter,
         slave->ble_rq.status, slave->ble_rq.data.packet_count,
         slave->ble_rq.data.anchor,
         slave->ble_rq.data.latency_permitted,
         slave->ble_rq.data.tx_count,
         slave->ble_rq.data.tx_acked_count,
         slave->ble_rq.data.tx_empty_count,
         slave->ble_rq.data.rx_ok_count,
         slave->ble_rq.data.rx_invalid_count,
         slave->ble_rq.data.rx_repeated_count,
         slave->ble_rq.data.rx_empty_count);

  slave->last_event_counter = slave->scheduled_event_counter;
  slave->last_unmapped_channel = slave->scheduled_unmapped_channel;
  slave->since_last_event_unit += slave->to_next_event_unit;

  if (slave->conn_params_pending
      && (int16_t)(slave->last_event_counter - slave->conn_params_update_instant) >= 0) {
    slave->conn_params_pending = 0;
    slave->timeout_tk = slave->pending_timeout_tk;
    slave->interval = slave->pending_interval;
    slave->slave_latency = slave->pending_slave_latency;
    slave->window = slave->pending_window;
    dprintk("Conn params update: int %d, lat %d (%llx)\n",
           slave->pending_interval,
           slave->pending_slave_latency,
           slave->last_anchor);
  }

  if (slave->channel_map_pending
      && (int16_t)(slave->last_event_counter - slave->channel_map_update_instant) >= 0) {
    slave->channel_map_pending = 0;
    memcpy(slave->mapped_channel, slave->pending_mapped_channel, 37);
    dprintk("Channel map update\n");
  }

  switch (slave->ble_rq.data.packet_count) {
  case 1:
    // We could still sync clocks
    slave->last_anchor = slave->ble_rq.data.anchor;
    slave->since_last_event_unit = 0;

    if (!slave->ble_rq.data.latency_permitted)
      slave->latency_backoff = 1;
    break;

  default:
    // Master will update its timer only if it sees a packet from us
    slave->drops_at = slave->ble_rq.data.anchor + slave->timeout_tk;

    // Sync clocks
    slave->last_anchor = slave->ble_rq.data.anchor;
    slave->since_last_event_unit = 0;

    slave->latency_backoff = 2 * slave->latency_backoff;
    break;

  case 0:
    slave->last_anchor = slave->scheduled_anchor;
    slave->latency_backoff = 1;
    break;
  }

  if (slave->latency_backoff > slave->slave_latency)
    slave->latency_backoff = slave->slave_latency + 1;

  dprintk(" latency %d, backoff %d\n",
         slave->slave_latency, slave->latency_backoff);

  /**
     TODO: Enqueue packets and do crypto on them
   */

  slave_request_schedule(slave);

  struct buffer_s *p;
  while ((p = buffer_queue_pop(&slave->ble_rq.data.rx_queue))) {
    struct net_task_s *task = net_scheduler_task_alloc(slave->layer.scheduler);
    dprintk("%s: [%P]\n", __FUNCTION__, p->data + p->begin, p->end - p->begin);
    net_task_inbound_push(task, &slave->layer, &slave->layer,
                          slave->ble_rq.data.anchor,
                          NULL, NULL, p);
    buffer_refdec(p);
  }
}

static
void slave_llcp_task_handle(struct ble_slave_s *slave, struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;
  const uint8_t *args = &p->data[p->begin + 1];
  uint8_t reason = 0;
  struct buffer_s *rsp;

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ:
    slave->latency_backoff = 1;
    slave->pending_window = args[0];
    slave->pending_win_offset = endian_le16_na_load(args + 1);
    slave->pending_interval = endian_le16_na_load(args + 3);
    slave->pending_slave_latency = endian_le16_na_load(args + 5);
    dev_timer_init_sec_round(&slave->timer, &slave->pending_timeout_tk,
                             NULL, endian_le16_na_load(args + 7), 100);
    slave->conn_params_update_instant = endian_le16_na_load(args + 9);
    slave->conn_params_pending = 1;

    if ((int16_t)(slave->conn_params_update_instant - slave->last_event_counter) < 0)
      slave->reason = BLE_INSTANT_PASSED;
    break;

  case BLE_LL_CHANNEL_MAP_REQ:
    slave->latency_backoff = 1;
    ble_conn_channel_mapping_expand(slave->pending_mapped_channel,
                                    endian_le32_na_load(args)
                                    | ((uint64_t)args[4] << 32));
    slave->channel_map_update_instant = endian_le16_na_load(args + 5);
    slave->channel_map_pending = 1;

    dprintk("Channel map update pending at %d [%P]\n", slave->channel_map_update_instant,
           slave->pending_mapped_channel, 37);

    if ((int16_t)(slave->channel_map_update_instant - slave->last_event_counter) < 0)
      slave->reason = BLE_INSTANT_PASSED;
    break;

  case BLE_LL_TERMINATE_IND:
    slave->reason = *args;
    break;

  case BLE_LL_PING_REQ: {
    rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 3);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_PING_RSP;

    goto reply;
  }

  case BLE_LL_VERSION_IND: {
    if (slave->version_sent)
      break;

    slave->version_sent = 1;

    rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 8);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 6;
    rsp->data[rsp->begin + 2] = BLE_LL_VERSION_IND;
    rsp->data[rsp->begin + 3] = BLE_LL_VERSION_4_0;
    endian_le16_na_store(&rsp->data[rsp->begin + 4], 0xffff);
    endian_le16_na_store(&rsp->data[rsp->begin + 6], 0);

    goto reply;
  }

  case BLE_LL_ENC_REQ: {
    uint8_t skd[16];
    uint8_t iv[8];
    error_t err;

    err = dev_rng_wait_read(slave->rng, skd + 8, 8);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto reject;
    }

    err = dev_rng_wait_read(slave->rng, iv + 4, 4);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto reject;
    }

    dprintk("skdm:      %P\n", args + 10, 8);
    dprintk("ivm:       %P\n", args + 18, 4);

    dprintk("rand:      %P\n", args, 8);
    dprintk("ediv:      %d\n", endian_le16_na_load(args + 8));

    dprintk("skds:      %P\n", skd + 8, 8);
    dprintk("ivs:       %P\n", iv + 4, 4);

    memcpy(skd, args + 10, 8);
    memcpy(iv, args + 18, 4);

    err = ble_peer_sk_get(slave->peer, skd,
                          args, endian_le16_na_load(args + 8),
                          slave->crypto_key);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      goto reject;
    }

    err = ble_peer_sk_get(slave->peer, skd,
                          args, endian_le16_na_load(args + 8),
                          slave->crypto_key);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      goto reject;
    }

    dprintk("SK:        %P\n", slave->crypto_key, 16);
    dprintk("IV:        %P\n", iv, 8);

    slave->crypto_rq.op = DEV_CRYPTO_INIT;
    slave->crypto_rq.ctx = &slave->ccm_tx_ctx;
    slave->crypto_rq.iv_ctr = iv;

    err = dev_crypto_wait_op(&slave->crypto, &slave->crypto_rq);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto reject;
    }

    slave->crypto_rq.op = DEV_CRYPTO_INIT | DEV_CRYPTO_INVERSE;
    slave->crypto_rq.ctx = &slave->ccm_rx_ctx;
    slave->crypto_rq.iv_ctr = iv;

    err = dev_crypto_wait_op(&slave->crypto, &slave->crypto_rq);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto reject;
    }

    rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 15);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 13;
    rsp->data[rsp->begin + 2] = BLE_LL_ENC_RSP;
    memcpy(rsp->data + rsp->begin + 3, skd + 8, 8);
    memcpy(rsp->data + rsp->begin + 11, iv + 4, 4);

    slave_llcp_push(slave, rsp);
    buffer_refdec(rsp);

    rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 3);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_START_ENC_REQ;

    goto reply;
  }

  case BLE_LL_START_ENC_RSP: {
    rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 3);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_START_ENC_RSP;

    net_task_notification_push(net_scheduler_task_alloc(slave->layer.scheduler),
                               &slave->layer, &slave->layer,
                               SLAVE_CONTEXT_UPDATED);

    goto reply;
  }

  case BLE_LL_PAUSE_ENC_REQ: {
    rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 3);
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_PAUSE_ENC_RSP;

    net_task_notification_push(net_scheduler_task_alloc(slave->layer.scheduler),
                               &slave->layer, &slave->layer,
                               SLAVE_CONTEXT_UPDATED);

    goto reply;
  }

  case BLE_LL_PAUSE_ENC_RSP:
    // Nothing done here. See in CCM handling
    break;

  case BLE_LL_START_ENC_REQ:
  case BLE_LL_ENC_RSP:
  case BLE_LL_CONNECTION_PARAM_REQ:
    // Only initiated by slave, should not be received.
    slave->reason = BLE_COMMAND_DISALLOWED;
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

  goto cleanup;

 reject:
  rsp = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 4);
  rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
  rsp->data[rsp->begin + 1] = 2;
  rsp->data[rsp->begin + 2] = BLE_LL_REJECT_IND;
  rsp->data[rsp->begin + 3] = reason;

 reply:
  slave_llcp_push(slave, rsp);
  buffer_refdec(rsp);

 cleanup:
  net_task_cleanup(task);
}

static
void ble_slave_destroyed(struct net_layer_s *layer)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);

  dprintk("Slave %p destroyed\n", slave);

  mem_free(slave->ccm_tx_ctx.state_data);
  mem_free(slave->ccm_rx_ctx.state_data);

  GCT_FOREACH(net_task_queue, &slave->ccm_queue, header,
              net_task_cleanup(net_task_s_from_header(header));
              );

  net_task_queue_destroy(&slave->ccm_queue);

  device_stop(&slave->radio);
  device_put_accessor(&slave->timer);
  device_put_accessor(&slave->radio);

  assert(!slave->ccm_task);
  assert(!slave->ccm_tmp_packet);

  slave->handler->destroyed(slave);
}

static
void ble_slave_task_handle(struct net_layer_s *layer,
                           struct net_task_header_s *header)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  dprintk("%s in %p %p -> %p", __FUNCTION__, slave, task->header.source, task->header.target);

  switch (header->type) {
  case NET_TASK_INBOUND:
    dprintk(" inbound @%lld, ll %d [%P]",
           task->inbound.timestamp,
           task->inbound.dst_addr.llid,
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    if (task->header.source != &slave->layer) {
      // Packet from another layer

      uint8_t header[] = {
        task->inbound.dst_addr.llid,
        task->inbound.buffer->end - task->inbound.buffer->begin,
      };
      buffer_prepend(task->inbound.buffer, header, 2);
      // Delete timestamp, this marks packet as ours
      task->inbound.timestamp = 0;

      dprintk(" outgoing -> ccm\n");

      slave_ccm_enqueue(slave, task);
      return;
    } else if (task->inbound.timestamp) {
      // Packet originating from radio

      switch (task->inbound.dst_addr.llid) {
      case BLE_LL_RESERVED:
        dprintk(" radio packet -> ccm\n");
        // Packet type not decoded yet, we are before crypto
        slave_ccm_enqueue(slave, task);
        return;

      case BLE_LL_DATA_CONT:
      case BLE_LL_DATA_START:
        dprintk(" decrypted packet -> l2\n");
        if (!slave->l2cap)
          break;

        slave->latency_backoff = 1;
        net_task_inbound_forward(task, slave->l2cap);
        return;

      case BLE_LL_CONTROL:
        dprintk(" decrypted control\n");
        slave_llcp_task_handle(slave, task);
        return;
      }
    } else {
      // Outgoing packet from ourselves, not yet encrypted

      dprintk(" llcp from self -> ccm\n");

      slave_ccm_enqueue(slave, task);
      return;
    }
    break;

  case NET_TASK_NOTIFICATION:
    switch (task->notification.opcode) {
    case SLAVE_CONTEXT_UPDATED:
      slave->layer.context.addr.secure = slave->tx_encryption && slave->rx_encryption;
      net_layer_context_changed(&slave->layer);
      break;
    }
    break;

  default:
    dprintk(" other\n");
    break;
  }

  net_task_cleanup(task);
}

static
error_t ble_slave_bound(struct net_layer_s *layer,
                        void *addr,
                        struct net_layer_s *child)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);

  // Child list takes the reference. Dont double it.
  switch (child->handler->type) {
  case BLE_LAYER_TYPE_L2CAP:
    slave->l2cap = child;
    return 0;

//  case BLE_LAYER_TYPE_SECURITY:
//    slave->security = child;
//    return 0;

  case BLE_LAYER_TYPE_GAP:
    slave->gap = child;
    return 0;

  default:
    return -ENOTSUP;
  }
}

static
void ble_slave_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);

  if (child == slave->security)
    slave->security = NULL;

  if (child == slave->gap)
    slave->gap = NULL;

  if (child == slave->l2cap) {
    slave->l2cap = NULL;

    if (!slave->reason) {
      dprintk("Slave l2cap unbound without connection closed\n");

      struct buffer_s *term = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 4);
      term->data[term->begin + 0] = BLE_LL_CONTROL;
      term->data[term->begin + 1] = 1;
      term->data[term->begin + 2] = BLE_LL_TERMINATE_IND;
      term->data[term->begin + 3] = BLE_REMOTE_USER_TERMINATED_CONNECTION;
      slave_llcp_push(slave, term);
      buffer_refdec(term);

      slave->reason = BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST;
    }
  }
}

static const struct net_layer_handler_s slave_handler = {
  .destroyed = ble_slave_destroyed,
  .task_handle = ble_slave_task_handle,
  .bound = ble_slave_bound,
  .unbound = ble_slave_unbound,
  .type = BLE_LAYER_TYPE_SLAVE,
};




/* Timing/data helpers */

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

#if defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
// Do a round calculation
# define TK_PER_CU ((CONFIG_BLE_SLEEP_CLOCK_HZ + 400) / 800)
#endif

static uint32_t cu_tk(struct ble_slave_s *slave, uint32_t units)
{
#if !defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
  dev_timer_delay_t tk;

  dev_timer_init_sec(&slave->timer, &tk, NULL, units, 1000000 / BLE_T_CONN_UNIT);

  return tk;
#elif CONFIG_BLE_SLEEP_CLOCK_HZ == 32768
  return units * 41 - units / 32 - units / 128;
#else
  return units * TK_PER_CU;
#endif
}

/**
   Compute number of sleep clock ticks to widen the window with
   depending on number of conn units (1250 us) that passed.

   ww_tk = ww_us / timer_us_per_tick
   ww_us = passed_us * (master_sca_ppm + slave_sca_ppm) / 1000000
   passed_us = passed_units * BLE_T_CONN_UNIT

   ww_tk = passed_units * (master_sca_ppm + slave_sca_ppm) / 1000000
 */
static uint32_t cu_ww_tk(struct ble_slave_s *slave, uint32_t units)
{
#if !defined(CONFIG_BLE_SLEEP_CLOCK_PPM)
  /**
     Here, nothing is known constantly, so compute everything the slow
     way.  Get local clock accuracy, assume worst case, get PPMs for
     everyone, and widen accordingly.
   */
  dev_timer_delay_t tk;
  struct dev_timer_config_s config;

  DEVICE_OP(&slave->timer, config, &config, 0);

  dev_timer_init_sec(&slave->timer, &tk, NULL,
    units * (ble_sca_wc[slave->master_sca] + dev_freq_acc_ppb(&config.acc) / 1000) / 1000000,
    1000000 / BLE_T_CONN_UNIT);

  return tk;
#elif !defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
  dev_timer_delay_t tk;

  /**
     Here, at least local clock accuracy is known, but not its speed
     (assume variable divisor/PLL somewhere).
   */

  dev_timer_init_sec(&slave->timer, &tk, NULL,
    units * (ble_sca_wc[slave->master_sca] + CONFIG_BLE_SLEEP_CLOCK_PPM) / 1000000,
    1000000 / BLE_T_CONN_UNIT);

  return tk;
#else
  /**
     Here, slave_sca_ppm is compile time constant, master_sca_ppm is
     one out of 8 possible values, so precompute a table.

     In the table, a number of conn units that may pass before we have
     to increment window widening of one timer tick.

     Final calculation is round(passed_cus / table[sca])
   */
# define UNITS_PER_WW_TICK(master_sca_ppm) \
    (1000000 / ((master_sca_ppm) + CONFIG_BLE_SLEEP_CLOCK_PPM) / TK_PER_CU)

  static const uint16_t sca_units_per_ww_tick[] = {
    UNITS_PER_WW_TICK(20), UNITS_PER_WW_TICK(30),
    UNITS_PER_WW_TICK(50), UNITS_PER_WW_TICK(75),
    UNITS_PER_WW_TICK(100), UNITS_PER_WW_TICK(150),
    UNITS_PER_WW_TICK(250), UNITS_PER_WW_TICK(500),
  };

# undef UNITS_PER_WW_TICK

  uint16_t units_per_ww_tick = sca_units_per_ww_tick[slave->master_sca];
  uint32_t tk = 0;

  /**
     Do poor man's rounded division.  Dont expect @tt tk to be
     anywhere above 2 or 3, so this wont actually spin much and is
     always cheaper than a division.
   */

  units += units_per_ww_tick / 2;

  while (units > units_per_ww_tick) {
    units -= units_per_ww_tick;
    tk++;
  }

  return tk + 1;
#endif
}
