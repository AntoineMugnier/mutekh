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
#include <ble/net/slave.h>
#include <ble/net/l2cap.h>
#include <ble/net/gap.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/data.h>

#include <ble/util/channel_mapper.h>
#include <ble/util/timing_mapper.h>

#include "ble_radio_private.h"

#define SLAVE_CONTEXT_UPDATED 1

static const struct ble_slave_handler_s slave_handler;
static KROUTINE_EXEC(slave_rq_done);

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define cprintk(...) do{}while(0)
//#define cprintk printk

struct ble_slave_s;

static void slave_request_schedule(struct ble_slave_s *slave);
static void slave_llcp_push(struct ble_slave_s *slave, struct buffer_s *buffer);


enum ble_slave_key_source_e
{
  BLE_SLAVE_KEY_SOURCE_NONE,
  BLE_SLAVE_KEY_SOURCE_STK,
  BLE_SLAVE_KEY_SOURCE_LTK,
};

/**
 BLE Slave ACL data layer.

 Handles connection data stream.

 Internally, this layer handles ACK-C PDUs (Connection control), it
 requires registration of a L2CAP layer above to handle packet
 fragmentation and L2CAP demux.
 */
struct ble_slave_s
{
  struct net_layer_s layer;

  const struct ble_slave_handler_s *handler;

  struct ble_private_s *pv;

  struct net_layer_s *l2cap;
  struct net_layer_s *gap;

  struct dev_ble_radio_rq_s ble_rq;

  struct ble_channel_mapper_s channel_mapper;
  struct ble_timing_mapper_s timing;

#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *security;
  struct dev_crypto_rq_s crypto_rq;
  struct dev_crypto_context_s ccm_tx_ctx;
  struct dev_crypto_context_s ccm_rx_ctx;
  struct net_task_s *ccm_task;
  net_task_queue_root_t ccm_queue;
  struct buffer_s *ccm_tmp_packet;
  uint8_t crypto_key[16];
#endif

  uint16_t last_event_counter;
  uint16_t scheduled_event_counter;

  uint16_t latency_backoff;
  uint16_t since_last_event_intervals;
  uint16_t since_last_event_unit;
  uint16_t to_next_event_unit;

  uint16_t tx_per_event_lp;
  uint16_t tx_queue_count;

  uint16_t features;

  uint8_t missed_event_count;

  struct net_task_s *conn_params_rq_pending;

  bool_t scheduled : 1;
  bool_t established : 1;
  bool_t dying : 1;
  bool_t version_sent : 1;
  bool_t features_sent : 1;
#if defined(CONFIG_BLE_CRYPTO)
  bool_t tx_encryption : 1;
  bool_t rx_encryption : 1;
  bool_t ccm_retry : 1;
#endif

  uint8_t reason;
};

STRUCT_COMPOSE(ble_slave_s, layer);


#if defined(CONFIG_BLE_CRYPTO)

static void slave_crypto_next(struct ble_slave_s *slave);

#define SLAVE_SUPPORTED_FEATURES (0                                     \
                                  | (1 << BLE_LL_FEATURE_LE_ENCRYPTION) \
                                  | (1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE) \
                                  | (1 << BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION) \
                                  | (1 << BLE_LL_FEATURE_LE_PING)       \
                                  )

#else

#define SLAVE_SUPPORTED_FEATURES (0                                     \
                                  | (1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE) \
                                  | (1 << BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION) \
                                  | (1 << BLE_LL_FEATURE_LE_PING)       \
                                  )

#endif

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

static void slave_error_push(struct ble_slave_s *slave, uint8_t error, bool_t send)
{
  if (slave->reason)
    return;

  slave->dying = 1;
  slave->reason = error;

  if (!send)
    return;

  struct buffer_s *term = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 4);
  term->data[term->begin + 0] = BLE_LL_CONTROL;
  term->data[term->begin + 1] = 2;
  term->data[term->begin + 2] = BLE_LL_TERMINATE_IND;
  term->data[term->begin + 3] = error;
  slave_llcp_push(slave, term);
  buffer_refdec(term);
}

static void slave_tx_enqueue(struct ble_slave_s *slave,
                             struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;

  (void)p;

  slave->tx_queue_count++;
  buffer_queue_pushback(&slave->ble_rq.data.tx_queue,
                        task->inbound.buffer);

#if defined(CONFIG_BLE_CRYPTO)
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
#endif

  dprintk("%s: @%lld %p ll %d [%P]\n", __FUNCTION__, task->inbound.timestamp,
         p, p->data[p->begin] & 0x3,
         p->data + p->begin,
         p->end - p->begin);

  dprintk("%s\n", __FUNCTION__);

  slave_request_schedule(slave);

  net_task_destroy(task);
}

static void slave_rx_enqueue(struct ble_slave_s *slave,
                             struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;

#if defined(CONFIG_BLE_CRYPTO)
  if ((p->data[p->begin] & 0x3) == BLE_LL_CONTROL) {
    switch (p->data[p->begin + 2]) {
    case BLE_LL_PAUSE_ENC_REQ:
      slave->rx_encryption = 0;
      break;
    }
  }
#endif

  task->inbound.dst_addr.llid = p->data[p->begin] & 0x3;
  p->begin += 2;

  dprintk("%s %lld: %p ll %d [%P]\n", __FUNCTION__, task->inbound.timestamp,
          p, task->inbound.dst_addr.llid,
          p->data + p->begin,
          p->end - p->begin);

  net_task_inbound_forward(task, &slave->layer);
}

#if defined(CONFIG_BLE_CRYPTO)
static KROUTINE_EXEC(slave_crypto_done)
{
  struct ble_slave_s *slave = KROUTINE_CONTAINER(kr, *slave, crypto_rq.rq.kr);
  struct net_task_s *task = slave->ccm_task;

  dprintk("%s %p %p\n", __FUNCTION__, slave, task);

  assert(task);

  // Replace packet in task
  
  if (slave->crypto_rq.err) {
    if (slave->ccm_retry) {
      printk("Slave ccm error %d at event %d\n", slave->crypto_rq.err, slave->last_event_counter);
      slave_error_push(slave, BLE_AUTHENTICATION_FAILURE, 1);

      buffer_refdec(task->inbound.buffer);
      task->inbound.buffer = slave->ccm_tmp_packet;
      slave->ccm_tmp_packet = NULL;
      slave->ccm_task = NULL;

      net_task_destroy(task);
      slave_crypto_next(slave);
      return;
    } else {
      printk("Slave ccm error %d at event %d, try again...\n", slave->crypto_rq.err, slave->last_event_counter);

      slave->ccm_retry = 1;

      // Task holding packet references slave layer
      kroutine_init(&slave->crypto_rq.rq.kr, slave_crypto_done, KROUTINE_INTERRUPTIBLE);
      DEVICE_OP(&slave->pv->crypto, request, &slave->crypto_rq);
      return;
    }
  }

  buffer_refdec(task->inbound.buffer);
  task->inbound.buffer = slave->ccm_tmp_packet;
  slave->ccm_tmp_packet = NULL;
  slave->ccm_task = NULL;

  if (!task->inbound.timestamp) {
    // Outbound
    dprintk("CRY < %P\n",
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);


    slave_crypto_next(slave);

    // Enqueue must be last thing: it may refdrop slave layer
    slave_tx_enqueue(slave, task);
  } else {
    // Inbound
    dprintk("CLR %d > %P\n",
           !task->inbound.dst_addr.unreliable,
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    task->inbound.src_addr.encrypted = 1;

    slave_crypto_next(slave);

    // Enqueue must be last thing: it may refdrop slave layer
    slave_rx_enqueue(slave, task);
  }
}

static void slave_crypto_next(struct ble_slave_s *slave)
{
  struct buffer_s *in, *out;
  struct net_task_s *task;
  uint16_t out_size = 0;

  if (slave->ccm_task)
    return;

  dprintk("%s\n", __FUNCTION__);

  slave->ccm_retry = 0;

 again:
  task = net_task_queue_pop(&slave->ccm_queue);

  if (!task)
    return;

  in = task->inbound.buffer;

  if (!task->inbound.timestamp) {
    // Outbound
    dprintk("CLR %d < %P\n",
           !task->inbound.dst_addr.unreliable,
           in->data + in->begin,
           in->end - in->begin);

    if (!slave->tx_encryption) {
      slave_tx_enqueue(slave, task);
      goto again;
    }

    slave->crypto_rq.op = DEV_CRYPTO_FINALIZE;
    slave->crypto_rq.ctx = &slave->ccm_tx_ctx;
    out_size = in->end - in->begin + 4;
  } else {
    // Inbound

    if (!slave->rx_encryption) {
      dprintk("CLR %d > %P\n",
             task->inbound.dst_addr.unreliable,
             in->data + in->begin,
             in->end - in->begin);

      slave_rx_enqueue(slave, task);
      goto again;
    }

    if (in->end - in->begin < 7) {
      dprintk("Short packet: %P\n",
             in->data + in->begin,
             in->end - in->begin);
      slave_error_push(slave, BLE_AUTHENTICATION_FAILURE, 1);
      net_task_destroy(task);
      goto again;
    }

    dprintk("CRY > %P\n",
           in->data + in->begin,
           in->end - in->begin);

    slave->crypto_rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
    slave->crypto_rq.ctx = &slave->ccm_rx_ctx;
    out_size = in->end - in->begin - 4;
  }

  assert(!slave->ccm_task);

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

  // Task holding packet references slave layer
  kroutine_init(&slave->crypto_rq.rq.kr, slave_crypto_done, KROUTINE_INTERRUPTIBLE);
  DEVICE_OP(&slave->pv->crypto, request, &slave->crypto_rq);
}

static void slave_ccm_enqueue(struct ble_slave_s *slave,
                              struct net_task_s *task)
{
  net_task_queue_pushback(&slave->ccm_queue, &task->header);
  slave_crypto_next(slave);
}
#endif

static void slave_llcp_push(struct ble_slave_s *slave,
                            struct buffer_s *buffer)
{
  struct net_task_s *task = net_scheduler_task_alloc(slave->layer.scheduler);
  net_task_inbound_push(task, &slave->layer, &slave->layer, 0, NULL, NULL, buffer);
}

static void slave_ref_cleanup(struct net_layer_s **layer)
{
  net_layer_refdec(*layer);
}

static void slave_request_schedule(struct ble_slave_s *slave)
{
  __attribute__((cleanup(slave_ref_cleanup)))
    struct net_layer_s *_layer = net_layer_refinc(&slave->layer);

  uint32_t event_advance;
  dev_timer_value_t now;
  error_t err;

  event_advance = __MIN(__MAX((int16_t)(slave->timing.current.latency + 1 - slave->since_last_event_intervals), 1),
                        __MIN(slave->timing.current.latency + 1, slave->latency_backoff));

  if (slave->timing.update_pending) {
    int16_t to_event = slave->timing.update_instant - slave->last_event_counter;
    if (to_event < event_advance)
      event_advance = to_event;
  }

  if (!slave->established || slave->missed_event_count || slave->reason)
    event_advance = 1;

  if (slave->ble_rq.data.md) {
    event_advance = 1;
  } else {
#if 0
    struct buffer_s *first = buffer_queue_head(&slave->ble_rq.data.tx_queue);
    if (first) {
      if (first->data[first->begin + 1])
        event_advance = 1;
      buffer_refdec(first);
    }
#else
    if (!buffer_queue_isempty(&slave->ble_rq.data.tx_queue))
      event_advance = 1;
#endif
  }

 scheduling_failed:
  DEVICE_OP(&slave->pv->timer, get_value, &now, 0);

  cprintk("Slave schedule %d/%d, %03d/%03d/%03d, reason %d",
         slave->last_event_counter, slave->scheduled_event_counter,
         event_advance, slave->latency_backoff, slave->timing.current.latency,
         slave->reason);

  if (now > slave->timing.drops_at) {
    cprintk(" dropped %lld > %lld\n", now, slave->timing.drops_at);
    slave_error_push(slave, BLE_CONNECTION_TIMEOUT, 1);
    goto drop_connection;
  }

  if (slave->scheduled) {
    cprintk(" scheduled");
    if (slave->scheduled_event_counter - slave->last_event_counter == event_advance) {
      cprintk(" is next\n");
      return;
    }

    if (DEVICE_OP(&slave->pv->radio, cancel, &slave->ble_rq) != 0) {
      cprintk(" cancel failed\n");
      return;
    } else {
      cprintk(" cancelled\n");
      net_layer_refdec(&slave->layer);
      slave->scheduled = 0;
    }
  } else  {
    cprintk("\n");
  }

 deadline_missed:
  DEVICE_OP(&slave->pv->timer, get_value, &now, 0);

  if (slave->reason && (buffer_queue_isempty(&slave->ble_rq.data.tx_queue)
                        || slave->since_last_event_intervals + event_advance > 6))
    goto drop_connection;

  slave->scheduled_event_counter = slave->last_event_counter + event_advance;

  cprintk(" event %d (%d + %d)\n", slave->scheduled_event_counter,
         slave->last_event_counter, event_advance);

  slave->ble_rq.data.channel = ble_channel_mapper_chan_get(&slave->channel_mapper,
                                                           slave->scheduled_event_counter);

  ble_timing_mapper_window_slave_get(&slave->timing,
                                     slave->scheduled_event_counter,
                                     &slave->ble_rq.not_before,
                                     &slave->ble_rq.not_after,
                                     &slave->ble_rq.max_duration);

  if (slave->ble_rq.not_before > slave->timing.drops_at) {
    cprintk(" too late, drops at %lld, scheduled at %lld, %lld too late\n",
            slave->timing.drops_at, slave->ble_rq.not_before,
            slave->ble_rq.not_before - slave->timing.drops_at);
    slave_error_push(slave, BLE_CONNECTION_TIMEOUT, 0);
    goto drop_connection;
  }

  if (slave->ble_rq.not_before < now) {
    event_advance++;
    cprintk(" deadline_missed %lld < %lld\n", slave->ble_rq.not_before, now);
    goto deadline_missed;
  }

  slave->ble_rq.data.latency_permitted = slave->established
    && ((slave->since_last_event_intervals + event_advance) < slave->latency_backoff);
  slave->ble_rq.data.tx_power = 0; //(slave->missed_event_count > 6) ? 8 * 4 : 0;

  if (slave->ble_rq.data.tx_power)
    cprintk("BOURRINAGE\n");

  dprintk(" scheduling at %llu->%llu on channel %d\n",
         slave->ble_rq.not_before, slave->ble_rq.not_after + slave->ble_rq.max_duration,
         slave->ble_rq.data.channel);

  kroutine_init(&slave->ble_rq.base.kr, slave_rq_done, KROUTINE_INTERRUPTIBLE);

  cprintk(" scheduled event %d at %lld on %d",
          slave->scheduled_event_counter,
          slave->ble_rq.not_before,
          slave->ble_rq.data.channel);

  err = DEVICE_OP(&slave->pv->radio, request, &slave->ble_rq);
  if (!err) {
    cprintk(" OK\n");
    net_layer_refinc(&slave->layer);
    slave->scheduled = 1;
    return;
  } else {
    cprintk(" error: %d\n", err);
    event_advance++;
    goto scheduling_failed;
  }

 drop_connection:
  cprintk("dropping connection\n");
  assert(slave->reason);

  if (slave->scheduled) {
    cprintk(" scheduled, cancelling\n");
    if (DEVICE_OP(&slave->pv->radio, cancel, &slave->ble_rq) == 0) {
      slave->scheduled = 0;
      net_layer_refdec(&slave->layer);
    }
  }
}

static
error_t ble_slave_init(
  struct ble_slave_s *slave,
  struct net_scheduler_s *scheduler,
  struct ble_private_s *pv,
  const struct ble_slave_param_s *params,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable)
{
  error_t err;
  struct ble_radio_info_s ble_info;
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_crypto_info_s crypto_info;
#endif

  memset(slave, 0, sizeof(*slave));

  err = net_layer_init(&slave->layer, &slave_handler, scheduler, delegate, delegate_vtable);
  if (err)
    return err;

  slave->pv = pv;

  DEVICE_OP(&slave->pv->radio, get_info, &ble_info);

  slave->layer.context.prefix_size = ble_info.prefix_size + 2;
  slave->layer.context.mtu = ble_info.mtu - 6;

  device_start(&slave->pv->radio);

#if defined(CONFIG_BLE_CRYPTO)
  DEVICE_OP(&slave->pv->crypto, info, &crypto_info);

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
#endif

  buffer_queue_init(&slave->ble_rq.data.tx_queue);
  buffer_queue_init(&slave->ble_rq.data.rx_queue);
  slave->ble_rq.type = DEVICE_BLE_RADIO_DATA_SLAVE;
  slave->ble_rq.packet_pool = scheduler->packet_pool;
  slave->ble_rq.data.access = params->conn_req.access_address;
  slave->ble_rq.data.crc_init = params->conn_req.crc_init;

  ble_channel_mapper_init(&slave->channel_mapper,
                          params->conn_req.channel_map,
                          params->conn_req.hop);

  ble_timing_mapper_init(&slave->timing,
                         &slave->pv->timer,
                         &params->conn_req,
                         params->connect_packet_timestamp);

  slave->since_last_event_intervals = 0;
  slave->latency_backoff = 1;

  slave_request_schedule(slave);

  dprintk("slave init done\n");

  return 0;

 err_out:
  return err;
}

static KROUTINE_EXEC(slave_rq_done)
{
  struct ble_slave_s *slave = KROUTINE_CONTAINER(kr, *slave, ble_rq.base.kr);

  slave->scheduled = 0;

  assert(!slave->ble_rq.base.drvdata);

  dprintk("Event %d done: %d\n", slave->scheduled_event_counter, slave->ble_rq.status);
  cprintk("Event %d done: %d, %d pkts, anchor %lld, lat perm %d\n  (%d tx, %d acked, %d empty, %d rxok, %d invalid, %d repeated, %d empty)\n",
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

  dprintk("Event %d done\n", slave->scheduled_event_counter);

  slave->established |= !!slave->ble_rq.data.tx_acked_count;
  slave->tx_queue_count = buffer_queue_count(&slave->ble_rq.data.tx_queue);
  slave->tx_per_event_lp += slave->ble_rq.data.tx_count - (slave->tx_per_event_lp >> 4);

  slave->since_last_event_intervals += (int16_t)(slave->scheduled_event_counter - slave->last_event_counter);
  slave->last_event_counter = slave->scheduled_event_counter;

  switch (slave->ble_rq.data.packet_count) {
  case 1:
    // We could still sync clocks
    slave->missed_event_count = 0;

    ble_channel_mapper_event_set(&slave->channel_mapper, slave->last_event_counter);
    ble_timing_mapper_event_set(&slave->timing, slave->last_event_counter,
                                slave->ble_rq.data.anchor);

    /* if (!slave->ble_rq.data.latency_permitted) */
    /*   slave->latency_backoff = 1; */
    break;

  default:
#if 0
    // Master will update its supervision timeout deadline only if it
    // sees a packet from us.  We may consider it had a packet from us
    // when it acknowledges.  If we transfer only 2 packets, consider
    // ours did not reach master until master does not retransmit,
    // else, consider we did not transmit at all.
    if ((slave->ble_rq.data.rx_repeated_count == 0 && slave->established)
        || slave->ble_rq.data.packet_count > 2) {
      slave->latency_backoff = 2 * slave->latency_backoff;
    } else {
      slave->latency_backoff = 1;
    }
#else
    slave->latency_backoff = 2 * slave->latency_backoff;
#endif

    ble_channel_mapper_event_set(&slave->channel_mapper, slave->last_event_counter);
    ble_timing_mapper_event_set(&slave->timing, slave->last_event_counter,
                                slave->ble_rq.data.anchor);

    if (slave->ble_rq.data.tx_count != slave->ble_rq.data.tx_empty_count)
      slave->latency_backoff = 1;

    // In any case, update missed event count, for not dropping the
    // connection too early.
    slave->missed_event_count = 0;

    // Sync clocks
    slave->since_last_event_intervals = 0;
    break;

  case 0:
    slave->missed_event_count++;
    if (!slave->ble_rq.data.latency_permitted)
      slave->latency_backoff = 1;
    else
      slave->latency_backoff = 2 * slave->latency_backoff;
    break;
  }

  if (slave->latency_backoff > slave->timing.current.latency)
    slave->latency_backoff = slave->timing.current.latency + 1;

  dprintk(" latency %d, backoff %d\n",
         slave->timing.current.latency, slave->latency_backoff);

  slave_request_schedule(slave);

  struct buffer_s *p;
  while ((p = buffer_queue_pop(&slave->ble_rq.data.rx_queue))) {
    if (!slave->reason) {
      struct net_task_s *task = net_scheduler_task_alloc(slave->layer.scheduler);
      dprintk("%s: [%P]\n", __FUNCTION__, p->data + p->begin, p->end - p->begin);
      net_task_inbound_push(task, &slave->layer, &slave->layer,
                            slave->ble_rq.data.anchor,
                            NULL, NULL, p);
    }
    buffer_refdec(p);
  }

  net_layer_refdec(&slave->layer);
}

static
void slave_llcp_task_handle(struct ble_slave_s *slave, struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;
  const uint8_t *args = &p->data[p->begin + 1];
  uint8_t reason = 0;
  struct buffer_s *rsp = task->inbound.buffer;

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ: {
    struct ble_conn_params_update update;

    ble_data_conn_params_update_parse(&p->data[p->begin], &update);

    error_t err = ble_timing_mapper_update_push(&slave->timing, &update);

    if (err)
      slave_error_push(slave, BLE_INSTANT_PASSED, 1);
    break;
  }

  case BLE_LL_CHANNEL_MAP_REQ: {
    error_t err = ble_channel_mapper_update_push(
        &slave->channel_mapper,
        endian_le16_na_load(args + 5),
        endian_le32_na_load(args) | ((uint64_t)args[4] << 32));

    if (err)
      slave_error_push(slave, BLE_INSTANT_PASSED, 1);
    break;
  }

  case BLE_LL_TERMINATE_IND:
    slave_error_push(slave, *args, 0);
    break;

  case BLE_LL_PING_REQ: {
    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 3;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_PING_RSP;

    goto reply;
  }

  case BLE_LL_VERSION_IND: {
    if (slave->version_sent)
      break;

    slave->version_sent = 1;

    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 8;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 6;
    rsp->data[rsp->begin + 2] = BLE_LL_VERSION_IND;
    rsp->data[rsp->begin + 3] = BLE_LL_VERSION_4_0;
    endian_le16_na_store(&rsp->data[rsp->begin + 4], 0xffff);
    endian_le16_na_store(&rsp->data[rsp->begin + 6], 0);

    goto reply;
  }

#if defined(CONFIG_BLE_CRYPTO)
  case BLE_LL_ENC_REQ: {
    uint8_t skd[16];
    uint8_t iv[8];
    error_t err;

    err = dev_rng_wait_read(&slave->pv->rng, skd + 8, 8);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto reject;
    }

    err = dev_rng_wait_read(&slave->pv->rng, iv + 4, 4);
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

    printk("SK:        %P\n", slave->crypto_key, 16);
    printk("IV:        %P\n", iv, 8);

    slave->crypto_rq.op = DEV_CRYPTO_INIT;
    slave->crypto_rq.ctx = &slave->ccm_tx_ctx;
    slave->crypto_rq.iv_ctr = iv;

    err = dev_crypto_wait_op(&slave->pv->crypto, &slave->crypto_rq);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto reject;
    }

    slave->crypto_rq.op = DEV_CRYPTO_INIT | DEV_CRYPTO_INVERSE;
    slave->crypto_rq.ctx = &slave->ccm_rx_ctx;
    slave->crypto_rq.iv_ctr = iv;

    err = dev_crypto_wait_op(&slave->pv->crypto, &slave->crypto_rq);
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

    rsp = task->inbound.buffer;
    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 3;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_START_ENC_REQ;

    goto reply;
  }

  case BLE_LL_START_ENC_RSP: {
    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 3;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_START_ENC_RSP;

    net_task_notification_push(net_scheduler_task_alloc(slave->layer.scheduler),
                               &slave->layer, &slave->layer,
                               SLAVE_CONTEXT_UPDATED);

    goto reply;
  }

  case BLE_LL_PAUSE_ENC_REQ: {
    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 3;

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
#else
  case BLE_LL_ENC_REQ:
  case BLE_LL_START_ENC_RSP:
  case BLE_LL_PAUSE_ENC_REQ:
  case BLE_LL_PAUSE_ENC_RSP:
    reason = BLE_PAIRING_NOT_ALLOWED;
    goto reject;
#endif

  case BLE_LL_CONNECTION_PARAM_REQ:
    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 26;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 24;
    rsp->data[rsp->begin + 2] = BLE_LL_CONNECTION_PARAM_RSP;
    endian_le16_na_store(rsp->data + rsp->begin + 3, 8);
    endian_le16_na_store(rsp->data + rsp->begin + 5, 12);
    endian_le16_na_store(rsp->data + rsp->begin + 7, 100);
    endian_le16_na_store(rsp->data + rsp->begin + 9, 200);
    rsp->data[rsp->begin + 11] = 8;
    endian_le16_na_store(rsp->data + rsp->begin + 12, 0);
    memset(rsp->data + rsp->begin + 14, 0xff, 12);

    goto reply;

  case BLE_LL_START_ENC_REQ:
  case BLE_LL_ENC_RSP:
    // Only initiated by slave, should not be received.
    slave_error_push(slave, BLE_COMMAND_DISALLOWED, 1);
    break;

  case BLE_LL_UNKNOWN_RSP:
  case BLE_LL_PING_RSP:
    // OSEF
    break;

  case BLE_LL_FEATURE_REQ: {
    if (slave->features_sent)
      break;

    slave->features_sent = 1;

    slave->features = SLAVE_SUPPORTED_FEATURES & endian_le64_na_load(p->data + p->begin + 1);

    rsp->begin = slave->layer.context.prefix_size;
    rsp->end = rsp->begin + 11;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 9;
    rsp->data[rsp->begin + 2] = BLE_LL_FEATURE_RSP;
    endian_le64_na_store(rsp->data + rsp->begin + 3, SLAVE_SUPPORTED_FEATURES);

    goto reply;
  }
    
  case BLE_LL_FEATURE_RSP:
  case BLE_LL_SLAVE_FEATURE_REQ:
    // TODO
    break;

  case BLE_LL_CONNECTION_PARAM_RSP:
    if (slave->conn_params_rq_pending) {
      net_task_query_respond_push(slave->conn_params_rq_pending, 0);
      slave->conn_params_rq_pending = NULL;
    }
    break;

  case BLE_LL_REJECT_IND:
    break;

  case BLE_LL_REJECT_IND_EXT:
    switch (p->data[p->begin + 1]) {
    case BLE_LL_CONNECTION_PARAM_REQ:
      if (slave->conn_params_rq_pending) {
        net_task_query_respond_push(slave->conn_params_rq_pending, -EINVAL);
        slave->conn_params_rq_pending = NULL;
      }
      break;
    }
    break;

  case BLE_LL_LENGTH_REQ:
  case BLE_LL_LENGTH_RSP:
    // Maybe
    break;
  }

  goto cleanup;

 reject:
  rsp->begin = slave->layer.context.prefix_size;
  rsp->end = rsp->begin + 4;

  rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
  rsp->data[rsp->begin + 1] = 2;
  rsp->data[rsp->begin + 2] = BLE_LL_REJECT_IND;
  rsp->data[rsp->begin + 3] = reason;

 reply:
  {
    struct net_addr_s dst = {};
    net_task_inbound_respond(task, 0, &dst);
  }
  return;

 cleanup:
  net_task_destroy(task);
}

static
void ble_slave_destroyed(struct net_layer_s *layer)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);

  printk("Slave %p destroyed\n", slave);

  if (slave->conn_params_rq_pending)
    net_task_destroy(slave->conn_params_rq_pending);

#if defined(CONFIG_BLE_CRYPTO)
  mem_free(slave->ccm_tx_ctx.state_data);
  mem_free(slave->ccm_rx_ctx.state_data);

  GCT_FOREACH(net_task_queue, &slave->ccm_queue, task,
              net_task_destroy(task);
              );

  net_task_queue_destroy(&slave->ccm_queue);

  assert(!slave->ccm_task);
  assert(!slave->ccm_tmp_packet);
#endif

  ble_timing_mapper_cleanup(&slave->timing);

  device_stop(&slave->pv->radio);

  mem_free(slave);
}

static
void ble_slave_task_handle(struct net_layer_s *layer,
                           struct net_task_s *task)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);

  dprintk("%s in %p %p -> %p", __FUNCTION__, slave, task->source, task->target);

  switch (task->type) {
  case NET_TASK_INBOUND:
    dprintk(" inbound @%lld %s, ll %d [%P]",
           task->inbound.timestamp,
           task->inbound.dst_addr.unreliable ? "bulk" : "reliable",
           task->inbound.dst_addr.llid,
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    if (task->source != &slave->layer) {
      // Packet from another layer

      // We are closing connection, dont enqueue anything
      if (slave->reason)
        break;

      uint8_t header[] = {
        task->inbound.dst_addr.llid,
        task->inbound.buffer->end - task->inbound.buffer->begin,
      };
      buffer_prepend(task->inbound.buffer, header, 2);
      // Delete timestamp, this marks packet as ours
      task->inbound.timestamp = 0;

      dprintk(" outgoing -> ccm\n");

      slave->latency_backoff = 1;

#if defined(CONFIG_BLE_CRYPTO)
      slave_ccm_enqueue(slave, task);
#else
      slave_tx_enqueue(slave, task);
#endif
      return;
    } else if (task->inbound.timestamp) {
      // Packet originating from radio

      switch (task->inbound.dst_addr.llid) {
      case BLE_LL_RESERVED:
        dprintk(" radio packet -> ccm\n");
        // Packet type not decoded yet, we are before crypto
#if defined(CONFIG_BLE_CRYPTO)
        slave_ccm_enqueue(slave, task);
#else
        slave_rx_enqueue(slave, task);
#endif
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

#if defined(CONFIG_BLE_CRYPTO)
      slave_ccm_enqueue(slave, task);
#else
      slave_tx_enqueue(slave, task);
#endif
      return;
    }
    break;

  case NET_TASK_NOTIFICATION:
    switch (task->notification.opcode) {
    case SLAVE_CONTEXT_UPDATED:
      slave->layer.context.addr.encrypted
#if defined(CONFIG_BLE_CRYPTO)
        = slave->layer.context.addr.authenticated
        = slave->tx_encryption && slave->rx_encryption;
#else
        = 0;
#endif
      net_layer_context_changed(&slave->layer);
      break;
    }
    break;

  case NET_TASK_QUERY:
    dprintk("Slave query, %S\n", &task->query.opcode, 4);

    switch (task->query.opcode) {
    case BLE_CONN_PARAMS_UPDATE: {
      struct ble_conn_params_update_task_s *up;
      struct buffer_s *pkt;

      up = ble_conn_params_update_task_s_from_task(task);

      if (up->interval_min <= slave->timing.current.interval
          && slave->timing.current.interval <= up->interval_max
          && slave->timing.current.latency == up->slave_latency) {
        net_task_query_respond_push(task, 0);
        return;
      }

      if (!(slave->features & BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE)) {
        net_task_query_respond_push(task, -ENOTSUP);
        return;
      }

      if (slave->conn_params_rq_pending) {
        net_task_query_respond_push(task, -EBUSY);
        return;
      }

      pkt = net_layer_packet_alloc(&slave->layer, slave->layer.context.prefix_size, 26);

      pkt->data[pkt->begin + 0] = BLE_LL_CONTROL;
      pkt->data[pkt->begin + 1] = 24;
      pkt->data[pkt->begin + 2] = BLE_LL_CONNECTION_PARAM_REQ;
      endian_le16_na_store(pkt->data + pkt->begin + 3, up->interval_min);
      endian_le16_na_store(pkt->data + pkt->begin + 5, up->interval_max);
      endian_le16_na_store(pkt->data + pkt->begin + 7, up->slave_latency);
      endian_le16_na_store(pkt->data + pkt->begin + 9, up->timeout);
      pkt->data[pkt->begin + 11] = 0;
      endian_le16_na_store(pkt->data + pkt->begin + 12, 0/*slave->scheduled_event_counter + 10*/);
      memset(pkt->data + pkt->begin + 14, 0xff, 12);
      slave_llcp_push(slave, pkt);
      buffer_refdec(pkt);

      slave->conn_params_rq_pending = task;

      // Dont destroy task
      return;
    }
    }

    break;

  default:
    dprintk(" other\n");
    break;
  }

  net_task_destroy(task);
}

static
error_t ble_slave_bound(struct net_layer_s *layer,
                        void *addr,
                        struct net_layer_s *child)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);
  uint16_t proto = *(uint16_t *)addr;

  // Child list takes the reference. Dont double it.
  switch (proto) {
  case BLE_SLAVE_LAYER_L2CAP:
    slave->l2cap = child;
    return 0;

//  case BLE_LAYER_TYPE_SECURITY:
//    slave->security = child;
//    return 0;

  case BLE_SLAVE_LAYER_GAP:
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

#if defined(CONFIG_BLE_CRYPTO)
  if (child == slave->security)
    slave->security = NULL;
#endif

  if (child == slave->gap)
    slave->gap = NULL;

  if (child == slave->l2cap) {
    slave->l2cap = NULL;

    if (!slave->reason) {
      dprintk("Slave l2cap unbound without connection closed\n");

      slave_error_push(slave, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST, 1);
    }
  }
}

error_t ble_slave_create(struct net_scheduler_s *scheduler,
                         struct ble_private_s *priv,
                         const void *params_,
                         void *delegate,
                         const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer)
{
  struct ble_slave_s *slave = mem_alloc(sizeof(*slave), mem_scope_sys);
  const struct ble_slave_param_s *params = params_;

  if (!slave)
    return -ENOMEM;

  error_t err = ble_slave_init(slave, scheduler, priv, params,
                               delegate, delegate_vtable);
  if (err)
    mem_free(slave);
  else
    *layer = &slave->layer;

  return err;
}

static
void slave_connection_close(struct net_layer_s *layer)
{
  struct ble_slave_s *slave = ble_slave_s_from_layer(layer);

  slave_error_push(slave, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST, 1);
}

static const struct ble_slave_handler_s slave_handler = {
  .base.destroyed = ble_slave_destroyed,
  .base.task_handle = ble_slave_task_handle,
  .base.bound = ble_slave_bound,
  .base.unbound = ble_slave_unbound,
  .base.type = BLE_NET_LAYER_SLAVE,
  .connection_close = slave_connection_close,
};
