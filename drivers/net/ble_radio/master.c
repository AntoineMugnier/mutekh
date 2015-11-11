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

#include <device/class/crypto.h>

#include <ble/net/master.h>
#include <ble/net/l2cap.h>
#include <ble/net/gap.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/data.h>

#define MASTER_CONTEXT_UPDATED 1

static const struct net_layer_handler_s master_handler;
static KROUTINE_EXEC(master_rq_done);

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define cprintk(...) do{}while(0)
//#define cprintk printk

static void master_request_schedule(struct ble_master_s *master);
static void master_llcp_push(struct ble_master_s *master, struct buffer_s *buffer);

#define MASTER_SUPPORTED_FEATURES (0                                     \
                                  | (1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE) \
                                  | (1 << BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION) \
                                  | (1 << BLE_LL_FEATURE_LE_PING)       \
                                  )

/**
 BLE Master ACL data layer.

 Handles connection data stream.

 Internally, this layer handles ACK-C PDUs (Connection control), it
 requires registration of a L2CAP layer above to handle packet
 fragmentation and L2CAP demux.
 */
struct ble_master_s
{
  struct net_layer_s layer;

  struct ble_channel_mapper_s channel_mapper;
  struct ble_timing_mapper_s timing;

  struct net_layer_s *l2cap;
  struct net_layer_s *gap;

  struct dev_ble_radio_rq_s ble_rq;

#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *security;
#endif

  uint16_t last_event_counter;
  uint16_t scheduled_event_counter;
  uint16_t missed_event_count;

  uint16_t tx_per_event_lp;
  uint16_t tx_queue_count;

  uint16_t features;

  uint8_t window;

  bool_t scheduled : 1;
  bool_t established : 1;
  bool_t dying : 1;
  bool_t version_sent : 1;
  bool_t features_sent : 1;

  uint8_t reason;

  struct ble_peer_s *peer;
};

STRUCT_COMPOSE(ble_master_s, layer);

static void master_error_push(struct ble_master_s *master, uint8_t error, bool_t send)
{
  if (master->reason)
    return;

  master->dying = 1;
  master->reason = error;

  if (!send)
    return;

  struct buffer_s *term = net_layer_packet_alloc(&master->layer, master->layer.context.prefix_size, 4);
  term->data[term->begin + 0] = BLE_LL_CONTROL;
  term->data[term->begin + 1] = 2;
  term->data[term->begin + 2] = BLE_LL_TERMINATE_IND;
  term->data[term->begin + 3] = error;
  master_llcp_push(master, term);
  buffer_refdec(term);
}

void ble_master_connection_close(struct ble_master_s *master)
{
  master_error_push(master, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST, 1);
}

static void master_tx_enqueue(struct ble_master_s *master,
                             struct net_task_s *task)
{
  __attribute__((unused))
  struct buffer_s *p = task->packet.buffer;

  master->tx_queue_count++;
  buffer_queue_pushback(&master->ble_rq.data.tx_queue,
                        task->packet.buffer);

  dprintk("%s: @%lld %p ll %d [%P]\n", __FUNCTION__, task->packet.timestamp,
         p, p->data[p->begin] & 0x3,
         p->data + p->begin,
         p->end - p->begin);

  dprintk("%s\n", __FUNCTION__);

  master_request_schedule(master);

  net_task_destroy(task);
}

static void master_rx_enqueue(struct ble_master_s *master,
                             struct net_task_s *task)
{
  struct buffer_s *p = task->packet.buffer;

  task->packet.dst_addr.llid = p->data[p->begin] & 0x3;
  p->begin += 2;

  dprintk("%s %lld: %p ll %d [%P]\n", __FUNCTION__, task->packet.timestamp,
          p, task->packet.dst_addr.llid,
          p->data + p->begin,
          p->end - p->begin);

  net_task_packet_forward(task, &master->layer);
}

static void master_llcp_push(struct ble_master_s *master,
                            struct buffer_s *buffer)
{
  struct net_task_s *task = net_scheduler_task_alloc(master->layer.scheduler);
  net_task_inbound_push(task, &master->layer, &master->layer, 0, NULL, NULL, buffer);
}

static void master_ref_cleanup(struct net_layer_s **layer)
{
  net_layer_refdec(*layer);
}

static void master_request_schedule(struct ble_master_s *master)
{
  __attribute__((cleanup(master_ref_cleanup)))
    struct net_layer_s *_layer = net_layer_refinc(&master->layer);

  uint32_t event_advance;
  dev_timer_value_t now;
  error_t err;

  event_advance = 1;

 scheduling_failed:
  DEVICE_OP(&master->timer, get_value, &now, 0);

  cprintk("Master schedule %d/%d, %03d/%03d/%03d, reason %d",
         master->last_event_counter, master->scheduled_event_counter,
         event_advance, master->latency_backoff, master->master_latency,
         master->reason);

  if (now > master->timing.drops_at) {
    cprintk(" dropped\n");
    master_error_push(master, BLE_CONNECTION_TIMEOUT, 1);
    goto drop_connection;
  }

  if (!master->established && master->missed_event_count > 6) {
    cprintk(" early dropped\n");
    master_error_push(master, BLE_CONNECTION_TIMEOUT, 0);
    goto drop_connection;
  }

  if (master->scheduled) {
    cprintk(" scheduled\n");
    return;
  }

 deadline_missed:
  DEVICE_OP(&master->timer, get_value, &now, 0);

  if (master->reason && buffer_queue_isempty(&master->ble_rq.data.tx_queue))
    goto drop_connection;

  master->scheduled_event_counter = master->last_event_counter + event_advance;
  master->ble_rq.data.channel = ble_channel_mapper_chan_get(&master->channel_mapper,
                                                            master->scheduled_event_counter);

  cprintk(" event %d (%d + %d)\n", master->scheduled_event_counter,
         master->last_event_counter, event_advance);

  ble_timing_mapper_window_get(&master->timing,
                               master->scheduled_event_counter,
                               &master->ble_rq.not_before,
                               &master->ble_rq.not_after,
                               &master->ble_rq.max_duration);

  if (master->ble_rq.not_before > master->timing.drops_at) {
    cprintk(" too late, drops at %lld, scheduled at %lld, %lld too late\n",
            master->drops_at, master->ble_rq.not_before,
            master->ble_rq.not_before - master->drops_at);
    master_error_push(master, BLE_CONNECTION_TIMEOUT, 0);
    goto drop_connection;
  }

  if (master->ble_rq.not_before < now) {
    event_advance++;
    cprintk(" deadline_missed\n");
    goto deadline_missed;
  }

  cprintk(" anchor %llu, ws %d\n", master->ble_rq.not_before, ws_tk);

  master->ble_rq.data.latency_permitted = 0;
  master->ble_rq.data.tx_power = 0;

  dprintk(" scheduling at %llu-%llu on channel %d\n",
         master->ble_rq.not_before, master->ble_rq.not_after,
         master->ble_rq.data.channel);

  kroutine_init(&master->ble_rq.base.kr, master_rq_done, KROUTINE_INTERRUPTIBLE);

  dprintk("master %p, dev %p, req %p\n", master, master->radio.dev, master->radio.api->f_request);

  assert(master->radio.dev);

  cprintk(" scheduled event %d at %lld",
         master->scheduled_event_counter, master->ble_rq.not_before);

  err = DEVICE_OP(&master->radio, request, &master->ble_rq);
  if (!err) {
    cprintk(" OK\n");
    net_layer_refinc(&master->layer);
    master->scheduled = 1;
    return;
  } else {
    cprintk(" error: %d\n", err);
    event_advance++;
    goto scheduling_failed;
  }

 drop_connection:
  cprintk("dropping connection\n");
  assert(master->reason);

  if (master->scheduled) {
    cprintk(" scheduled, cancelling\n");
    if (DEVICE_OP(&master->radio, cancel, &master->ble_rq) == 0) {
      master->scheduled = 0;
      net_layer_refdec(&master->layer);
    }
  }
}

static
error_t ble_master_init(
  struct ble_master_s *master,
  struct net_scheduler_s *scheduler,
  const char *ble,
  const char *crypto,
  struct dev_rng_s *rng,
  const struct ble_master_parameters_s *params)
{
  error_t err;
  struct ble_radio_info_s info;

  memset(master, 0, sizeof(*master));

  err = device_get_accessor_by_path(&master->radio, NULL, ble, DRIVER_CLASS_BLE_RADIO);
  if (err)
    goto err_out;

  err = device_get_accessor_by_path(&master->timer, NULL, ble, DRIVER_CLASS_TIMER);
  if (err)
    goto err_put_radio;

  err = net_layer_init(&master->layer, &master_handler, scheduler);
  if (err)
    goto err_put_devices;

  DEVICE_OP(&master->radio, get_info, &info);

  master->layer.context.prefix_size = info.prefix_size + 2;
  master->layer.context.mtu = info.mtu - 6;

  master->peer = params->peer;

  device_start(&master->radio);

  buffer_queue_init(&master->ble_rq.data.tx_queue);
  buffer_queue_init(&master->ble_rq.data.rx_queue);
  master->ble_rq.type = DEVICE_BLE_RADIO_DATA_MASTER;
  master->ble_rq.packet_pool = scheduler->packet_pool;
  master->ble_rq.data.access = params->conn_req.access_address;
  master->ble_rq.data.crc_init = params->conn_req.crc_init;

  ble_channel_mapper_init(&master->channel_mapper,
                          params->conn_req.channel_map,
                          params->conn_req.hop);

  ble_timing_mapper_init(&master->timing,
                         ble,
                         &params->conn_req,
                         params->connect_packet_timestamp);

  master_request_schedule(master);

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
  struct ble_master_s *master = KROUTINE_CONTAINER(kr, *master, ble_rq.base.kr);

  assert(!master->ble_rq.base.drvdata);

  dprintk("Event %d done: %d\n", master->scheduled_event_counter, master->ble_rq.status);
  cprintk("Event %d done: %d, %d pkts, anchor %lld, lat perm %d\n  (%d tx, %d acked, %d empty, %d rxok, %d invalid, %d repeated, %d empty)\n",
         master->scheduled_event_counter,
         master->ble_rq.status, master->ble_rq.data.packet_count,
         master->ble_rq.data.anchor,
         master->ble_rq.data.latency_permitted,
         master->ble_rq.data.tx_count,
         master->ble_rq.data.tx_acked_count,
         master->ble_rq.data.tx_empty_count,
         master->ble_rq.data.rx_ok_count,
         master->ble_rq.data.rx_invalid_count,
         master->ble_rq.data.rx_repeated_count,
         master->ble_rq.data.rx_empty_count);

  master->established |= !!master->ble_rq.data.tx_acked_count;
  master->tx_queue_count = buffer_queue_count(&master->ble_rq.data.tx_queue);
  master->tx_per_event_lp += master->ble_rq.data.tx_count - (master->tx_per_event_lp >> 4);

  master->missed_event_count += master->scheduled_event_counter - master->last_event_counter;
  master->last_event_counter = master->scheduled_event_counter;

  if (master->ble_rq.data.packet_count >= 2) {
    master->missed_event_count = 0;

    ble_channel_mapper_event_set(&master->channel_mapper, master->last_event_counter);
    ble_timing_mapper_event_set(&master->timing, master->last_event_counter,
                                master->ble_rq.data.anchor);
  }

  master_request_schedule(master);

  struct buffer_s *p;
  while ((p = buffer_queue_pop(&master->ble_rq.data.rx_queue))) {
    if (!master->reason) {
      struct net_task_s *task = net_scheduler_task_alloc(master->layer.scheduler);
      printk("%s: [%P]\n", __FUNCTION__, p->data + p->begin, p->end - p->begin);
      net_task_inbound_push(task, &master->layer, &master->layer,
                            master->ble_rq.data.anchor,
                            NULL, NULL, p);
    }
    buffer_refdec(p);
  }

  net_layer_refdec(&master->layer);
}

static
void master_llcp_task_handle(struct ble_master_s *master, struct net_task_s *task)
{
  struct buffer_s *p = task->packet.buffer;
  uint8_t reason = 0;
  struct buffer_s *rsp = task->packet.buffer;

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ:
  case BLE_LL_CHANNEL_MAP_REQ:
  case BLE_LL_ENC_REQ:
    // Invalid
    goto cleanup;

  case BLE_LL_TERMINATE_IND:
    master->reason = p->data[p->begin + 1];
    goto cleanup;

  case BLE_LL_PING_REQ: {
    rsp->begin = master->layer.context.prefix_size;
    rsp->end = rsp->begin + 3;
    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 1;
    rsp->data[rsp->begin + 2] = BLE_LL_PING_RSP;

    goto reply;
  }

  case BLE_LL_VERSION_IND: {
    if (master->version_sent)
      break;

    master->version_sent = 1;

    rsp->begin = master->layer.context.prefix_size;
    rsp->end = rsp->begin + 8;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 6;
    rsp->data[rsp->begin + 2] = BLE_LL_VERSION_IND;
    rsp->data[rsp->begin + 3] = BLE_LL_VERSION_4_0;
    endian_le16_na_store(&rsp->data[rsp->begin + 4], 0xffff);
    endian_le16_na_store(&rsp->data[rsp->begin + 6], 0);

    goto reply;
  }

  case BLE_LL_ENC_RSP:
  case BLE_LL_START_ENC_REQ:
  case BLE_LL_START_ENC_RSP:
  case BLE_LL_PAUSE_ENC_REQ:
  case BLE_LL_PAUSE_ENC_RSP:
    // TODO
    goto cleanup;

  case BLE_LL_UNKNOWN_RSP:
  case BLE_LL_PING_RSP:
    // OSEF
    goto cleanup;

  case BLE_LL_FEATURE_RSP:
    // TODO
    goto cleanup;

  case BLE_LL_FEATURE_REQ: {
    if (master->features_sent)
      break;

    master->features_sent = 1;

    master->features = MASTER_SUPPORTED_FEATURES & endian_le64_na_load(p->data + p->begin + 1);

    rsp->begin = master->layer.context.prefix_size;
    rsp->end = rsp->begin + 11;

    rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
    rsp->data[rsp->begin + 1] = 9;
    rsp->data[rsp->begin + 2] = BLE_LL_FEATURE_RSP;
    endian_le64_na_store(rsp->data + rsp->begin + 3, MASTER_SUPPORTED_FEATURES);

    goto reply;
  }

  case BLE_LL_CONNECTION_PARAM_REQ:
  case BLE_LL_CONNECTION_PARAM_RSP:
    // Who cares ?
    goto cleanup;

  case BLE_LL_REJECT_IND:
  case BLE_LL_REJECT_IND_EXT:
    // WTF
    goto cleanup;

  case BLE_LL_LENGTH_REQ:
  case BLE_LL_LENGTH_RSP:
    // Maybe
    goto cleanup;
  }

 reject:
  rsp->begin = master->layer.context.prefix_size;
  rsp->end = rsp->begin + 4;

  rsp->data[rsp->begin + 0] = BLE_LL_CONTROL;
  rsp->data[rsp->begin + 1] = 2;
  rsp->data[rsp->begin + 2] = BLE_LL_REJECT_IND;
  rsp->data[rsp->begin + 3] = reason;

 reply:
  {
    struct net_addr_s dst = {};
    net_task_packet_respond(task, 0, &dst);
  }
  return;

 cleanup:
  net_task_destroy(task);
}

static
void ble_master_destroyed(struct net_layer_s *layer)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);

  printk("Master %p destroyed\n", master);

  ble_timing_mapper_cleanup(&master->timing);

  device_stop(&master->radio);
  device_put_accessor(&master->timer);
  device_put_accessor(&master->radio);

  mem_free(master);
}

static
void ble_master_task_handle(struct net_layer_s *layer,
                           struct net_task_s *task)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);

  dprintk("%s in %p %p -> %p", __FUNCTION__, master, task->source, task->target);

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    dprintk(" outbound @%lld %s, ll %d [%P]",
           task->packet.timestamp,
           task->packet.dst_addr.unreliable ? "bulk" : "reliable",
           task->packet.dst_addr.llid,
           task->packet.buffer->data + task->packet.buffer->begin,
           task->packet.buffer->end - task->packet.buffer->begin);

    if (task->source != &master->layer) {
      // Packet from another layer

      // We are closing connection, dont enqueue anything
      if (master->reason)
        break;

      uint8_t header[] = {
        task->packet.dst_addr.llid,
        task->packet.buffer->end - task->packet.buffer->begin,
      };
      buffer_prepend(task->packet.buffer, header, 2);
      // Delete timestamp, this marks packet as ours
      task->packet.timestamp = 0;

      dprintk(" outgoing -> ccm\n");

#if defined(CONFIG_BLE_CRYPTO)
      master_ccm_enqueue(master, task);
#else
      master_tx_enqueue(master, task);
#endif
      return;
    } else if (task->packet.timestamp) {
      // Packet originating from radio

      switch (task->packet.dst_addr.llid) {
      case BLE_LL_RESERVED:
        dprintk(" radio packet -> ccm\n");
        // Packet type not decoded yet, we are before crypto
#if defined(CONFIG_BLE_CRYPTO)
        master_ccm_enqueue(master, task);
#else
        master_rx_enqueue(master, task);
#endif
        return;

      case BLE_LL_DATA_CONT:
      case BLE_LL_DATA_START:
        dprintk(" decrypted packet -> l2\n");
        if (!master->l2cap)
          break;

        net_task_packet_forward(task, master->l2cap);
        return;

      case BLE_LL_CONTROL:
        dprintk(" decrypted control\n");
        master_llcp_task_handle(master, task);
        return;
      }
    } else {
      // Outgoing packet from ourselves, not yet encrypted

      dprintk(" llcp from self -> ccm\n");

#if defined(CONFIG_BLE_CRYPTO)
      master_ccm_enqueue(master, task);
#else
      master_tx_enqueue(master, task);
#endif
      return;
    }
    break;

  case NET_TASK_NOTIFICATION:
    switch (task->notification.opcode) {
    case MASTER_CONTEXT_UPDATED:
      master->layer.context.addr.encrypted
        = master->layer.context.addr.authenticated
        = 0;
      net_layer_context_changed(&master->layer);
      break;
    }
    break;

  default:
    dprintk(" other\n");
    break;
  }

  net_task_destroy(task);
}

static
error_t ble_master_bound(struct net_layer_s *layer,
                        void *addr,
                        struct net_layer_s *child)
{
  struct ble_master_s *master = ble_master_s_from_layer(layer);
  uint16_t proto = *(uint16_t *)addr;

  // Child list takes the reference. Dont double it.
  switch (proto) {
  case BLE_MASTER_LAYER_L2CAP:
    master->l2cap = child;
    return 0;

//  case BLE_LAYER_TYPE_SECURITY:
//    master->security = child;
//    return 0;

  case BLE_MASTER_LAYER_GAP:
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

#if defined(CONFIG_BLE_CRYPTO)
  if (child == master->security)
    master->security = NULL;
#endif

  if (child == master->gap)
    master->gap = NULL;

  if (child == master->l2cap) {
    master->l2cap = NULL;

    if (!master->reason) {
      dprintk("Master l2cap unbound without connection closed\n");

      master_error_push(master, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST, 1);
    }
  }
}

static const struct net_layer_handler_s master_handler = {
  .destroyed = ble_master_destroyed,
  .task_handle = ble_master_task_handle,
  .bound = ble_master_bound,
  .unbound = ble_master_unbound,
  .type = BLE_NET_LAYER_MASTER,
};

error_t ble_master_create(struct net_scheduler_s *scheduler,
                          const char *ble,
                          const char *crypto,
                          struct dev_rng_s *rng,
                          const struct ble_master_parameters_s *params,
                          struct net_layer_s **layer)
{
  struct ble_master_s *master = mem_alloc(sizeof(*master), mem_scope_sys);

  if (!master)
    return -ENOMEM;

  error_t err = ble_master_init(master, scheduler, ble, crypto, rng, params);
  if (err)
    mem_free(master);
  else
    *layer = &master->layer;

  return err;
}
