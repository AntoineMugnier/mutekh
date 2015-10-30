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
#include <ble/util/datapath.h>
#include <ble/protocol/error.h>
#include <ble/net/slave.h>
#include <ble/net/layer.h>
#include <ble/peer.h>
#include "ble.h"
#include "ble_slave.h"

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

  struct net_layer_s *l2cap;
  struct net_layer_s *gap;
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s *rng;
#endif

  struct ble_channel_mapper_s channel_mapper;
  struct ble_timing_mapper_s timing;
  struct ble_datapath_s datapath;

  struct ble_peer_s *peer;
  struct net_task_s *conn_params_rq_pending;

  dev_timer_value_t anchor;

  uint32_t access_address;
  uint32_t crc_init;

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
};

STRUCT_COMPOSE(nrf5x_ble_slave_s, context);
STRUCT_COMPOSE(nrf5x_ble_slave_s, datapath);

static
void slave_llcp_push(struct nrf5x_ble_slave_s *slave, struct buffer_s *buffer)
{
  uint8_t header[] = {
    BLE_LL_CONTROL,
    buffer->end - buffer->begin,
  };

  buffer_prepend(buffer, header, 2);

  ble_datapath_packet_push(&slave->datapath, DATA_WAY_TX, buffer);
}

static
void nrf5x_ble_slave_error(struct nrf5x_ble_slave_s *slave, uint8_t reason, bool_t propagate)
{
  const struct ble_slave_delegate_vtable_s *vtable
    = const_ble_slave_delegate_vtable_s_from_base(slave->context.layer.delegate_vtable);

  if (slave->reason)
    return;

  slave->reason = reason;

  if (propagate) {
    struct buffer_s *pkt;

    pkt = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 2);
    pkt->data[pkt->begin + 0] = BLE_LL_TERMINATE_IND;
    pkt->data[pkt->begin + 1] = reason;

    slave_llcp_push(slave, pkt);
    buffer_refdec(pkt);
  }

  vtable->connection_closed(slave->context.layer.delegate,
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
      || !buffer_queue_isempty(&slave->datapath.way[DATA_WAY_TX].queue))
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

  if (slave->reason && (buffer_queue_isempty(&slave->datapath.way[DATA_WAY_TX].queue)
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
    nrf5x_ble_slave_error(slave, BLE_CONNECTION_TIMEOUT, 0);
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
static const struct ble_slave_handler_s ble_slave_layer_handler;
static const struct ble_datapath_handler_s ble_slave_data_handler;

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
  struct device_crypto_s *crypto = NULL;

  slave = mem_alloc(sizeof(*slave), mem_scope_sys);
  if (!slave)
    return -ENOMEM;

  memset(slave, 0, sizeof(*slave));

  err = device_get_accessor(&self_as_timer, priv->dev, DRIVER_CLASS_TIMER, 0);
  if (err)
    return err;

  err = nrf5x_ble_context_init(&slave->context,
                               scheduler,
                               &ble_slave_layer_handler.base,
                               priv, 
                               &ble_slave_ctx_handler,
                               delegate, delegate_vtable);
  if (err)
    goto err_put_timer;

  assert(CONFIG_BLE_PACKET_SIZE - 1 - 6 >= 27);

  slave->context.layer.context.prefix_size = 1 + 2;
  // Default MTU, may be negociated on nRF52 with packet length
  // extension, but not on nRF51 as there is a hard limitation in CCM.
  slave->context.layer.context.mtu = 27;

  slave->peer = params->peer;
#if defined(CONFIG_BLE_CRYPTO)
  slave->rng = params->rng;
#endif

  slave->access_address = params->conn_req.access_address;
  slave->crc_init = params->conn_req.crc_init;
  slave->peer = params->peer;

  slave->last_event_counter = -1;
  slave->scheduled_event_counter = -1;
  slave->latency_backoff = 1;

#if defined(CONFIG_BLE_CRYPTO)
  crypto = &priv->crypto;
#endif

  ble_datapath_init(&slave->datapath, &ble_slave_data_handler, crypto);

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
void slave_data_pending(struct ble_datapath_s *datapath)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_datapath(datapath);

  dprintk("%s\n", __FUNCTION__);

  if (!buffer_queue_isempty(&slave->datapath.way[DATA_WAY_TX].queue)) {
    slave_schedule(slave);
  }

  dev_timer_value_t now = net_scheduler_time_get(slave->context.layer.scheduler);
  struct buffer_s *p;

  while ((p = buffer_queue_pop(&slave->datapath.way[DATA_WAY_RX].queue))) {
    if (!slave->reason) {
      struct net_task_s *task = net_scheduler_task_alloc(slave->context.layer.scheduler);
      struct net_addr_s addr = {
        .llid = p->data[p->begin] & 0x3,
      };

      p->begin += 2;

      dprintk("%s: llid %d [%P]\n", __FUNCTION__, addr.llid,
        p->data + p->begin, p->end - p->begin);

      net_task_inbound_push(task, &slave->context.layer, &slave->context.layer,
        now, NULL, &addr, p);

      slave->latency_backoff = 1;
    }
    buffer_refdec(p);
  }
}

static
void slave_data_error(struct ble_datapath_s *datapath, uint8_t reason)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_datapath(datapath);

  nrf5x_ble_slave_error(slave, reason, 1);
}

static
void slave_layer_destroyed(struct net_layer_s *layer)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  nrf5x_ble_context_cleanup(&slave->context);
  ble_datapath_cleanup(&slave->datapath);
  ble_channel_mapper_cleanup(&slave->channel_mapper);
  ble_timing_mapper_cleanup(&slave->timing);

  mem_free(layer);
}

static void slave_query_task_handle(struct nrf5x_ble_slave_s *slave,
                                    struct net_task_s *task)
{
  struct ble_conn_params_update_task_s *up;
  struct buffer_s *pkt;

  switch (task->query.opcode) {
  case BLE_CONN_PARAMS_UPDATE:
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

    pkt = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 24);

    pkt->data[pkt->begin + 0] = BLE_LL_CONNECTION_PARAM_REQ;
    endian_le16_na_store(pkt->data + pkt->begin + 1, up->interval_min);
    endian_le16_na_store(pkt->data + pkt->begin + 3, up->interval_max);
    endian_le16_na_store(pkt->data + pkt->begin + 5, up->slave_latency);
    endian_le16_na_store(pkt->data + pkt->begin + 7, up->timeout);
    pkt->data[pkt->begin + 9] = 0;
    endian_le16_na_store(pkt->data + pkt->begin + 10, 0/*slave->scheduled_event_counter + 10*/);
    memset(pkt->data + pkt->begin + 12, 0xff, 12);

    slave_llcp_push(slave, pkt);
    buffer_refdec(pkt);

    slave->conn_params_rq_pending = task;

    // Dont destroy task
    return;
  }

  net_task_destroy(task);
}

static
void slave_llcp_task_handle(struct nrf5x_ble_slave_s *slave,
                            struct net_task_s *task)
{
  struct buffer_s *p = task->inbound.buffer;
  const uint8_t *args = &p->data[p->begin + 1];
  struct buffer_s *rsp = NULL;
  uint8_t reason = 0;
  error_t err;

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ: {
    struct ble_conn_params_update update;

    ble_data_conn_params_update_parse(&p->data[p->begin], &update);
    err = ble_timing_mapper_update_push(&slave->timing, &update);

    if (err)
      nrf5x_ble_slave_error(slave, BLE_INSTANT_PASSED, 1);
    break;
  }

  case BLE_LL_CHANNEL_MAP_REQ:
    err = ble_channel_mapper_update_push(
        &slave->channel_mapper,
        endian_le16_na_load(args + 5),
        endian_le32_na_load(args) | ((uint64_t)args[4] << 32));

    if (err)
      nrf5x_ble_slave_error(slave, BLE_INSTANT_PASSED, 1);
    break;

  case BLE_LL_TERMINATE_IND:
    nrf5x_ble_slave_error(slave, args[0], 0);
    break;

  case BLE_LL_PING_REQ: {
    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 1);

    rsp->data[rsp->begin + 0] = BLE_LL_PING_RSP;

    break;
  }

  case BLE_LL_VERSION_IND:
    if (slave->version_sent)
      break;

    slave->version_sent = 1;

    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 6);

    rsp->data[rsp->begin + 0] = BLE_LL_VERSION_IND;
    rsp->data[rsp->begin + 1] = BLE_LL_VERSION_4_0;
    endian_le16_na_store(&rsp->data[rsp->begin + 2], 0xffff);
    endian_le16_na_store(&rsp->data[rsp->begin + 4], 0);

    break;

#if defined(CONFIG_BLE_CRYPTO)
  case BLE_LL_ENC_REQ: {
    uint8_t skd[16];
    uint8_t sk[16];
    uint8_t iv[8];
    error_t err;

    err = dev_rng_wait_read(slave->rng, skd + 8, 8);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      break;
    }

    err = dev_rng_wait_read(slave->rng, iv + 4, 4);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      break;
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
                          sk);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      break;
    }

    err = ble_peer_sk_get(slave->peer, skd,
                          args, endian_le16_na_load(args + 8),
                          sk);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      break;
    }

    printk("SK:        %P\n", sk, 16);
    printk("IV:        %P\n", iv, 8);
    err = ble_datapath_encryption_setup(&slave->datapath, sk, iv);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      break;
    }

    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 13);
    rsp->data[rsp->begin + 0] = BLE_LL_ENC_RSP;
    memcpy(rsp->data + rsp->begin + 1, skd + 8, 8);
    memcpy(rsp->data + rsp->begin + 9, iv + 4, 4);

    slave_llcp_push(slave, rsp);
    buffer_refdec(rsp);

    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 1);

    rsp->data[rsp->begin + 0] = BLE_LL_START_ENC_REQ;

    break;
  }

  case BLE_LL_START_ENC_RSP: {
    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 1);

    rsp->data[rsp->begin + 0] = BLE_LL_START_ENC_RSP;

    net_task_notification_push(net_scheduler_task_alloc(slave->context.layer.scheduler),
                               &slave->context.layer, &slave->context.layer,
                               SLAVE_CONTEXT_UPDATED);

    break;
  }

  case BLE_LL_PAUSE_ENC_REQ: {
    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 1);

    rsp->data[rsp->begin + 0] = BLE_LL_PAUSE_ENC_RSP;

    net_task_notification_push(net_scheduler_task_alloc(slave->context.layer.scheduler),
                               &slave->context.layer, &slave->context.layer,
                               SLAVE_CONTEXT_UPDATED);

    break;
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
    break;
#endif

  case BLE_LL_CONNECTION_PARAM_REQ:
    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 24);

    rsp->data[rsp->begin + 0] = BLE_LL_CONNECTION_PARAM_RSP;
    endian_le16_na_store(rsp->data + rsp->begin + 1, 8);
    endian_le16_na_store(rsp->data + rsp->begin + 3, 12);
    endian_le16_na_store(rsp->data + rsp->begin + 5, 100);
    endian_le16_na_store(rsp->data + rsp->begin + 7, 200);
    rsp->data[rsp->begin + 9] = 8;
    endian_le16_na_store(rsp->data + rsp->begin + 10, 0);
    memset(rsp->data + rsp->begin + 12, 0xff, 12);

    break;

  case BLE_LL_START_ENC_REQ:
  case BLE_LL_ENC_RSP:
    // Only initiated by slave, should not be received.
    nrf5x_ble_slave_error(slave, BLE_COMMAND_DISALLOWED, 1);
    break;

  case BLE_LL_UNKNOWN_RSP:
  case BLE_LL_PING_RSP:
    // OSEF
    break;

  case BLE_LL_FEATURE_REQ: {
    if (slave->features_sent)
      break;

    slave->features_sent = 1;

    slave->features = SUPPORTED_FEATURES & endian_le64_na_load(p->data + p->begin + 1);

    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 9);

    rsp->data[rsp->begin + 0] = BLE_LL_FEATURE_RSP;
    endian_le64_na_store(rsp->data + rsp->begin + 1, SUPPORTED_FEATURES);

    break;
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

  if (reason && !rsp) {
    rsp = net_layer_packet_alloc(&slave->context.layer, slave->context.layer.context.prefix_size + 2, 2);

    rsp->data[rsp->begin + 0] = BLE_LL_REJECT_IND;
    rsp->data[rsp->begin + 1] = reason;
  }

  if (rsp) {
    slave_llcp_push(slave, rsp);
    buffer_refdec(rsp);
  }

  net_task_destroy(task);
  return;
}

static
void slave_layer_task_handle(struct net_layer_s *layer,
                             struct net_task_header_s *header)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);
  struct net_task_s *task = net_task_s_from_header(header);

  dprintk("%s in %p %p -> %p", __FUNCTION__, slave, task->header.source, task->header.target);

  switch (header->type) {
  case NET_TASK_INBOUND:
    dprintk(" inbound @%lld %s, ll %d [%P]",
           task->inbound.timestamp,
           task->inbound.dst_addr.unreliable ? "bulk" : "reliable",
           task->inbound.dst_addr.llid,
           task->inbound.buffer->data + task->inbound.buffer->begin,
           task->inbound.buffer->end - task->inbound.buffer->begin);

    if (task->header.source != &slave->context.layer) {
      // Packet from another layer

      // We are closing connection, dont enqueue anything
      if (slave->reason)
        break;

      uint8_t header[] = {
        task->inbound.dst_addr.llid,
        task->inbound.buffer->end - task->inbound.buffer->begin,
      };
      buffer_prepend(task->inbound.buffer, header, 2);

      dprintk(" outgoing -> ccm\n");

      ble_datapath_packet_push(&slave->datapath, DATA_WAY_TX, task->inbound.buffer);
      break;
    }

    // Packet originating from ourselves
    switch (task->inbound.dst_addr.llid) {
    case BLE_LL_RESERVED:
      dprintk(" broken packet\n");
      nrf5x_ble_slave_error(slave, BLE_INVALID_PARAMETERS, 1);
      break;

    case BLE_LL_DATA_CONT:
    case BLE_LL_DATA_START:
      dprintk(" data packet -> l2cap\n");
      if (!slave->l2cap)
        break;

      net_task_inbound_forward(task, slave->l2cap);
      return;

    case BLE_LL_CONTROL:
      dprintk(" control\n");
      slave_llcp_task_handle(slave, task);
      return;
    }
    break;

  case NET_TASK_NOTIFICATION:
    switch (task->notification.opcode) {
    case SLAVE_CONTEXT_UPDATED:
      slave->context.layer.context.addr.encrypted
        = slave->context.layer.context.addr.authenticated
#if defined(CONFIG_BLE_CRYPTO)
        = slave->datapath.way[DATA_WAY_TX].crypto_enabled
          && slave->datapath.way[DATA_WAY_RX].crypto_enabled;
#else
        = 0;
#endif
      net_layer_context_changed(&slave->context.layer);
      break;
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
error_t slave_layer_bound(struct net_layer_s *layer,
                          void *addr,
                          struct net_layer_s *child)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);
  uint16_t proto = *(uint16_t *)addr;

  // Child list takes the reference. Dont double it.
  switch (proto) {
  case BLE_SLAVE_LAYER_L2CAP:
    slave->l2cap = child;
    return 0;

  case BLE_SLAVE_LAYER_GAP:
    slave->gap = child;
    return 0;

  default:
    return -ENOTSUP;
  }
}

static
void slave_layer_unbound(struct net_layer_s *layer,
                         struct net_layer_s *child)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  if (child == slave->gap)
    slave->gap = NULL;

  if (child == slave->l2cap) {
    slave->l2cap = NULL;

    if (!slave->reason) {
      dprintk("Slave l2cap unbound without connection closed\n");

      nrf5x_ble_slave_error(slave, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST, 1);
    }
  }
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
    slave->latency_backoff = slave->timing.current.latency;

  cprintk(" latency %d, backoff %d\n",
         slave->timing.current.latency, slave->latency_backoff);

  slave->opened = 0;

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
    || !buffer_queue_isempty(&slave->datapath.way[DATA_WAY_TX].queue)
    || slave->peer_md
    || (slave->event_packet_count >= 2 && (slave->event_packet_count & 1));
}

static
struct buffer_s *slave_ctx_payload_get(struct nrf5x_ble_context_s *context)
{
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(context);
  struct buffer_s *packet, *next;

  if (buffer_queue_isempty(&slave->datapath.way[DATA_WAY_TX].queue)) {
    // Tx queue is empty, we have to create an empty packet to piggyback
    // handshaking.  Cannot borrow reference to a common packet as it is
    // chained in tx queue.
    packet = net_layer_packet_alloc(&slave->context.layer,
                                    slave->context.layer.context.prefix_size, 2);

    packet->data[packet->begin + 0] = BLE_LL_DATA_CONT;
    packet->data[packet->begin + 1] = 0;

    buffer_queue_pushback(&slave->datapath.way[DATA_WAY_TX].queue, packet);
    next = buffer_queue_next(&slave->datapath.way[DATA_WAY_TX].queue, packet);
  } else {
    packet = buffer_queue_head(&slave->datapath.way[DATA_WAY_TX].queue);
    next = NULL;
  }

  packet->data[packet->begin] = (packet->data[packet->begin] & 0x3)
    // If head packet has data in it, force master to ack it.
    | (packet->data[packet->begin + 1] ? BLE_LL_DATA_MD : 0)
    // Normal MD semantics
    | (next ? BLE_LL_DATA_MD : 0)
    | (slave->nesn ? BLE_LL_DATA_NESN : 0)
    | (slave->sn ? BLE_LL_DATA_SN : 0);

  if (next)
    buffer_refdec(next);

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

  if (slave->nesn == sn && !empty)
    ble_datapath_packet_push(&slave->datapath, DATA_WAY_RX, packet);

  slave->peer_md = md;
  slave->nesn = !sn;

  if (nesn != slave->sn) {
    slave->event_acked_count++;
    slave->sn = nesn;

    struct buffer_s *packet = buffer_queue_pop(&slave->datapath.way[DATA_WAY_TX].queue);

    if (packet)
      buffer_refdec(packet);
  }
}

static
void slave_connection_close(struct net_layer_s *layer)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_s_from_layer(layer);
  struct nrf5x_ble_slave_s *slave = nrf5x_ble_slave_s_from_context(ctx);

  nrf5x_ble_slave_error(slave, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST, 1);
}

static const struct ble_slave_handler_s ble_slave_layer_handler = {
  .base.destroyed = slave_layer_destroyed,
  .base.task_handle = slave_layer_task_handle,
  .base.bound = slave_layer_bound,
  .base.unbound = slave_layer_unbound,
  .base.type = BLE_NET_LAYER_SLAVE,
  .connection_close = slave_connection_close,
};

static const struct nrf5x_ble_context_handler_s ble_slave_ctx_handler = {
  .event_opened = slave_ctx_event_opened,
  .event_closed = slave_ctx_event_closed,
  .radio_params = slave_ctx_radio_params,
  .payload_get = slave_ctx_payload_get,
  .ifs_event = slave_ctx_ifs_event,
  .payload_received = slave_ctx_payload_received,
};

static const struct ble_datapath_handler_s ble_slave_data_handler = {
  .pending = slave_data_pending,
  .error = slave_data_error,
};
