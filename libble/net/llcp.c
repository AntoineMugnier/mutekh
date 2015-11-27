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

#include <ble/net/llcp.h>
#include <ble/net/generic.h>
#include <ble/net/gap.h>
#include <ble/protocol/data.h>
#include <ble/protocol/error.h>

#include <ble/peer.h>
#include <device/class/crypto.h>

//#define dprintk printk
#define dprintk(...) do{}while(0)

struct ble_llcp_s;

struct ble_llcp_handler_s;

#define SUPPORTED_FEATURES (0                                  \
                            | (_CONFIG_BLE_CRYPTO << BLE_LL_FEATURE_LE_ENCRYPTION) \
                            | (1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE) \
                            | (1 << BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION) \
                            | (1 << BLE_LL_FEATURE_LE_PING)             \
                            )

/**
 BLE LLCP layer.

 Handles fragmentation and CID multiplexing.  For a compliant LE
 device, this layer expects SM and ATT layers.
 */
struct ble_llcp_s
{
  struct net_layer_s layer;

  uint8_t local_skd[8];

  struct net_layer_s *gap;
  struct net_task_s *feature_req_later;
  net_task_queue_root_t pending;

#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s *rng;
  struct ble_peer_s *peer;
#endif

  uint8_t error;
  uint8_t features;

  bool_t version_sent : 1;
  bool_t features_requested : 1;
};

STRUCT_COMPOSE(ble_llcp_s, layer);

static void llcp_connection_closed(struct ble_llcp_s *llcp, uint8_t err)
{
  if (llcp->error)
    return;

  llcp->error = err;

  const struct ble_llcp_delegate_vtable_s *vtable
    = const_ble_llcp_delegate_vtable_s_from_base(llcp->layer.delegate_vtable);

  if (llcp->layer.delegate && vtable->connection_closed)
    vtable->connection_closed(llcp->layer.delegate, &llcp->layer, err);
}

static bool_t llcp_is_slave(struct ble_llcp_s *llcp)
{
  return 1;
}

static
void llcp_query_pending_respond(struct ble_llcp_s *llcp, uint32_t type, error_t err)
{
  struct net_task_s *found = NULL;

  GCT_FOREACH(net_task_queue, &llcp->pending, task,
              if (!type || task->query.opcode == type) {
                found = task;
                net_task_queue_nolock_remove(&llcp->pending, task);
                GCT_FOREACH_BREAK;
              });

  if (!found)
    return;

  net_task_query_respond_push(found, err);
}

static
bool_t llcp_query_is_pending(struct ble_llcp_s *llcp, uint32_t type)
{
  bool_t ret = 0;

  GCT_FOREACH(net_task_queue, &llcp->pending, task,
              if (task->query.opcode == type) {
                ret = 1;
                GCT_FOREACH_BREAK;
              });

  return ret;
}

static void ble_llcp_packet_handle(struct ble_llcp_s *llcp, struct net_task_s *task)
{
  struct buffer_s *p = task->packet.buffer;
  const uint8_t *args = &p->data[p->begin + 1];
  uint8_t reason = 0;
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };
  bool_t is_slave = llcp_is_slave(llcp);

  dprintk("LLCP %s req %P...", is_slave ? "slave" : "master", p->data + p->begin, p->end - p->begin);

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ: {
    if (!is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent || !llcp->layer.parent->parent)
      break;

    struct ble_llcp_connection_parameters_update_s *up
      = mem_alloc(sizeof(*up), mem_scope_sys);

    if (!up) {
      reason = BLE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES;
      goto error;
    }

    up->task.destroy_func = memory_allocator_push;

    dprintk("connection parameters update\n");

    ble_data_conn_params_update_parse(&p->data[p->begin], &up->update);
    net_task_query_push(&up->task, llcp->layer.parent->parent, &llcp->layer,
                        BLE_LLCP_CONNECTION_PARAMETERS_UPDATE);
    break;
  }

  case BLE_LL_CHANNEL_MAP_REQ: {
    if (!is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent || !llcp->layer.parent->parent)
      break;

    struct ble_llcp_channel_map_update_s *up
      = mem_alloc(sizeof(*up), mem_scope_sys);

    if (!up) {
      reason = BLE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES;
      goto error;
    }

    up->task.destroy_func = memory_allocator_push;

    dprintk("channel map update\n");

    up->channel_map = endian_le32_na_load(args) | ((uint64_t)args[4] << 32);
    up->instant = endian_le16_na_load(args + 5);
    net_task_query_push(&up->task, llcp->layer.parent->parent, &llcp->layer,
                        BLE_LLCP_CHANNEL_MAP_UPDATE);
    break;
  }

  case BLE_LL_TERMINATE_IND:
    dprintk("terminated %d\n", args[0]);

    llcp_connection_closed(llcp, args[0]);
    break;

  case BLE_LL_PING_REQ:
    dprintk("ping req\n");

    p->data[p->begin] = BLE_LL_PING_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_VERSION_IND:
    if (llcp->version_sent)
      break;

    dprintk("version ind\n");

    llcp->version_sent = 1;

    p->data[p->begin + 0] = BLE_LL_VERSION_IND;
    p->data[p->begin + 1] = BLE_LL_VERSION_4_2;
    endian_le16_na_store(&p->data[p->begin + 2], 0xffff);
    endian_le16_na_store(&p->data[p->begin + 4], 0);
    p->end = p->begin + 6;

    goto respond;

#if defined(CONFIG_BLE_CRYPTO)
  case BLE_LL_ENC_REQ: {
    uint8_t skd[16];

    dprintk("enc req\n");

    if (!is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent)
      break;

    struct ble_llcp_encryption_setup_s *setup
      = mem_alloc(sizeof(*setup), mem_scope_sys);

    if (!setup) {
      reason = BLE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES;
      goto error;
    }

    setup->task.destroy_func = memory_allocator_push;

    error_t err;

    err = dev_rng_wait_read(llcp->rng, llcp->local_skd, 8);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto error;
    }

    err = dev_rng_wait_read(llcp->rng, setup->iv + 4, 4);
    if (err) {
      reason = BLE_HARDWARE_FAILURE;
      goto error;
    }

    dprintk("skdm:      %P\n", args + 10, 8);
    dprintk("ivm:       %P\n", args + 18, 4);

    dprintk("rand:      %P\n", args, 8);
    dprintk("ediv:      %d\n", endian_le16_na_load(args + 8));

    dprintk("skds:      %P\n", llcp->local_skd, 8);
    dprintk("ivs:       %P\n", setup->iv + 4, 4);

    memcpy(skd, args + 10, 8);
    memcpy(skd + 8, llcp->local_skd, 8);

    memcpy(setup->iv, args + 18, 4);

    err = ble_peer_sk_get(llcp->peer, skd,
                          args, endian_le16_na_load(args + 8),
                          setup->sk);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      goto error;
    }

    dprintk("SK:        %P\n", setup->sk, 16);
    dprintk("IV:        %P\n", setup->iv, 8);

#if defined(CONFIG_BLE_SECURITY_DB)
    setup->authenticated = llcp->peer->mitm_protection;
#else
    setup->authenticated = 0;
#endif

    net_task_query_push(&setup->task, llcp->layer.parent, &llcp->layer,
                        BLE_LLCP_ENCRYPTION_SETUP);
    break;
  }

  case BLE_LL_ENC_RSP:
    if (is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent)
      break;

    dprintk("enc rsp\n");

    // TODO master crypto
    reason = BLE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE;
    goto error;

  case BLE_LL_START_ENC_REQ:
    if (is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent)
      break;

    dprintk("start enc req\n");

    p->data[p->begin + 0] = BLE_LL_START_ENC_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_START_ENC_RSP:
    if (!is_slave)
      break;

    if (!llcp->layer.parent)
      break;

    dprintk("start enc rsp\n");

    p->data[p->begin + 0] = BLE_LL_START_ENC_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_PAUSE_ENC_REQ:
    if (!is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent)
      break;

    dprintk("pause enc req\n");

    p->data[p->begin + 0] = BLE_LL_PAUSE_ENC_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_PAUSE_ENC_RSP:
    if (is_slave)
      break;

    dprintk("pause enc rsp\n");

    p->data[p->begin + 0] = BLE_LL_PAUSE_ENC_RSP;
    p->end = p->begin + 1;

    goto respond;
#endif

#if 0
  case BLE_LL_CONNECTION_PARAM_REQ:
    if (is_slave) {
      p->data[p->begin + 0] = BLE_LL_CONNECTION_PARAM_RSP;
      // TODO: fetch parameters from delegate
      endian_le16_na_store(p->data + p->begin + 1, 8);
      endian_le16_na_store(p->data + p->begin + 3, 12);
      endian_le16_na_store(p->data + p->begin + 5, 100);
      endian_le16_na_store(p->data + p->begin + 7, 200);
      p->data[p->begin + 9] = 8;
      endian_le16_na_store(p->data + p->begin + 10, 0);
      memset(p->data + p->begin + 12, 0xff, 12);
      p->end = p->begin + 24;

      goto respond;
    } else {
      // TODO: Handle slave-initiated connection update
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }
#endif

  case BLE_LL_PING_RSP:
    dprintk("ping rsp\n");
    llcp_query_pending_respond(llcp, BLE_LLCP_PING, 0);
    break;

  case BLE_LL_UNKNOWN_RSP:
    switch (args[0]) {
    case BLE_LL_PING_REQ:
      llcp->features |= 0x80;
      llcp->features &= ~(1 << BLE_LL_FEATURE_LE_PING);
      llcp_query_pending_respond(llcp, BLE_LLCP_PING, -ENOTSUP);
      break;

    case BLE_LL_CONNECTION_PARAM_REQ:
      llcp->features |= 0x80;
      llcp->features &= ~(1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE);
      llcp_query_pending_respond(llcp, BLE_GAP_CONN_PARAMS_UPDATE, -ENOTSUP);
      break;
    }
    break;

  case BLE_LL_REJECT_IND:
    // Dont care
    llcp_query_pending_respond(llcp, 0, -ENOTSUP);
    break;

  case BLE_LL_FEATURE_REQ:
    if (!is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    send_features:

    dprintk("features\n");

    llcp->features = SUPPORTED_FEATURES & endian_le64_na_load(args);

    p->data[p->begin + 0] = BLE_LL_FEATURE_RSP;
    endian_le64_na_store(p->data + p->begin + 1, SUPPORTED_FEATURES);
    p->end = p->begin + 9;

    goto respond;
    
  case BLE_LL_FEATURE_RSP:
    llcp->features = SUPPORTED_FEATURES & endian_le64_na_load(args);
    break;

  case BLE_LL_SLAVE_FEATURE_REQ:
    if (is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }
    goto send_features;

  case BLE_LL_CONNECTION_PARAM_RSP:
    llcp_query_pending_respond(llcp, BLE_GAP_CONN_PARAMS_UPDATE, -ENOTSUP);
    break;

  case BLE_LL_REJECT_IND_EXT:
    dprintk("reject %d %d\n", args[0], args[1]);

    switch (args[0]) {
    case BLE_LL_CONNECTION_PARAM_REQ:
      llcp_query_pending_respond(llcp, BLE_GAP_CONN_PARAMS_UPDATE, -EINVAL);
      break;

    case BLE_LL_PING_REQ:
      llcp_query_pending_respond(llcp, BLE_LLCP_PING, -EINVAL);
      break;
    }
    break;

  case BLE_LL_LENGTH_REQ: {
    uint16_t unit_size = net_scheduler_packet_mtu(llcp->layer.scheduler);
    unit_size -= llcp->layer.context.prefix_size;
    unit_size -= 4;
    unit_size = __MIN(unit_size, 251);

    uint16_t acceptable_mtu = __MIN(unit_size, endian_le16_na_load(args + 1));
    acceptable_mtu = __MIN(acceptable_mtu, 251);

    p->data[p->begin + 0] = BLE_LL_LENGTH_RSP;
    // RX MTU, what we can locally do
    endian_le16_na_store(p->data + p->begin + 1, unit_size);
    endian_le16_na_store(p->data + p->begin + 3, unit_size * 8 + 14);
    // TX MTU, what we will do based on peer's capabilities
    endian_le16_na_store(p->data + p->begin + 5, acceptable_mtu);
    endian_le16_na_store(p->data + p->begin + 7, acceptable_mtu * 8 + 14);
    p->end = p->begin + 9;

    goto respond;
  }

  case BLE_LL_LENGTH_RSP:
    
    break;
  }

  dprintk("Done\n");

  net_task_destroy(task);
  return;

 error:
  dprintk("rejected: %d\n", reason);

  p->data[p->begin + 1] = p->data[p->begin + 0];
  p->data[p->begin + 0] = BLE_LL_REJECT_IND_EXT;
  p->data[p->begin + 2] = reason;

 respond:
  if (llcp->layer.parent)
    net_task_packet_respond(task, llcp->layer.parent, 0, &dst);
  else
    net_task_destroy(task);
}

static void ble_llcp_response_handle(struct ble_llcp_s *llcp, struct net_task_s *task)
{
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };
  struct net_task_s *in;
  struct buffer_s *p;
  bool_t is_slave = llcp_is_slave(llcp);

  switch (task->query.opcode) {
  case BLE_LLCP_ENCRYPTION_SETUP: {
    struct ble_llcp_encryption_setup_s *setup
      = ble_llcp_encryption_setup_s_from_task(task);

      if (!llcp->layer.parent)
        break;

    if (task->query.err && is_slave) {
      p = net_layer_packet_alloc(&llcp->layer,
                                 llcp->layer.context.prefix_size,
                                 3);
      if (!p)
        break;

      in = net_scheduler_task_alloc(llcp->layer.scheduler);
      if (!in) {
        buffer_refdec(p);
        break;
      }

      p->data[p->begin + 0] = BLE_LL_REJECT_IND_EXT;
      p->data[p->begin + 1] = BLE_LL_ENC_REQ;
      p->data[p->begin + 2] = task->query.err;
      net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                            0, NULL, &dst, p);
      buffer_refdec(p);
      break;
    }

    p = net_layer_packet_alloc(&llcp->layer,
                               llcp->layer.context.prefix_size,
                               13);
    if (!p)
      break;
    in = net_scheduler_task_alloc(llcp->layer.scheduler);
    if (!in) {
      buffer_refdec(p);
      break;
    }

    p->data[p->begin + 0] = BLE_LL_ENC_RSP;
    memcpy(p->data + p->begin + 1, llcp->local_skd, 8);
    memcpy(p->data + p->begin + 9, setup->iv + 4, 4);

    net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                          0, NULL, &dst, p);
    buffer_refdec(p);

    p = net_layer_packet_alloc(&llcp->layer,
                               llcp->layer.context.prefix_size,
                               1);
    if (!p)
      break;
    in = net_scheduler_task_alloc(llcp->layer.scheduler);
    if (!in) {
      buffer_refdec(p);
      break;
    }

    p->data[p->begin + 0] = BLE_LL_START_ENC_REQ;

    net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                          0, NULL, &dst, p);
    buffer_refdec(p);

    break;
  }
  }

  net_task_destroy(task);
}

static void ble_llcp_query_handle(struct ble_llcp_s *llcp, struct net_task_s *task)
{
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };
  struct net_task_s *in;
  struct buffer_s *p;
  bool_t is_slave = llcp_is_slave(llcp);

  dprintk("%s %x\n", __FUNCTION__, task->query.opcode);

  switch (task->query.opcode) {
  case BLE_LLCP_PING:
    if (llcp_query_is_pending(llcp, BLE_LLCP_PING)) {
      net_task_query_respond_push(task, -EBUSY);
      return;
    }

    if (!llcp->layer.parent) {
      net_task_query_respond_push(task, -EIO);
      return;
    }

    p = net_layer_packet_alloc(&llcp->layer,
                               llcp->layer.context.prefix_size,
                               1);
    if (!p)
      break;
    in = net_scheduler_task_alloc(llcp->layer.scheduler);
    if (!in) {
      buffer_refdec(p);
      break;
    }

    p->data[p->begin + 0] = BLE_LL_PING_REQ;
    net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                          0, NULL, &dst, p);
    buffer_refdec(p);

    net_task_queue_pushback(&llcp->pending, task);
    return;

  case BLE_GAP_CONN_PARAMS_UPDATE:
    if (!is_slave) {
      net_task_query_respond_push(task, -ENOTSUP);
      return;
    } else {
      struct ble_gap_conn_params_update_s *up
        = ble_gap_conn_params_update_s_from_task(task);

      /* if (up->interval_min <= llcp->timing.current.interval */
      /*     && llcp->timing.current.interval <= up->interval_max */
      /*     && llcp->timing.current.latency == up->slave_latency) { */
      /*   net_task_query_respond_push(task, 0); */
      /*   return; */
      /* } */

      if (llcp->features
          && !(llcp->features & BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE)) {
        net_task_query_respond_push(task, -ENOTSUP);
        return;
      }

      if (llcp_query_is_pending(llcp, BLE_GAP_CONN_PARAMS_UPDATE)) {
        net_task_query_respond_push(task, -EBUSY);
        return;
      }

      if (!llcp->layer.parent) {
        net_task_query_respond_push(task, -EIO);
        return;
      }

      p = net_layer_packet_alloc(&llcp->layer,
                                 llcp->layer.context.prefix_size,
                                 24);
      if (!p) {
        net_task_query_respond_push(task, -ENOMEM);
        return;
      }
      in = net_scheduler_task_alloc(llcp->layer.scheduler);
      if (!in) {
        buffer_refdec(p);
        net_task_query_respond_push(task, -ENOMEM);
        return;
      }

      p->data[p->begin + 0] = BLE_LL_CONNECTION_PARAM_REQ;
      endian_le16_na_store(p->data + p->begin + 1, up->interval_min);
      endian_le16_na_store(p->data + p->begin + 3, up->interval_max);
      endian_le16_na_store(p->data + p->begin + 5, up->slave_latency);
      endian_le16_na_store(p->data + p->begin + 7, up->timeout);
      p->data[p->begin + 9] = 0;
      endian_le16_na_store(p->data + p->begin + 10, 0);
      memset(p->data + p->begin + 12, 0xff, 12);

      dprintk("%s sending %d\n", __FUNCTION__, p->data[p->begin]);

      net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                            0, NULL, &dst, p);
      buffer_refdec(p);

      net_task_queue_pushback(&llcp->pending, task);
    }
    return;
  }

  net_task_destroy(task);
}

static void ble_llcp_timeout_handle(struct ble_llcp_s *llcp, struct net_task_s *task)
{
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };
  struct net_task_s *in;
  struct buffer_s *p;

  dprintk("%s %p %p\n", __FUNCTION__, task, llcp->feature_req_later);

  if (task == llcp->feature_req_later) {
    llcp->feature_req_later = NULL;
    net_task_destroy(task);

    if (llcp->features_requested)
      return;

    llcp->features_requested = 1;

    p = net_layer_packet_alloc(&llcp->layer,
                               llcp->layer.context.prefix_size,
                               9);
    if (!p)
      return;
    in = net_scheduler_task_alloc(llcp->layer.scheduler);
    if (!in) {
      buffer_refdec(p);
      return;
    }

    p->data[p->begin + 0] = llcp_is_slave(llcp) ? BLE_LL_SLAVE_FEATURE_REQ : BLE_LL_FEATURE_REQ;
    endian_le64_na_store(p->data + p->begin + 1, SUPPORTED_FEATURES);

    dprintk("%s sending %d\n", __FUNCTION__, p->data[p->begin]);

    net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                          0, NULL, &dst, p);
    buffer_refdec(p);

    return;
  }

  net_task_destroy(task);
}

static
void ble_llcp_task_handle(struct net_layer_s *layer,
                           struct net_task_s *task)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_INBOUND:
    ble_llcp_packet_handle(llcp, task);
    return;

  case NET_TASK_RESPONSE:
    ble_llcp_response_handle(llcp, task);
    return;

  case NET_TASK_QUERY:
    ble_llcp_query_handle(llcp, task);
    return;

  case NET_TASK_TIMEOUT:
    ble_llcp_timeout_handle(llcp, task);
    return;

  default:
    break;
  }

  net_task_destroy(task);
}

static
error_t ble_llcp_bound(struct net_layer_s *layer,
                               void *addr,
                               struct net_layer_s *child)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);
  uint16_t proto = *(uint16_t *)addr;

  switch (proto) {
  case BLE_LLCP_CHILD_GAP:
    llcp->gap = child;
    return 0;
  }

  return -EINVAL;
}

static
void ble_llcp_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  if (child == llcp->gap)
    llcp->gap = NULL;
}

static void llcp_feature_req_later(struct ble_llcp_s *llcp)
{
  dprintk("%s\n", __FUNCTION__);

  if (llcp->feature_req_later) {
    net_scheduler_task_cancel(llcp->layer.scheduler,
                              llcp->feature_req_later);
    net_task_destroy(llcp->feature_req_later);
    llcp->feature_req_later = NULL;
  }

  dprintk("%s\n", __FUNCTION__);

  struct net_task_s *timeout = net_scheduler_task_alloc(llcp->layer.scheduler);
  if (timeout) {
    dev_timer_delay_t ticks;

    dev_timer_init_sec(&llcp->layer.scheduler->timer, &ticks, NULL, 1, 1);

    net_task_timeout_push(timeout, &llcp->layer,
                          net_scheduler_time_get(llcp->layer.scheduler) + ticks, 0);
    llcp->feature_req_later = timeout;
  }
}

static
bool_t ble_llcp_context_updated(struct net_layer_s *layer,
                                 const struct net_layer_context_s *parent_context)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  layer->context.addr = parent_context->addr;
  layer->context.prefix_size = parent_context->prefix_size + 4;
  layer->context.mtu = parent_context->mtu - 4;

  if (!llcp->features_requested)
    llcp_feature_req_later(llcp);

  return 1;
}

static
void ble_llcp_destroyed(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  dprintk("Llcp %p destroyed\n", llcp);

  mem_free(llcp);
}

static void ble_llcp_dandling(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  if (llcp->feature_req_later) {
    net_scheduler_task_cancel(llcp->layer.scheduler,
                              llcp->feature_req_later);
    net_task_destroy(llcp->feature_req_later);
    llcp->feature_req_later = NULL;
  }

  net_task_queue_reject_all(&llcp->pending);
}

static const struct net_layer_handler_s llcp_handler = {
  .destroyed = ble_llcp_destroyed,
  .task_handle = ble_llcp_task_handle,
  .bound = ble_llcp_bound,
  .unbound = ble_llcp_unbound,
  .context_updated = ble_llcp_context_updated,
  .dandling = ble_llcp_dandling,
};

error_t ble_llcp_create(struct net_scheduler_s *scheduler,
                        const void *_params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer)
{
#if defined(CONFIG_BLE_CRYPTO)
  const struct ble_llcp_params_s *params = _params;
#endif
  struct ble_llcp_s *llcp = mem_alloc(sizeof(*llcp), mem_scope_sys);

  if (!llcp)
    return -ENOMEM;

  memset(llcp, 0, sizeof(*llcp));

  error_t err = net_layer_init(&llcp->layer, &llcp_handler, scheduler,
                               delegate, delegate_vtable);
  if (err) {
    mem_free(llcp);
    return err;
  }

  llcp->gap = NULL;
#if defined(CONFIG_BLE_CRYPTO)
  llcp->rng = params->rng;
  llcp->peer = params->peer;
#endif

  net_task_queue_init(&llcp->pending);

  *layer = &llcp->layer;

  return err;
}
