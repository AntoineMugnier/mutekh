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

#define LOGK_MODULE_ID "llcp"

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

struct ble_llcp_s;

struct ble_llcp_handler_s;

#define SUPPORTED_FEATURES (0                                  \
                            | (_CONFIG_BLE_CRYPTO << BLE_LL_FEATURE_LE_ENCRYPTION) \
                            | (1 << BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE) \
                            | (1 << BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION) \
                            | (1 << BLE_LL_FEATURE_SLAVE_INITIATED_FEATURES_EXCHANGE) \
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
  uint8_t local_iv[4];

  struct net_layer_s *gap;
  struct net_task_s *feature_req_later;
  struct net_task_s *drop_later;
  net_task_queue_root_t pending;

#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s *rng;
  struct ble_peer_s *peer;
#endif

  struct ble_conn_timing_param_s current_timing;
  struct ble_gap_preferred_conn_params_s wanted_timing;

  uint8_t error;
  uint8_t reason_later;
  uint8_t features;

  bool_t version_sent : 1;
  bool_t features_exchanged : 1;
  bool_t encryption_started : 1;
};

STRUCT_COMPOSE(ble_llcp_s, layer);

static void llcp_connection_closed(struct ble_llcp_s *llcp, uint8_t err)
{
  if (llcp->drop_later)
    return;

  if (llcp->error)
    return;

  llcp->error = err;

  const struct ble_llcp_delegate_vtable_s *vtable
    = const_ble_llcp_delegate_vtable_s_from_base(llcp->layer.delegate_vtable);

  if (llcp->layer.delegate && vtable->connection_closed)
    vtable->connection_closed(llcp->layer.delegate, &llcp->layer, err);
}

static void llcp_termination_indicate(struct ble_llcp_s *llcp, uint8_t reason)
{
  struct net_task_s *in;
  struct buffer_s *p;
  struct net_addr_s dst = {
    .llid = BLE_LL_CONTROL,
    .fatal = 1,
  };

  p = net_layer_packet_alloc(&llcp->layer,
                             llcp->layer.context.prefix_size,
                             2);
  if (!p)
    return;

  in = net_scheduler_task_alloc(llcp->layer.scheduler);
  if (!in) {
    buffer_refdec(p);
    return;
  }

  p->data[p->begin + 0] = BLE_LL_TERMINATE_IND;
  p->data[p->begin + 1] = reason;
  net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                         0, NULL, &dst, p);
  buffer_refdec(p);
}

static void llcp_terminate_and_signal(struct ble_llcp_s *llcp, uint32_t ms, uint8_t reason)
{
  struct net_task_s *task;
  dev_timer_delay_t ticks;

  if (llcp->reason_later || llcp->drop_later)
    return;

  llcp_termination_indicate(llcp, reason);

  task = net_scheduler_task_alloc(llcp->layer.scheduler);
  if (!task)
    return;

  llcp->reason_later = reason;
  llcp->drop_later = task;

  dev_timer_init_sec(&llcp->layer.scheduler->timer, &ticks, NULL, ms, 1000);

  net_task_timeout_push(task, &llcp->layer,
                        net_scheduler_time_get(llcp->layer.scheduler) + ticks, 0);
}

#if defined(CONFIG_BLE_CRYPTO)
static error_t llcp_start_enc(struct ble_llcp_s *llcp)
{
  struct net_task_s *in;
  struct buffer_s *p;
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };
  error_t err;

  if (llcp->error)
    return -EIO;

  if (!llcp->peer->stk_present && !(llcp->peer->ltk_present && llcp->peer->identity_present))
    return -EPERM;

  if (llcp->encryption_started)
    return -EAGAIN;

  err = dev_rng_wait_read(llcp->rng, llcp->local_skd, 8);
  if (err)
    return err;

  err = dev_rng_wait_read(llcp->rng, llcp->local_iv, 4);
  if (err)
    return err;

  p = net_layer_packet_alloc(&llcp->layer,
                             llcp->layer.context.prefix_size,
                             23);
  if (!p)
    return -ENOMEM;

  in = net_scheduler_task_alloc(llcp->layer.scheduler);
  if (!in) {
    buffer_refdec(p);
    return -ENOMEM;
  }

  llcp->encryption_started = 1;

  p->data[p->begin + 0] = BLE_LL_ENC_REQ;
  if (llcp->peer->stk_present) {
    memset(p->data + p->begin + 1, 0, 10);
  } else {
    memcpy(p->data + p->begin + 1, llcp->peer->rand, 8);
    endian_le16_na_store(p->data + p->begin + 9, llcp->peer->ediv);
  }
  memcpy(p->data + p->begin + 11, llcp->local_skd, 8);
  memcpy(p->data + p->begin + 19, llcp->local_iv, 4);
  net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                         0, NULL, &dst, p);
  buffer_refdec(p);

  return 0;
}

static error_t llcp_restart_enc(struct ble_llcp_s *llcp)
{
  struct net_task_s *in;
  struct buffer_s *p;
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };

  if (llcp->error)
    return -EIO;

  if (!llcp->encryption_started)
    return -EAGAIN;

  p = net_layer_packet_alloc(&llcp->layer,
                             llcp->layer.context.prefix_size,
                             1);
  if (!p)
    return -ENOMEM;

  in = net_scheduler_task_alloc(llcp->layer.scheduler);
  if (!in) {
    buffer_refdec(p);
    return -ENOMEM;
  }

  llcp->encryption_started = 0;

  p->data[p->begin + 0] = BLE_LL_PAUSE_ENC_REQ;

  net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                         0, NULL, &dst, p);
  buffer_refdec(p);

  return 0;
}
#endif

static bool_t llcp_is_slave(struct ble_llcp_s *llcp)
{
#if defined(CONFIG_BLE_CENTRAL) && defined(CONFIG_BLE_PERIPHERAL)
  return !llcp->layer.context.addr.is_master;
#elif defined(CONFIG_BLE_PERIPHERAL)
  return 1;
#else
  return 0;
#endif
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
  bool_t fatal = 0;

  logk_trace("%s req %P...", is_slave ? "slave" : "master", p->data + p->begin, p->end - p->begin);

  switch (p->data[p->begin]) {
  case BLE_LL_CONNECTION_UPDATE_REQ: {
    if (!is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    llcp_query_pending_respond(llcp, BLE_GAP_CONN_PARAMS_UPDATE, 0);

    if (!llcp->layer.parent || !llcp->layer.parent->parent)
      break;

    struct ble_llcp_connection_parameters_update_s *up
      = mem_alloc(sizeof(*up), mem_scope_sys);

    if (!up) {
      reason = BLE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES;
      goto error;
    }

    up->task.destroy_func = memory_allocator_push;

    ble_data_conn_params_update_parse(&p->data[p->begin], &up->update);

    logk_trace("connection parameters update %d lat %d to %d at %d",
            up->update.timing.interval,
            up->update.timing.latency,
            up->update.timing.timeout,
            up->update.instant);

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

    logk_trace("channel map update");

    up->channel_map = endian_le32_na_load(args) | ((uint64_t)args[4] << 32);
    up->instant = endian_le16_na_load(args + 5);
    net_task_query_push(&up->task, llcp->layer.parent->parent, &llcp->layer,
                        BLE_LLCP_CHANNEL_MAP_UPDATE);
    break;
  }

  case BLE_LL_TERMINATE_IND:
    logk_trace("terminated %d", args[0]);

    llcp_connection_closed(llcp, args[0]);
    break;

  case BLE_LL_PING_REQ:
    logk_trace("ping req");

    p->data[p->begin] = BLE_LL_PING_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_VERSION_IND:
    if (llcp->version_sent)
      break;

    logk_trace("version ind");

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

    logk_trace("enc req");

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

    logk_trace("skdm:      %P", args + 10, 8);
    logk_trace("ivm:       %P", args + 18, 4);

    logk_trace("rand:      %P", args, 8);
    logk_trace("ediv:      %d", endian_le16_na_load(args + 8));

    logk_trace("skds:      %P", llcp->local_skd, 8);
    logk_trace("ivs:       %P", setup->iv + 4, 4);

    memcpy(skd, args + 10, 8);
    memcpy(skd + 8, llcp->local_skd, 8);

    memcpy(setup->iv, args + 18, 4);

    err = ble_peer_sk_get(llcp->peer, skd,
                          args, endian_le16_na_load(args + 8),
                          setup->sk);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      fatal = 1;
      goto error;
    }

    logk_trace("SK:        %P", setup->sk, 16);
    logk_trace("IV:        %P", setup->iv, 8);

#if defined(CONFIG_BLE_SECURITY_DB)
    setup->authenticated = llcp->peer->mitm_protection;
#else
    setup->authenticated = 0;
#endif

    net_task_query_push(&setup->task, llcp->layer.parent, &llcp->layer,
                        BLE_LLCP_ENCRYPTION_SETUP);
    break;
  }

  case BLE_LL_ENC_RSP: {
    uint8_t skd[16];
    uint8_t rand[8];
    uint16_t ediv;

    if (is_slave) {
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

    logk_trace("skdm:      %P", llcp->local_skd, 8);
    logk_trace("ivm:       %P", llcp->local_iv, 4);

    memcpy(skd, llcp->local_skd, 8);
    memcpy(skd + 8, args, 8);

    err = ble_peer_master_sk_get(llcp->peer, skd, &ediv, rand, setup->sk);
    if (err) {
      reason = BLE_PIN_OR_KEY_MISSING;
      fatal = 1;
      goto error;
    }

    logk_trace("rand:      %P", rand, 8);
    logk_trace("ediv:      %d", ediv);

    logk_trace("skds:      %P", args, 8);
    logk_trace("ivs:       %P", args + 8, 4);

    memcpy(setup->iv, llcp->local_iv, 4);
    memcpy(setup->iv + 4, args + 8, 4);

    logk_trace("SK:        %P", setup->sk, 16);
    logk_trace("IV:        %P", setup->iv, 8);

#if defined(CONFIG_BLE_SECURITY_DB)
    setup->authenticated = llcp->peer->mitm_protection;
#else
    setup->authenticated = 0;
#endif

    net_task_query_push(&setup->task, llcp->layer.parent, &llcp->layer,
                        BLE_LLCP_ENCRYPTION_SETUP);
    break;
  }

  case BLE_LL_START_ENC_REQ:
    if (is_slave) {
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

    if (!llcp->layer.parent)
      break;

    logk_trace("start enc req");

    p->data[p->begin + 0] = BLE_LL_START_ENC_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_START_ENC_RSP:
    if (!is_slave)
      break;

    if (!llcp->layer.parent)
      break;

    logk_trace("start enc rsp");

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

    logk_trace("pause enc req");

    p->data[p->begin + 0] = BLE_LL_PAUSE_ENC_RSP;
    p->end = p->begin + 1;

    goto respond;

  case BLE_LL_PAUSE_ENC_RSP:
    if (is_slave)
      break;

    logk_trace("pause enc rsp");

    p->data[p->begin + 0] = BLE_LL_PAUSE_ENC_RSP;
    p->end = p->begin + 1;

    if (llcp->layer.parent)
      net_task_packet_respond(task, llcp->layer.parent, 0, &dst);
    else
      net_task_destroy(task);

    llcp_start_enc(llcp);

    return;
#endif

  case BLE_LL_CONNECTION_PARAM_REQ:
    if (is_slave) {
      p->data[p->begin + 0] = BLE_LL_CONNECTION_PARAM_RSP;

      endian_le16_na_store(p->data + p->begin + 1, llcp->wanted_timing.interval_min);
      endian_le16_na_store(p->data + p->begin + 3, llcp->wanted_timing.interval_max);
      endian_le16_na_store(p->data + p->begin + 5, llcp->wanted_timing.latency);
      endian_le16_na_store(p->data + p->begin + 7, llcp->wanted_timing.timeout);

      p->data[p->begin + 9] = 0;
      endian_le16_na_store(p->data + p->begin + 10, 0);
      memset(p->data + p->begin + 12, 0xff, 12);
      p->end = p->begin + 24;

      goto respond;
    } else {
      // TODO: Handle slave-initiated connection update
      reason = BLE_COMMAND_DISALLOWED;
      goto error;
    }

  case BLE_LL_PING_RSP:
    logk_trace("ping rsp");
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

    logk_trace("features");

    llcp->features_exchanged = 1;
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
    llcp_query_pending_respond(llcp, BLE_GAP_CONN_PARAMS_UPDATE, 0);
    break;

  case BLE_LL_REJECT_IND_EXT:
    logk_trace("reject %d %d", args[0], args[1]);

    switch (args[0]) {
    case BLE_LL_CONNECTION_PARAM_REQ:
      llcp_query_pending_respond(llcp, BLE_GAP_CONN_PARAMS_UPDATE, -EINVAL);
      break;

    case BLE_LL_PING_REQ:
      llcp_query_pending_respond(llcp, BLE_LLCP_PING, -EINVAL);
      break;

    case BLE_LL_ENC_REQ:
      llcp_terminate_and_signal(llcp, 500, BLE_AUTHENTICATION_FAILURE);
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

  logk_trace("Done");

  net_task_destroy(task);
  return;

 error:
  logk_trace("rejected: %d", reason);

  p->data[p->begin + 1] = p->data[p->begin + 0];
  p->data[p->begin + 0] = BLE_LL_REJECT_IND_EXT;
  p->data[p->begin + 2] = reason;

 respond:
  if (llcp->layer.parent)
    net_task_packet_respond(task, llcp->layer.parent, 0, &dst);
  else
    net_task_destroy(task);

  if (fatal)
    llcp_terminate_and_signal(llcp, 500, reason);
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

      if (!is_slave)
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

  case BLE_LLCP_CONNECTION_PARAMETERS_UPDATE: {
    struct ble_llcp_connection_parameters_update_s *up =
      ble_llcp_connection_parameters_update_s_from_task(task);

    if (task->query.err)
      llcp_termination_indicate(llcp, BLE_UNACCEPTABLE_CONNECTION_PARAMETERS);

    /**
       This is not exactly true, as phy may not have committed the
       update yet, but this will be done in a limited time if it got
       accepted, so assume this is OK.
     */
    llcp->current_timing = up->update.timing;
    break;
  }

  case BLE_LLCP_CHANNEL_MAP_UPDATE:
    if (task->query.err)
      llcp_termination_indicate(llcp, BLE_INVALID_PARAMETERS);
    break;
  }

  net_task_destroy(task);
}

static void ble_llcp_query_handle(struct ble_llcp_s *llcp, struct net_task_s *task)
{
  struct net_addr_s dst = { .llid = BLE_LL_CONTROL };
  struct net_task_s *in;
  struct buffer_s *p;
  bool_t is_slave = llcp_is_slave(llcp);

  logk_trace("%s %x", __FUNCTION__, task->query.opcode);

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

      llcp->wanted_timing.interval_min = up->interval_min;
      llcp->wanted_timing.interval_max = up->interval_max;
      llcp->wanted_timing.latency = up->slave_latency;
      llcp->wanted_timing.timeout = up->timeout;

      if (up->interval_min <= llcp->current_timing.interval
          && llcp->current_timing.interval <= up->interval_max
          && llcp->current_timing.latency == up->slave_latency
          && llcp->current_timing.timeout == up->timeout) {
        net_task_query_respond_push(task, 0);
        return;
      }

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

      logk_trace("%s sending %d", __FUNCTION__, p->data[p->begin]);

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

  logk_trace("%s %p %p", __FUNCTION__, task, llcp->feature_req_later);

  if (task == llcp->feature_req_later) {
    llcp->feature_req_later = NULL;
    net_task_destroy(task);

    if (llcp->features_exchanged)
      return;

    llcp->features_exchanged = 1;

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

    logk_trace("%s sending %d", __FUNCTION__, p->data[p->begin]);

    net_task_outbound_push(in, llcp->layer.parent, &llcp->layer,
                          0, NULL, &dst, p);
    buffer_refdec(p);

    return;
  }

  if (task == llcp->drop_later) {
    llcp->drop_later = NULL;
    net_task_destroy(task);

    if (llcp->error)
      return;

    llcp_connection_closed(llcp, llcp->reason_later);

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
  logk_trace("%s", __FUNCTION__);

  if (llcp->feature_req_later) {
    net_scheduler_task_cancel(llcp->layer.scheduler,
                              llcp->feature_req_later);
    llcp->feature_req_later = NULL;
  }

  logk_trace("%s", __FUNCTION__);

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
void ble_llcp_context_changed(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  if (!llcp->features_exchanged)
    llcp_feature_req_later(llcp);
}

static
void ble_llcp_destroyed(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  logk_trace("%p destroyed", llcp);

  mem_free(llcp);
}

static void ble_llcp_dandling(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  if (llcp->feature_req_later) {
    net_scheduler_task_cancel(llcp->layer.scheduler,
                              llcp->feature_req_later);
    llcp->feature_req_later = NULL;
  }

  net_task_queue_reject_all(&llcp->pending);
}

static void _ble_llcp_connection_close(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  llcp_terminate_and_signal(llcp, 500, BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
}

static error_t _ble_llcp_encryption_enable(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  if (llcp_is_slave(llcp))
    return -ENOTSUP;

#if defined(CONFIG_BLE_CRYPTO)
  return llcp_start_enc(llcp);
#else
  return -ENOTSUP;
#endif
}

static error_t _ble_llcp_encryption_restart(struct net_layer_s *layer)
{
  struct ble_llcp_s *llcp = ble_llcp_s_from_layer(layer);

  if (llcp_is_slave(llcp))
    return -ENOTSUP;

#if defined(CONFIG_BLE_CRYPTO)
  return llcp_restart_enc(llcp);
#else
  return -ENOTSUP;
#endif
}

static const struct ble_llcp_handler_s llcp_handler = {
  .base.destroyed = ble_llcp_destroyed,
  .base.task_handle = ble_llcp_task_handle,
  .base.bound = ble_llcp_bound,
  .base.unbound = ble_llcp_unbound,
  .base.context_changed = ble_llcp_context_changed,
  .base.dandling = ble_llcp_dandling,
  .connection_close = _ble_llcp_connection_close,
  .encryption_enable = _ble_llcp_encryption_enable,
  .encryption_restart = _ble_llcp_encryption_restart,
};

error_t ble_llcp_create(struct net_scheduler_s *scheduler,
                        const void *_params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer)
{
  const struct ble_llcp_params_s *params = _params;
  struct ble_llcp_s *llcp = mem_alloc(sizeof(*llcp), mem_scope_sys);

  if (!llcp)
    return -ENOMEM;

  memset(llcp, 0, sizeof(*llcp));

  error_t err = net_layer_init(&llcp->layer, &llcp_handler.base, scheduler,
                               delegate, delegate_vtable);
  if (err) {
    mem_free(llcp);
    return err;
  }

  llcp->gap = NULL;
  llcp->current_timing = params->conn_timing;
#if defined(CONFIG_BLE_CRYPTO)
  llcp->rng = params->rng;
  llcp->peer = params->peer;
#endif

  llcp->wanted_timing = params->wanted_timing;

  net_task_queue_init(&llcp->pending);

  *layer = &llcp->layer;

  return err;
}
