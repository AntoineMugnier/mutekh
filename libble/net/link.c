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

#define LOGK_MODULE_ID "blnk"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/link.h>
#include <ble/net/llcp.h>
#include <ble/net/generic.h>
#include <ble/ccm_params.h>
#include <ble/protocol/data.h>

struct ble_link_s;
struct ble_sm_s;
struct ble_att_s;

struct ble_link_handler_s;

enum ble_link_state_e
{
  LINK_CLEAR,
#if defined(CONFIG_BLE_CRYPTO)
  LINK_ENC_STARTING0,
  LINK_ENC_STARTING1,
  LINK_ENC_STARTING2,
  LINK_ENC_RUNNING,
  LINK_ENC_STOPPING,
  LINK_ENC_PAUSED,
#endif
  LINK_FAILED,
};

enum ble_link_way_e
{
  LINK_IN,
  LINK_OUT,
  LINK_WAY_COUNT,
};

struct ble_link_s
{
  struct net_layer_s layer;

  struct net_layer_s *llcp;
  struct net_layer_s *l2cap;

  net_task_queue_root_t queue;

  int32_t outbound_accepted_count;

#if defined(CONFIG_BLE_CRYPTO)
  struct ble_ccm_state_s ccm_state[LINK_WAY_COUNT];

  struct net_task_s *ccm_task;
  struct buffer_s *tmp_packet;

  struct dev_crypto_rq_s crypto_rq;
  struct device_crypto_s crypto;
  struct dev_crypto_context_s ccm_ctx;

  bool_t authenticated;
#endif

  enum ble_link_state_e state;
};

STRUCT_COMPOSE(ble_link_s, layer);

#if defined(CONFIG_BLE_CRYPTO)
static void link_crypto_next(struct ble_link_s *link);
static
error_t link_crypto_setup(struct ble_link_s *link,
                        const uint8_t sk[static 16],
                        const uint8_t iv[static 8]);

static bool_t link_is_master(struct ble_link_s *link)
{
  return !!link->layer.context.addr.master;
}
#endif

static void link_state_set(struct ble_link_s *link, enum ble_link_state_e state)
{
  if (link->state == state)
    return;

  logk_trace("%s %d -> %d", __FUNCTION__, link->state, state);

#if defined(CONFIG_BLE_CRYPTO)
  bool_t crypto_state_changed = state == LINK_ENC_RUNNING || link->state == LINK_ENC_RUNNING;

  link->state = state;

  if (crypto_state_changed) {
    link->layer.context.addr.encrypted = state == LINK_ENC_RUNNING;
    link->layer.context.addr.authenticated = link->authenticated && link->layer.context.addr.encrypted;
    net_layer_context_changed(&link->layer);
  }
#else
  link->state = state;
#endif
}

static void link_task_forward(struct ble_link_s *link, struct net_task_s *task)
{
  struct net_layer_s *dest = NULL;

  logk_trace("%s %P...", __FUNCTION__,
          task->packet.buffer->data + task->packet.buffer->begin,
          task->packet.buffer->end - task->packet.buffer->begin);

  if (task->type == NET_TASK_INBOUND) {
    task->packet.dst_addr.llid = task->packet.buffer->data[task->packet.buffer->begin] & 0x03;
    task->packet.buffer->begin += 2;

    switch (task->packet.dst_addr.llid) {
    case BLE_LL_RESERVED:
      logk_error(" Bad packet type: error");
      link_state_set(link, LINK_FAILED);
      break;

    case BLE_LL_DATA_START:
    case BLE_LL_DATA_CONT:
      logk_trace(" to L2CAP");
      if (link->state != LINK_FAILED)
        dest = link->l2cap;
      break;

    case BLE_LL_CONTROL:
      logk_trace(" to LLCP");
      dest = link->llcp;

#if defined(CONFIG_BLE_CRYPTO)
      if (!link_is_master(link)) {
        uint8_t opcode = task->packet.buffer->data[task->packet.buffer->begin];

        switch (link->state) {
        default:
          break;

        case LINK_ENC_RUNNING:
          if (opcode == BLE_LL_PAUSE_ENC_REQ)
            link_state_set(link, LINK_ENC_STOPPING);
          break;

        case LINK_ENC_STOPPING:
          if (opcode == BLE_LL_PAUSE_ENC_RSP)
            link_state_set(link, LINK_ENC_STARTING1);
          break;
        }
      }
#endif
      break;
    }
  } else {
    logk_trace(" to parent");
    dest = link->layer.parent;

#if defined(CONFIG_BLE_CRYPTO)
    if (!link_is_master(link)) {
      uint8_t opcode = task->packet.buffer->data[task->packet.buffer->begin + 2];

      switch (link->state) {
      default:
        break;

      case LINK_ENC_STARTING1:
        if (opcode == BLE_LL_START_ENC_REQ)
          link_state_set(link, LINK_ENC_STARTING2);
        break;
      }
    }
#endif
  }

  if (dest)
    net_task_packet_forward(task, dest);
  else
    net_task_destroy(task);
}

#if defined(CONFIG_BLE_CRYPTO)
static KROUTINE_EXEC(link_crypto_done)
{
  struct ble_link_s *link = KROUTINE_CONTAINER(kr, *link, crypto_rq.base.kr);
  struct net_task_s *task;
  struct buffer_s *tmp;
  error_t err;
  reg_t irq_state;

  cpu_interrupt_savestate_disable(&irq_state);

  task = link->ccm_task;
  tmp = link->tmp_packet;

  assert(task && tmp);

  err = link->crypto_rq.error;

  if (err == -EAGAIN) {
    logk_error("EAGAIN in CCM ???");
    DEVICE_OP(&link->crypto, request, &link->crypto_rq);
    cpu_interrupt_restorestate(&irq_state);
    return;
  }

  assert(link->crypto_rq.out == tmp->data + tmp->begin);
  assert(link->crypto_rq.in == task->packet.buffer->data + task->packet.buffer->begin);

  /* Swap cleartext with ciphertext */
  link->tmp_packet = task->packet.buffer;
  task->packet.buffer = tmp;
  link->ccm_task = NULL;

  cpu_interrupt_restorestate(&irq_state);

  logk_trace("%s %p %P", __FUNCTION__, tmp->data + tmp->begin,
          tmp->data + tmp->begin,
          tmp->end - tmp->begin);

  assert(err != -EAGAIN);

  if (err) {
    logk_error("Crypto error, task type: %d, in payload %P, out payload %P, error %d",
           task->type,
           link->tmp_packet->data + link->tmp_packet->begin,
           link->tmp_packet->end - link->tmp_packet->begin,
           task->packet.buffer->data + task->packet.buffer->begin,
           task->packet.buffer->end - task->packet.buffer->begin,
           err);
    link_state_set(link, LINK_FAILED);

    net_task_destroy(task);
  } else {
    ((struct ble_ccm_state_s *)link->crypto_rq.iv_ctr)->packet_counter++;

    if (link_is_master(link)
        && task->type == NET_TASK_INBOUND
        && (task->packet.buffer->data[task->packet.buffer->begin] & 3) == BLE_LL_CONTROL) {
      uint8_t opcode = task->packet.buffer->data[task->packet.buffer->begin + 2];

      logk_debug("Master llcp opcode %d", opcode);

      switch (link->state) {
      default:
        break;

      case LINK_ENC_STARTING2:
        if (opcode == BLE_LL_START_ENC_RSP)
          link_state_set(link, LINK_ENC_RUNNING);
        break;

      case LINK_ENC_STOPPING:
        if (opcode == BLE_LL_PAUSE_ENC_RSP)
          link_state_set(link, LINK_ENC_PAUSED);
        break;
      }
    }

    link_task_forward(link, task);
  }

  link_crypto_next(link);

  net_layer_refdec(&link->layer);
}

static void link_task_crypt(struct ble_link_s *link, struct net_task_s *task)
{
  struct buffer_s *in, *out;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  assert(!link->ccm_task);
  assert(link->tmp_packet);

  logk_trace("%s", __FUNCTION__);

  //assert(buffer_refcount(task->packet.buffer) == 1);

  in = task->packet.buffer;
  in->data[in->begin] &= 3;
  out = link->tmp_packet;
  out->begin = link->layer.context.prefix_size;

  link->ccm_task = task;

  if (task->type == NET_TASK_INBOUND) {
    link->crypto_rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
    link->crypto_rq.iv_ctr = (void*)&link->ccm_state[LINK_IN];
    out->end = out->begin + (in->end - in->begin) - 4;
  } else {
    link->crypto_rq.op = DEV_CRYPTO_FINALIZE;
    link->crypto_rq.iv_ctr = (void*)&link->ccm_state[LINK_OUT];
    out->end = out->begin + (in->end - in->begin) + 4;

    if ((in->data[in->begin] & 3) == BLE_LL_CONTROL && !link_is_master(link)) {
      uint8_t opcode = in->data[in->begin + 2];

      switch (link->state) {
      default:
        break;

      case LINK_ENC_STARTING2:
        if (opcode == BLE_LL_START_ENC_RSP)
          link_state_set(link, LINK_ENC_RUNNING);
        break;
      }
    }
  }

  task->packet.dst_addr.encrypted = 1;
  task->packet.dst_addr.authenticated = link->authenticated;

  link->crypto_rq.ctx = &link->ccm_ctx;
  link->crypto_rq.ad_len = 0;
  link->crypto_rq.ad = 0;
  link->crypto_rq.out = out->data + out->begin;
  link->crypto_rq.in = in->data + in->begin;
  link->crypto_rq.len = in->end - in->begin;
  CPU_INTERRUPT_RESTORESTATE;

  logk_trace("%s %p -> %p %d", __FUNCTION__, link->crypto_rq.in, link->crypto_rq.out, link->crypto_rq.len);

  net_layer_refinc(&link->layer);
  DEVICE_OP(&link->crypto, request, &link->crypto_rq);
}

static bool_t is_enc_control(const struct buffer_s *buffer)
{
  uint8_t opcode = buffer->data[buffer->begin + 2];
  uint32_t enc_control = 0
    | (1 << BLE_LL_TERMINATE_IND)
    | (1 << BLE_LL_ENC_REQ)
    | (1 << BLE_LL_ENC_RSP)
    | (1 << BLE_LL_START_ENC_REQ)
    | (1 << BLE_LL_START_ENC_RSP)
    | (1 << BLE_LL_PAUSE_ENC_RSP)
    ;

  return (enc_control >> opcode) & 1;
}
#endif

static void link_crypto_next(struct ble_link_s *link)
{
  struct net_task_s *task;

  assert(cpu_is_interruptible());

#if defined(CONFIG_BLE_CRYPTO)
  if (link->ccm_task)
    return;
#endif

  logk_trace("%s", __FUNCTION__);

 again:
  task = NULL;

  if (!link->layer.parent) {
    logk_trace("%s No parent, flushing queue", __FUNCTION__);
    while ((task = net_task_queue_pop(&link->queue)))
      net_task_destroy(task);
    return;
  }

  switch (link->state) {
#if defined(CONFIG_BLE_CRYPTO)
  case LINK_ENC_STARTING0:
  case LINK_ENC_STARTING1:
  case LINK_ENC_STARTING2:
  case LINK_ENC_PAUSED:
  case LINK_ENC_STOPPING:
    logk_trace("%s in state %d, getting non-data or inbound packets", __FUNCTION__, link->state);
    GCT_FOREACH(net_task_queue, &link->queue, t,
                if (t->type == NET_TASK_INBOUND
                    || (t->source == link->llcp && is_enc_control(t->packet.buffer))) {
                  net_task_queue_nolock_remove(&link->queue, t);
                  task = t;
                  GCT_FOREACH_BREAK;
                });
    break;
#endif

  case LINK_FAILED:
    logk_trace("%s in state %d, getting non-data outbound packets", __FUNCTION__, link->state);
    GCT_FOREACH(net_task_queue, &link->queue, t,
                if (t->source == link->llcp && (t->packet.buffer->end - t->packet.buffer->begin) <= 2) {
                  net_task_queue_nolock_remove(&link->queue, t);
                  task = t;
                  GCT_FOREACH_BREAK;
                });
    break;

  default:
    task = net_task_queue_pop(&link->queue);
    break;
  }

  if (!task)
    return;

  logk_trace("%s state %d handling %P", __FUNCTION__,
          link->state,
          task->packet.buffer->data + task->packet.buffer->begin,
          task->packet.buffer->end - task->packet.buffer->begin);

  switch (link->state) {
#if defined(CONFIG_BLE_CRYPTO)
  case LINK_CLEAR:
    if (link_is_master(link)
        && (task->packet.buffer->data[task->packet.buffer->begin] & 3) == BLE_LL_CONTROL
        && task->packet.buffer->data[task->packet.buffer->begin + 2] == BLE_LL_ENC_REQ) {
      link_state_set(link, LINK_ENC_STARTING0);
    }
    goto forward;

  case LINK_ENC_STARTING1:
    if (link_is_master(link)
        && (task->packet.buffer->data[task->packet.buffer->begin] & 3) == BLE_LL_CONTROL
        && task->packet.buffer->data[task->packet.buffer->begin + 2] == BLE_LL_START_ENC_RSP) {
      link_state_set(link, LINK_ENC_STARTING2);
    } else {
      goto forward;
    }
    
  case LINK_ENC_RUNNING:
    if (link_is_master(link)
        && task->type == NET_TASK_OUTBOUND
        && (task->packet.buffer->data[task->packet.buffer->begin] & 3) == BLE_LL_CONTROL
        && task->packet.buffer->data[task->packet.buffer->begin + 2] == BLE_LL_PAUSE_ENC_REQ)
      link_state_set(link, LINK_ENC_STOPPING);
      goto crypt;

  case LINK_ENC_STOPPING:
  case LINK_ENC_STARTING2:
    if (task->type == NET_TASK_INBOUND
        && (task->packet.buffer->end - task->packet.buffer->begin) < 6) {
      link_state_set(link, LINK_FAILED);
      net_task_destroy(task);
      goto again;
    }

    crypt:
    link_task_crypt(link, task);
    break;
#endif

  forward:
  default:
    link_task_forward(link, task);
    goto again;
  }
}

static
void ble_link_task_handle(struct net_layer_s *layer,
                          struct net_task_s *task)
{
  struct ble_link_s *link = ble_link_s_from_layer(layer);

  /* logk_trace("%s %d", __FUNCTION__, task->type); */

  assert(cpu_is_interruptible());

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    if (!link->layer.parent)
      break;

    logk_trace("%s < %P", __FUNCTION__,
            task->packet.buffer->data + task->packet.buffer->begin,
            task->packet.buffer->end - task->packet.buffer->begin);

    assert(task->packet.buffer);

    if (task->packet.dst_addr.unreliable && link->outbound_accepted_count < 0) {
      logk("Dropping unreliable packet because of overflow");
      break;
    }

    link->outbound_accepted_count--;

    buffer_prepend(task->packet.buffer, (uint8_t[]){
      task->packet.dst_addr.llid,
      task->packet.buffer->end - task->packet.buffer->begin,
    }, 2);

    net_task_queue_pushback(&link->queue, task);
    link_crypto_next(link);
    return;

  case NET_TASK_INBOUND:
    logk_trace("%s > %P", __FUNCTION__,
            task->packet.buffer->data + task->packet.buffer->begin,
            task->packet.buffer->end - task->packet.buffer->begin);

    assert(task->packet.buffer);

    net_task_queue_pushback(&link->queue, task);
    link_crypto_next(link);
    return;

  case NET_TASK_QUERY:
    switch (task->query.opcode) {
#if defined(CONFIG_BLE_CRYPTO)
    case BLE_LLCP_ENCRYPTION_SETUP: {
      struct ble_llcp_encryption_setup_s *setup
        = ble_llcp_encryption_setup_s_from_task(task);
      net_task_query_respond_push(task, link_crypto_setup(link, setup->sk, setup->iv));
      link->authenticated = setup->authenticated;
      link_state_set(link, LINK_ENC_STARTING1);
      return;      
    }
#endif
    }
    break;


  case NET_TASK_NOTIFICATION:
    switch (task->notification.opcode) {
    case BLE_LINK_FLOW_UPDATE: {
      struct ble_link_flow_update_s *up
        = ble_link_flow_update_s_from_task(task);
      link->outbound_accepted_count = up->accepted_count;
      /* logk_trace("Now at %d accepted packets", link->outbound_accepted_count); */
      break;
    }
    }
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

#if defined(CONFIG_BLE_CRYPTO)
static
error_t link_crypto_setup(struct ble_link_s *link,
                        const uint8_t sk[static 16],
                        const uint8_t iv[static 8])
{
  error_t err;

  link->crypto_rq.op = DEV_CRYPTO_INIT;
  link->crypto_rq.ctx = &link->ccm_ctx;
  link->crypto_rq.iv_ctr = (void*)iv;
  link->ccm_ctx.mode = DEV_CRYPTO_MODE_BLE_CCM;
  link->ccm_ctx.iv_len = 8;
  link->ccm_ctx.key_data = (void*)sk;
  link->ccm_ctx.key_len = 16;

  link->ccm_state[LINK_IN].packet_counter = 0;
  link->ccm_state[LINK_IN].sent_by_master = !link_is_master(link);
  link->ccm_state[LINK_OUT].packet_counter = 0;
  link->ccm_state[LINK_OUT].sent_by_master = link_is_master(link);
  
  err = dev_crypto_wait_rq(&link->crypto, &link->crypto_rq);

  dev_crypto_rq_init(&link->crypto_rq, link_crypto_done);

  return err;
}
#endif

static
error_t ble_link_bound(struct net_layer_s *layer,
                               void *addr,
                               struct net_layer_s *child)
{
  struct ble_link_s *link = ble_link_s_from_layer(layer);
  uint16_t proto = *(uint16_t *)addr;

  switch (proto) {
  case BLE_LINK_CHILD_LLCP:
    link->llcp = child;
    return 0;

  case BLE_LINK_CHILD_L2CAP:
    link->l2cap = child;
    return 0;
  }

  return -EINVAL;
}

static
void ble_link_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct ble_link_s *link = ble_link_s_from_layer(layer);

  if (child == link->l2cap)
    link->l2cap = NULL;

  if (child == link->llcp)
    link->llcp = NULL;
}

static
void ble_link_child_context_adjust(const struct net_layer_s *layer,
                                   struct net_layer_context_s *cc)
{
  cc->prefix_size += 2;
  cc->mtu -= (_CONFIG_BLE_CRYPTO ? 6 : 2);
}

static
void ble_link_destroyed(struct net_layer_s *layer)
{
  struct ble_link_s *link = ble_link_s_from_layer(layer);

  logk_trace("Data link-layer %p destroyed", link);

#if defined(CONFIG_BLE_CRYPTO)
  device_put_accessor(&link->crypto.base);
  if (link->tmp_packet)
    buffer_destroy(link->tmp_packet);
#endif

  net_task_queue_destroy(&link->queue);

  mem_free(link);
}

static void ble_link_dandling(struct net_layer_s *layer)
{
  struct ble_link_s *link = ble_link_s_from_layer(layer);

  net_task_queue_reject_all(&link->queue);
}

static const struct net_layer_handler_s link_handler = {
  .destroyed = ble_link_destroyed,
  .task_handle = ble_link_task_handle,
  .bound = ble_link_bound,
  .unbound = ble_link_unbound,
  .child_context_adjust = ble_link_child_context_adjust,
  .dandling = ble_link_dandling,
};

error_t ble_link_create(struct net_scheduler_s *scheduler,
                      const void *_params,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                      struct net_layer_s **layer)
{
  __unused__
  const struct ble_link_param_s *params = _params;
  struct ble_link_s *link;
  size_t alloc_size = sizeof(*link);

#if defined(CONFIG_BLE_CRYPTO)
  struct dev_crypto_info_s crypto_info;

  DEVICE_OP(params->crypto, info, &crypto_info);
  alloc_size += crypto_info.state_size;
#endif

  link = mem_alloc(alloc_size, mem_scope_sys);
  if (!link)
    return -ENOMEM;

  error_t err = net_layer_init(&link->layer, &link_handler, scheduler,
                               delegate, delegate_vtable);
  if (err) {
    mem_free(link);
    return err;
  }

  link->llcp = NULL;
  link->l2cap = NULL;

  net_task_queue_init(&link->queue);

#if defined(CONFIG_BLE_CRYPTO)
  err = device_copy_accessor(&link->crypto.base, &params->crypto->base);
  if (err) {
    net_layer_refdec(&link->layer);
    return err;
  }

  link->ccm_task = NULL;
  memset(&link->ccm_ctx, 0, sizeof(link->ccm_ctx));
  memset(&link->crypto_rq, 0, sizeof(link->crypto_rq));
  link->ccm_ctx.state_data = link + 1;

  link->tmp_packet = net_layer_packet_alloc(&link->layer, link->layer.context.prefix_size, 0);
  assert(link->tmp_packet);
#endif

  link->state = LINK_CLEAR;

  *layer = &link->layer;

  return 0;
}
