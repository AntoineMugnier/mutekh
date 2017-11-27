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

#define LOGK_MODULE_ID "bsm_"

#include <string.h>

#include <hexo/bit.h>
#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <device/class/crypto.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/sm.h>
#include <ble/net/generic.h>
#include <ble/crypto.h>
#include <ble/protocol/sm.h>
#include <ble/protocol/l2cap.h>

#include <ble/peer.h>
#include <device/class/crypto.h>

#include "sm.h"
#include "sm_impl.o.h"

#undef SM_HAS_SECURE_PAIRING

#if !defined(CONFIG_BLE_PERIPHERAL) && !defined(CONFIG_BLE_CENTRAL)
# error How good is Security manager without either Central nor Peripheral roles ?
#endif

static ALWAYS_INLINE bool_t sm_is_slave(struct ble_sm_s *sm)
{
#if defined(CONFIG_BLE_CENTRAL) && defined(CONFIG_BLE_PERIPHERAL)
  return !sm->layer.context.addr.master;
#elif  defined(CONFIG_BLE_CENTRAL)
  return 0;
#else
  return 1;
#endif
}

static
void sm_state_advance(struct ble_sm_s *sm);

static
void ble_sm_destroyed(struct net_layer_s *layer)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  device_put_accessor(&sm->aes.base);
  kroutine_seq_cleanup(&sm->seq);

  mem_free(sm);
}

static void sm_fail(struct ble_sm_s *sm, uint8_t err)
{
  sm->state = SM_FAIL;

  logk_trace("Fail for reason %x", err);

  // TODO: Send failure packet
}

static KROUTINE_EXEC(sm_aes_done)
{
  struct ble_sm_s *sm = KROUTINE_CONTAINER(kr, *sm, aes_rq.base.kr);
  
  switch (sm->state) {
  case SM_VM_WAIT_RAND:
    logk_trace("Random out: %P", sm->aes_io, sm->aes_rq.len);
    break;

  case SM_VM_WAIT_CONF:
    logk_trace("Confirm out: %P", sm->aes_io + 16, 16);
    break;

  case SM_VM_WAIT_STK:
    logk_trace("STK: %P", sm->aes_io + 16, 16);
    ble_peer_phase2_done(sm->peer, sm->aes_io + 16,
                         !!(sm->preq[3] & sm->pres[3] & BLE_SM_REQ_MITM));
    break;

  default:
    logk_trace("Crypto done, bad state: %d", sm->state);
    return;
  }

  if (sm->aes_rq.err)
    return sm_fail(sm, BLE_SM_REASON_UNSPECIFIED_REASON);

  sm->state = SM_VM_IDLE;

  sm_state_advance(sm);
}

static void sm_rng_read(struct ble_sm_s *sm,
                        uint8_t *dest,
                        size_t size)
{
  sm->aes_rq.op = DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE;
  sm->aes_rq.in = NULL;
  sm->aes_rq.out = dest;
  sm->aes_rq.len = size;
  sm->aes_ctx.mode = DEV_CRYPTO_MODE_RANDOM;
  sm->aes_ctx.state_data = sm->rng->state_data;

  logk_trace("RNG Read, %p %d", dest, size);

  sm->state = SM_VM_WAIT_RAND;

  DEVICE_OP(&sm->aes, request, &sm->aes_rq);
}

static void sm_s1(struct ble_sm_s *sm,
                  const uint8_t *k,
                  const uint8_t *r1,
                  const uint8_t *r2)
{
  sm->aes_rq.op = 0;
  sm->aes_rq.in = sm->aes_io;
  sm->aes_rq.out = sm->aes_io + 16;
  sm->aes_rq.len = 16;
  sm->aes_ctx.key_data = (void*)k;
  sm->aes_ctx.key_len = 16;
  sm->aes_ctx.mode = DEV_CRYPTO_MODE_ECB;
  sm->aes_ctx.cache_ptr = NULL;
  
  memcpy(sm->aes_io, r1 + 8, 8);
  memcpy(sm->aes_io + 8, r2 + 8, 8);

  logk_trace("STK calculation from %P", sm->aes_io, 16);

  sm->state = SM_VM_WAIT_STK;

  DEVICE_OP(&sm->aes, request, &sm->aes_rq);
}

static void sm_c1(struct ble_sm_s *sm,
                  uint8_t *conf,
                  const uint8_t *rand)
{
  const struct ble_addr_s *ia;
  const struct ble_addr_s *ra;

  if (sm_is_slave(sm)) {
    ia = &sm->peer->lookup_addr;
    ra = &sm->local_addr;
  } else {
    ia = &sm->local_addr;
    ra = &sm->peer->lookup_addr;
  }

  memrevcpy(sm->aes_io, sm->pres, 7);
  memrevcpy(sm->aes_io + 7, sm->preq, 7);
  sm->aes_io[14] = ra->type;
  sm->aes_io[15] = ia->type;
  sm->aes_io[16] = 0;
  sm->aes_io[17] = 0;
  sm->aes_io[18] = 0;
  sm->aes_io[19] = 0;
  memrevcpy(sm->aes_io + 20, ia->addr, 6);
  memrevcpy(sm->aes_io + 26, ra->addr, 6);

  logk_trace("Conf in %P", sm->aes_io, 32);

  memcpy(conf, rand, 16);

  logk_trace("Conf iv %P", conf, 16);

  sm->aes_ctx.key_data = sm->tk;
  sm->aes_ctx.key_len = 16;
  sm->aes_ctx.mode = DEV_CRYPTO_MODE_CBC;
  sm->aes_ctx.iv_len = 16;
  sm->aes_ctx.auth_len = 0;
  sm->aes_rq.op = DEV_CRYPTO_FINALIZE;
  sm->aes_rq.in = sm->aes_io;
  sm->aes_rq.out = sm->aes_io;
  sm->aes_rq.ad = NULL;
  sm->aes_rq.ad_len = 0;
  sm->aes_rq.len = 32;
  sm->aes_rq.iv_ctr = conf;
  sm->aes_rq.auth = NULL;

  sm->state = SM_VM_WAIT_CONF;

  DEVICE_OP(&sm->aes, request, &sm->aes_rq);
}

static void sm_tx_done(struct net_task_s *task)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(task->source);

  mem_free(task);

  if (sm->state == SM_VM_WAIT_TX) {
    sm->state = SM_VM_IDLE;

    logk_trace("TX done");

    sm_state_advance(sm);
  }
}

static void sm_tx(struct ble_sm_s *sm, struct buffer_s *packet)
{
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
  };
  struct net_task_s *task;

  logk_trace("TX < %P",
          packet->data + packet->begin,
          packet->end - packet->begin);

  task = mem_alloc(sizeof(*task), mem_scope_sys);
  if (task) {
    task->destroy_func = sm->state == SM_VM_WAIT_TX ? (void*)sm_tx_done : (void*)mem_free;
    task->type = NET_TASK_INVALID;
    task->source = NULL;
    task->target = NULL;
  }

  if (task)
    net_task_outbound_push(task,
                           sm->layer.parent, &sm->layer,
                           0, NULL, &dst, packet);
  else if (sm->state == SM_VM_WAIT_TX)
    sm_fail(sm, BLE_SM_REASON_UNSPECIFIED_REASON);

  buffer_refdec(packet);
}

static void sm_pairing_start(struct ble_sm_s *sm)
{
  bc_set_reg(&sm->vm, SM_IMPL_BCGLOBAL_PV, (uintptr_t)sm);

  logk_trace("Req:           %P", sm->preq, 7);
  logk_trace("Res:           %P", sm->pres, 7);

#if defined(SM_HAS_SECURE_PAIRING)
  if (sm->preq[3] & sm->pres[3] & BLE_SM_REQ_SC) {
    // TODO: implement secure pairing

    return;
  } else {
#endif
    logk_trace("Starting legacy pairing");

    bc_set_pc(&sm->vm, sm_is_slave(sm)
              ? &ble_sm_slave_legacy
              : &ble_sm_master_legacy);
#if defined(SM_HAS_SECURE_PAIRING)
  }
#endif

  sm->state = SM_VM_IDLE;

  sm_state_advance(sm);
}

static void sm_command_handle(struct ble_sm_s *sm, struct net_task_s *task)
{
  const uint8_t *data = task->packet.buffer->data + task->packet.buffer->begin;
  const size_t size = task->packet.buffer->end - task->packet.buffer->begin;
  const struct ble_sm_delegate_vtable_s *vtable
    = const_ble_sm_delegate_vtable_s_from_base(sm->layer.delegate_vtable);

  logk_trace("RX > %P", data, size);

  switch (data[0]) {
  case BLE_SM_PAIRING_REQUEST:
    if (!sm_is_slave(sm))
      goto unhandled;

    if (size != 7)
      goto invalid;

    if (sm->state != SM_IDLE) {
      logk_trace("Bad state for pairing req");
      goto invalid;
    }

    memcpy(sm->preq, data, 7);

    sm->state = SM_WAIT_USER;

    logk_trace("Pairing requested from master, asking user for confirmation");
    
    vtable->pairing_requested(sm->layer.delegate, &sm->layer,
                              !!(sm->preq[3] & BLE_SM_REQ_BONDING));
    return;

  case BLE_SM_PAIRING_RESPONSE:
    if (sm_is_slave(sm))
      goto unhandled;

    if (size != 7)
      goto invalid;

    if (sm->state != SM_WAIT_RESPONSE) {
      logk_trace("Bad state for pairing response");
      goto invalid;
    }
    
    memcpy(sm->pres, data, 7);

    logk_trace("Pairing response from slave");

    sm_pairing_start(sm);
    return;

  case BLE_SM_SECURITY_REQUEST:
    if (sm_is_slave(sm))
      goto unhandled;

    if (size != 2)
      goto invalid;

    logk_trace("Security request from slave");

    sm->state = SM_WAIT_USER;

    vtable->pairing_requested(sm->layer.delegate, &sm->layer,
                              !!(data[1] & BLE_SM_REQ_BONDING));
    return;

  case BLE_SM_PAIRING_FAILED:
    logk_error("Pairing failed beause of peer (%d)", data[1]);

    if (size != 2)
      goto invalid;

    vtable->pairing_failed(sm->layer.delegate, &sm->layer, data[1]);
    sm->state = SM_FAIL;
    return;

  default:
    if (sm->state == SM_VM_WAIT_RX) {
      logk_trace("Packet received for VM");

      sm->state = SM_VM_IDLE;

      if (sm->pkt)
        buffer_refdec(sm->pkt);
      sm->pkt = buffer_refinc(task->packet.buffer);

      bc_set_reg(&sm->vm, SM_IMPL_BCGLOBAL_PKT, (uintptr_t)sm->pkt->data + sm->pkt->begin);

      sm_state_advance(sm);
      return;
    }

    logk_trace("Other packet, unhandled");

  unhandled:
  invalid:
    sm_fail(sm, BLE_SM_REASON_INVALID_PARAMETERS);
    return;
  }
}

static
void ble_sm_task_handle(struct net_layer_s *layer,
                        struct net_task_s *task)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_INBOUND:
    sm_command_handle(sm, task);
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static void ble_sm_context_changed(struct net_layer_s *layer)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  logk_trace("context changed, now %s, state %d",
          layer->context.addr.encrypted ? "encrypted" : "clear",
          sm->state);

  if (layer->parent
      && layer->context.addr.encrypted
      && sm->state == SM_VM_WAIT_ENCRYPTION) {
    sm->state = SM_VM_IDLE;

    sm_state_advance(sm);
  }
}

static
void sm_pairing_request(struct net_layer_s *layer,
                        bool_t mitm_protection,
                        bool_t bonding)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  if (sm->state != SM_IDLE || !sm->layer.parent)
    return;

  if (sm_is_slave(sm)) {
    struct buffer_s *rsp = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 2);

    rsp->data[rsp->begin] = BLE_SM_SECURITY_REQUEST;
    rsp->data[rsp->begin + 1] = 0
#if defined(CONFIG_BLE_SECURITY_DB)
      | (bonding ? BLE_SM_REQ_BONDING : 0)
#endif
      | (mitm_protection ? BLE_SM_REQ_MITM : 0);

    sm->state = SM_WAIT_REQUEST;

    sm_tx(sm, rsp);
  } else {
    const struct ble_sm_delegate_vtable_s *vtable
      = const_ble_sm_delegate_vtable_s_from_base(sm->layer.delegate_vtable);

    sm->state = SM_WAIT_USER;

    vtable->pairing_requested(sm->layer.delegate, &sm->layer, bonding);
  }
}

static
void sm_pairing_accept(struct net_layer_s *layer,
                       bool_t mitm_protection,
                       uint32_t pin,
                       const void *oob_data)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  logk("%s()", __func__);

  if (!sm->layer.parent)
    return;

  if (sm->state != SM_WAIT_USER)
    return;

  struct buffer_s *pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 7);

  if (oob_data) {
    memcpy(sm->tk, oob_data, 16);
  } else {
    endian_le32_na_store(sm->tk, pin);
    memset(sm->tk + 4, 0, 12);
  }

  if (sm_is_slave(sm)) {
    sm->pres[0] = BLE_SM_PAIRING_RESPONSE;
    sm->pres[1] = sm->io_cap;
    sm->pres[2] = !!oob_data;
    sm->pres[3] = 0
#if defined(CONFIG_BLE_SECURITY_DB)
      | BLE_SM_REQ_BONDING
#endif
      | (mitm_protection ? BLE_SM_REQ_MITM : 0)
#if defined(SM_HAS_SECURE_PAIRING)
      | BLE_SM_REQ_SC
#endif
      ;
    sm->pres[4] = 16;
    sm->pres[5] = sm->preq[5] & (BLE_SM_ENC_KEY | BLE_SM_ID_KEY);
    sm->pres[6] = sm->preq[6] & (BLE_SM_ENC_KEY | BLE_SM_ID_KEY);

    memcpy(pkt->data + pkt->begin, sm->pres, 7);

    sm_tx(sm, pkt);

    logk_trace("Pairing RSP sent");

    sm_pairing_start(sm);
  } else {
    error_t err = ble_peer_reset(sm->peer);
    if (err)
      return;

    sm->preq[0] = BLE_SM_PAIRING_REQUEST;
    sm->preq[1] = sm->io_cap;
    sm->preq[2] = !!oob_data;
    sm->preq[3] = 0
#if defined(CONFIG_BLE_SECURITY_DB)
      | BLE_SM_REQ_BONDING
#endif
      | (mitm_protection ? BLE_SM_REQ_MITM : 0)
#if defined(SM_HAS_SECURE_PAIRING)
      | BLE_SM_REQ_SC
#endif
      ;
    sm->preq[4] = 16;
    sm->preq[5] = BLE_SM_ENC_KEY | BLE_SM_ID_KEY;
    sm->preq[6] = BLE_SM_ENC_KEY | BLE_SM_ID_KEY;

    memcpy(pkt->data + pkt->begin, sm->preq, 7);

    sm->state = SM_WAIT_RESPONSE;

    sm_tx(sm, pkt);

    logk("Pairing REQ sent");
  }
}

static
void sm_pairing_abort(struct net_layer_s *layer, enum sm_reason reason)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  if (sm->state == SM_IDLE || !sm->layer.parent)
    return;

  logk_trace("Pairing aborted by user");

  sm_fail(sm, BLE_SM_REASON_UNSPECIFIED_REASON);
}

static
void sm_pairing_success(struct ble_sm_s *sm)
{
  const struct ble_sm_delegate_vtable_s *vtable
    = const_ble_sm_delegate_vtable_s_from_base(sm->layer.delegate_vtable);

  logk_trace("Notifying user of pairing success");

  vtable->pairing_success(sm->layer.delegate, &sm->layer);
}

static
void sm_bonding_success(struct ble_sm_s *sm)
{
  const struct ble_sm_delegate_vtable_s *vtable
    = const_ble_sm_delegate_vtable_s_from_base(sm->layer.delegate_vtable);

  logk_trace("Notifying user of pairing failure");

  vtable->bonding_success(sm->layer.delegate, &sm->layer);
}

static
void sm_state_advance(struct ble_sm_s *sm)
{
  bc_opcode_t op;

  logk_trace("VM");

  assert(sm->state == SM_VM_IDLE);

 again:
  op = bc_run(&sm->vm);

  assert(op & 0x8000);

  switch (bit_get_range(op, 12, 14)) {
  case 0: // Packet management
    switch (bit_get_range(op, 8, 11)) {
    case 0: // Packet alloc
      logk_trace("Packet alloc");
      if (!sm->pkt) {
        sm->pkt = net_layer_packet_alloc(&sm->layer, 0, 0);
        if (!sm->pkt) {
          sm_fail(sm, BLE_SM_REASON_UNSPECIFIED_REASON);
          return;
        }
      }
      sm->pkt->end = sm->pkt->begin = sm->layer.context.prefix_size;

      logk_trace("Packet alloc: %p, data at %p", sm->pkt, sm->pkt->data + sm->pkt->begin);

      bc_set_reg(&sm->vm, SM_IMPL_BCGLOBAL_PKT, (uintptr_t)sm->pkt->data + sm->pkt->end);
      goto again;

    case 1: // Packet wait
      logk_trace("Packet wait");
      if (sm->pkt)
        buffer_refdec(sm->pkt);
      sm->pkt = NULL;
      bc_set_reg(&sm->vm, SM_IMPL_BCGLOBAL_PKT, 0);
      sm->state = SM_VM_WAIT_RX;
      return;

    case 2: { // Packet left
      struct buffer_s *pkt = sm->pkt;
      uint8_t *point = (uint8_t *)bc_get_reg(&sm->vm, SM_IMPL_BCGLOBAL_PKT);

      assert(pkt);
      assert(point > pkt->data);

      size_t left = pkt->end - (point - pkt->data);
      
      logk_trace("Packet left: %d", left);

      bc_set_reg(&sm->vm, bit_get_range(op, 0, 3), left);
      goto again;
    }

    case 3: { // Packet send
      struct buffer_s *pkt = sm->pkt;
      uint8_t *point = (uint8_t *)bc_get_reg(&sm->vm, SM_IMPL_BCGLOBAL_PKT);

      assert(pkt);
      assert(point > pkt->data);

      sm->pkt->end = point - pkt->data;

      logk_trace("Packet send: %d bytes", sm->pkt->end - sm->pkt->begin);

      sm->pkt = NULL;
      sm->state = SM_VM_WAIT_TX;

      sm_tx(sm, pkt);
      return;
    }
    }
    break;

  case 1: // DB handling
    logk_trace("DB %d", bit_get_range(op, 8, 11));

    switch (bit_get_range(op, 8, 11)) {
    case 0: // LTK get
    case 1: // IRK get
    case 2: // Peer ID get
    case 3: // EDIV get
    case 4: // Peer CSRK get
    case 5: // Peer LTK set
    case 6: // Peer ID set
    case 7: // Peer EDIV set
    case 8: // Peer IRK set
    case 9: // Peer CSRK set
      ;
    }
    break;

  case 2: // Crypto
    switch (bit_get_range(op, 8, 11)) {
    case 0: { // RNG Read
      size_t size = 1 + bit_get_range(op, 4, 7);
      void *buffer = (void*)bc_get_reg(&sm->vm, bit_get_range(op, 0, 3));

      logk_trace("RNG Read");

      sm_rng_read(sm, buffer, size);
      return;
    }

    case 1: { // Confirm calc
      void *rand = (void*)bc_get_reg(&sm->vm, bit_get_range(op, 0, 3));
      void *conf = (void*)bc_get_reg(&sm->vm, bit_get_range(op, 4, 7));

      logk_trace("Confirm calc");

      sm_c1(sm, conf, rand);
      return;
    }

    case 2: // STK compute
      logk_trace("STK");

      sm_s1(sm, sm->tk, sm->srand, sm->mrand);
      return;
    }
    break;

  case 3: // State
    switch (bit_get_range(op, 8, 11)) {
    case 0: // Pairing success
      logk_trace("Pairing success");
      sm_pairing_success(sm);
      goto again;

    case 1: // Encryption wait
      logk_trace("Encryption wait");
      sm->state = SM_VM_WAIT_ENCRYPTION;
      return;

    case 2: // Bonding success
      logk_trace("Bonding success");
      sm_bonding_success(sm);
      goto again;

    case 3: { // Failure
      uint8_t reason = bit_get_range(op, 0, 7);
      logk_trace("Pairing failure 0x%x", reason);
      sm_fail(sm, reason);
      return;
    }
    }
    break;
  }

  sm_fail(sm, BLE_SM_REASON_UNSPECIFIED_REASON);
}

static const struct ble_sm_handler_s sm_handler = {
  .base.destroyed = ble_sm_destroyed,
  .base.task_handle = ble_sm_task_handle,
  .base.context_changed = ble_sm_context_changed,
  .pairing_request = sm_pairing_request,
  .pairing_accept = sm_pairing_accept,
  .pairing_abort = sm_pairing_abort,
};

static
error_t ble_sm_init(
  struct ble_sm_s *sm,
  struct net_scheduler_s *scheduler,
  const struct ble_sm_param_s *params,
  void *delegate,
  const struct ble_sm_delegate_vtable_s *delegate_vtable)
{
  error_t err;

  memset(sm, 0, sizeof(*sm));

  sm->rng = params->rng;

  err = device_copy_accessor(&sm->aes.base, &params->crypto->base);
  if (err)
    goto err_out;

  kroutine_seq_init(&sm->seq);

  err = net_layer_init_seq(&sm->layer, &sm_handler.base, scheduler,
                           delegate, &delegate_vtable->base,
                           &sm->seq);

  if (err)
    goto err_put_aes;

  sm->peer = params->peer;
  sm->io_cap = BLE_SM_IO_CAP_NO_INPUT_NO_OUTPUT;
  sm->local_addr = params->local_addr;
  bc_init(&sm->vm, &sm_impl_bytecode);

  sm->aes_rq.ctx = &sm->aes_ctx;
  kroutine_init_deferred_seq(&sm->aes_rq.base.kr, sm_aes_done, &sm->seq);

  return 0;

 err_put_aes:
  device_put_accessor(&sm->aes.base);
 err_out:
  return err;
}

error_t ble_sm_create(struct net_scheduler_s *scheduler,
                      const void *params_,
                      void *delegate,
                      const struct net_layer_delegate_vtable_s *delegate_vtable_,
                      struct net_layer_s **layer)
{
  struct ble_sm_s *sm = mem_alloc(sizeof(*sm), mem_scope_sys);
  const struct ble_sm_param_s *params = params_;
  const struct ble_sm_delegate_vtable_s *delegate_vtable
    = const_ble_sm_delegate_vtable_s_from_base(delegate_vtable_);

  if (!sm)
    return -ENOMEM;

  error_t err = ble_sm_init(sm, scheduler, params, delegate, delegate_vtable);
  if (err)
    mem_free(sm);
  else
    *layer = &sm->layer;

  return err;
}
