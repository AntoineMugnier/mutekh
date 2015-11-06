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

#include <string.h>

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <device/class/crypto.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/layer.h>
#include <ble/net/sm.h>
#include <ble/net/generic.h>
#include <ble/crypto.h>
#include <ble/protocol/sm.h>
#include <ble/protocol/l2cap.h>

#define EXPECT_ENCRYPTION_INFORMATION 1
#define EXPECT_MASTER_IDENTIFICATION 2
#define EXPECT_IDENTITY_INFORMATION 4
#define EXPECT_IDENTITY_ADDRESS_INFORMATION 8
#define EXPECT_CSRK 16

#define dprintk(...) do{}while(0)
//#define dprintk printk

#include <ble/peer.h>
#include <device/class/crypto.h>

struct ble_sm_handler_s;
struct dev_rng_s;

enum ble_sm_state_e
{
  BLE_SM_IDLE,
  BLE_SM_REQUESTED,
  BLE_SM_REQUEST_DONE,
  BLE_SM_REQUEST_ANSWERED,
  BLE_SM_MCONF_SENT,
  BLE_SM_SCONF_SENT,
  BLE_SM_MRAND_SENT,
  BLE_SM_STK_DONE,
  BLE_SM_DISTRIBUTION_DONE,
  BLE_SM_RECEPTION_DONE,
};

/**
 BLE Security manager layer.

 Handles device pairing and bonding.
 */
struct ble_sm_s
{
  struct net_layer_s layer;

  struct dev_rng_s *rng;
  struct device_crypto_s aes_dev;

  const struct ble_sm_handler_s *handler;
  struct ble_peer_s *peer;

  enum ble_sm_state_e pairing_state;
  bool_t security_requested;

  struct ble_addr_s local_addr;

  uint8_t to_distribute;
  uint8_t to_expect;

  uint8_t io_cap;
  bool_t mitm_protection;

  __attribute__((align(16)))
  struct {
    uint8_t tk[16];

    uint8_t srand[16];
    uint8_t mrand[16];

    uint8_t mconf[16];
    uint8_t sconf[16];

    uint8_t preq[7];
    uint8_t pres[7];
  };
};

STRUCT_COMPOSE(ble_sm_s, layer);

static ALWAYS_INLINE bool_t sm_is_slave(struct ble_sm_s *sm)
{
#if defined(CONFIG_BLE_CENTRAL) && defined(CONFIG_BLE_PERIPHERAL)
  return sm->layer.parent
    && sm->layer.parent->parent
    && sm->layer.parent->parent->handler.type == BLE_NET_LAYER_SLAVE;
#elif  defined(CONFIG_BLE_CENTRAL)
  return 0;
#else
  return 1;
#endif
}

static
void ble_sm_destroyed(struct net_layer_s *layer)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  device_put_accessor(&sm->aes_dev);

  mem_free(sm);
}

static uint8_t sm_sconf_setup(struct ble_sm_s *sm, uint8_t *sconfirm)
{
  error_t err;

  err = ble_peer_reset(sm->peer);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  err = dev_rng_wait_read(sm->rng, sm->srand, sizeof(sm->srand));
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("Starting pairing, new device id: %lld\n", sm->peer->id);
  dprintk("Generating sconf:\n");

  dprintk("TK:         %P\n", sm->tk, 16);
  dprintk("srand:      %P\n", sm->srand, 16);
  dprintk("preq:       %P\n", sm->preq, 7);
  dprintk("pres:       %P\n", sm->pres, 7);
  dprintk("init@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->peer->lookup_addr));
  dprintk("resp@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->local_addr));

  err = ble_c1(&sm->aes_dev, sm->tk, sm->srand, sm->preq, sm->pres,
               &sm->peer->lookup_addr, &sm->local_addr,
               sconfirm);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("sconf_tx:   %P\n", sconfirm, 16);

  return 0;
}

static uint8_t sm_mconf_setup(struct ble_sm_s *sm)
{
  error_t err;

  dprintk("Creating mconf:\n");

  err = dev_rng_wait_read(sm->rng, sm->mrand, sizeof(sm->mrand));
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("mrand:      %P\n", sm->mrand, 16);
  dprintk("preq:       %P\n", sm->preq, 7);
  dprintk("pres:       %P\n", sm->pres, 7);
  dprintk("init@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->peer->lookup_addr));
  dprintk("resp@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->local_addr));

  err = ble_c1(&sm->aes_dev, sm->tk, sm->mrand, sm->preq, sm->pres,
               &sm->peer->lookup_addr, &sm->local_addr,
               sm->mconf);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("mconf:      %P\n", sm->mconf, 16);

  return 0;
}

static uint8_t sm_mconf_check(struct ble_sm_s *sm, const uint8_t *mrand)
{
  error_t err;
  uint8_t mconf[16];
  uint8_t authreq = sm->preq[3];
  uint8_t stk[16];

  dprintk("Checking mconf:\n");

  dprintk("mrand:      %P\n", mrand, 16);
  dprintk("preq:       %P\n", sm->preq, 7);
  dprintk("pres:       %P\n", sm->pres, 7);
  dprintk("init@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->peer->lookup_addr));
  dprintk("resp@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->local_addr));
  dprintk("mconf_rx:   %P\n", sm->mconf, 16);

  err = ble_c1(&sm->aes_dev, sm->tk, mrand, sm->preq, sm->pres,
               &sm->peer->lookup_addr, &sm->local_addr,
               mconf);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("mconf_calc: %P\n", mconf, 16);

  if (memcmp(sm->mconf, mconf, 16))
    return BLE_SM_REASON_CONFIRM_VALUE_FAILED;

  dprintk("mconf OK\n");

  err = ble_s1(&sm->aes_dev, sm->tk, sm->srand, mrand, stk);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("STK:        %P\n", stk, 16);

  err = ble_peer_paired(sm->peer,
                        (authreq & BLE_SM_REQ_BONDING_MASK) == BLE_SM_REQ_BONDING,
                        !!(authreq & BLE_SM_REQ_MITM),
                        !!(authreq & BLE_SM_REQ_SC),
                        stk);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  return 0;
}

static uint8_t sm_sconf_check(struct ble_sm_s *sm, const uint8_t *sconf)
{
  error_t err;
  uint8_t sconf_calc[16];
  uint8_t authreq = sm->preq[3];
  uint8_t stk[16];

  dprintk("Checking sconf:\n");

  dprintk("srand:      %P\n", sm->srand, 16);
  dprintk("preq:       %P\n", sm->preq, 7);
  dprintk("pres:       %P\n", sm->pres, 7);
  dprintk("init@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->peer->lookup_addr));
  dprintk("resp@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->local_addr));
  dprintk("sconf_rx:   %P\n", sm->sconf, 16);

  err = ble_c1(&sm->aes_dev, sm->tk, sm->srand, sm->preq, sm->pres,
               &sm->peer->lookup_addr, &sm->local_addr,
               sconf_calc);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("sconf_calc: %P\n", sconf_calc, 16);

  if (memcmp(sconf_calc, sconf, 16))
    return BLE_SM_REASON_CONFIRM_VALUE_FAILED;

  dprintk("sconf OK\n");

  err = ble_s1(&sm->aes_dev, sm->tk, sm->srand, sm->mrand, stk);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  dprintk("STK:        %P\n", stk, 16);

  err = ble_peer_paired(sm->peer,
                        (authreq & BLE_SM_REQ_BONDING_MASK) == BLE_SM_REQ_BONDING,
                        !!(authreq & BLE_SM_REQ_MITM),
                        !!(authreq & BLE_SM_REQ_SC),
                        stk);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  return 0;
}

static uint8_t sm_expected_compute(uint8_t filt)
{
  uint8_t ret = 0;

  if (filt & BLE_SM_ENC_KEY) {
    ret |= EXPECT_ENCRYPTION_INFORMATION;
    ret |= EXPECT_MASTER_IDENTIFICATION;
  }

  if (filt & BLE_SM_ID_KEY) {
    ret |= EXPECT_IDENTITY_INFORMATION;
    ret |= EXPECT_IDENTITY_ADDRESS_INFORMATION;
  }

  if (filt & BLE_SM_SIGN_KEY) {
    ret |= EXPECT_CSRK;
  }

  return ret;
}

static void sm_distribute(struct ble_sm_s *sm)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
  };
  struct buffer_s *pkt;
  struct net_task_s *task;

  if (sm->to_distribute & EXPECT_ENCRYPTION_INFORMATION) {
    pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 17);
    if (pkt) {
      pkt->data[pkt->begin] = BLE_SM_ENCRYPTION_INFORMATION;
      ble_peer_ltk_get(sm->peer, pkt->data + pkt->begin + 1);
      task = net_scheduler_task_alloc(sm->layer.scheduler);
      if (task)
        net_task_outbound_push(task,
                               sm->layer.parent, &sm->layer,
                               0, NULL, &dst, pkt);
      buffer_refdec(pkt);
    }
  }

  if (sm->to_distribute & EXPECT_MASTER_IDENTIFICATION) {
    uint16_t ediv;

    pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 11);
    if (pkt) {
      pkt->data[pkt->begin] = BLE_SM_MASTER_IDENTIFICATION;
      ble_peer_id_get(sm->peer, pkt->data + pkt->begin + 3, &ediv);
      endian_le16_na_store(pkt->data + pkt->begin + 1, ediv);
      task = net_scheduler_task_alloc(sm->layer.scheduler);
      if (task)
        net_task_outbound_push(task,
                               sm->layer.parent, &sm->layer,
                               0, NULL, &dst, pkt);
      buffer_refdec(pkt);
    }
  }

  if (sm->to_distribute & EXPECT_IDENTITY_INFORMATION) {
    pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 17);
    if (pkt) {
      pkt->data[pkt->begin] = BLE_SM_IDENTITY_INFORMATION;
      memcpy(pkt->data + pkt->begin + 1, sm->peer->db->irk, 16);
      task = net_scheduler_task_alloc(sm->layer.scheduler);
      if (task)
        net_task_outbound_push(task,
                               sm->layer.parent, &sm->layer,
                               0, NULL, &dst, pkt);
      buffer_refdec(pkt);
    }
  }

  if (sm->to_distribute & EXPECT_IDENTITY_ADDRESS_INFORMATION) {
    pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 8);
    if (pkt) {
      pkt->data[pkt->begin] = BLE_SM_IDENTITY_ADDRESS_INFORMATION;
      pkt->data[pkt->begin + 1] = sm->local_addr.type == BLE_ADDR_RANDOM;
      memcpy(pkt->data + pkt->begin + 2, sm->local_addr.addr, 6);
      task = net_scheduler_task_alloc(sm->layer.scheduler);
      if (task)
        net_task_outbound_push(task,
                               sm->layer.parent, &sm->layer,
                               0, NULL, &dst, pkt);
      buffer_refdec(pkt);
    }
  }

  if (sm->to_distribute & EXPECT_CSRK) {
    pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 17);
    if (pkt) {
      pkt->data[pkt->begin] = BLE_SM_SIGNING_INFORMATION;
      ble_peer_csrk_get(sm->peer, pkt->data + pkt->begin + 1);
      task = net_scheduler_task_alloc(sm->layer.scheduler);
      if (task)
        net_task_outbound_push(task,
                               sm->layer.parent, &sm->layer,
                               0, NULL, &dst, pkt);
      buffer_refdec(pkt);
    }
  }
#endif
}

static void sm_command_handle(struct ble_sm_s *sm, struct net_task_s *task)
{
  const uint8_t *data = task->packet.buffer->data + task->packet.buffer->begin;
  const size_t size = task->packet.buffer->end - task->packet.buffer->begin;
  struct buffer_s *rsp = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 0);
  const struct ble_sm_delegate_vtable_s *vtable
    = const_ble_sm_delegate_vtable_s_from_base(sm->layer.delegate_vtable);
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
  };
  uint8_t err;

  dprintk("SM > %P\n", data, size);

  switch (data[0]) {
  case BLE_SM_PAIRING_REQUEST:
    if (size != 7) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm->pairing_state != BLE_SM_IDLE && sm->pairing_state != BLE_SM_REQUESTED) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      dprintk("Bad state %d\n", sm->pairing_state);
      goto error;
    }

    memcpy(sm->preq, data, 7);

    sm->to_distribute = 0;
    sm->to_expect = 0;

    sm->pairing_state = BLE_SM_REQUEST_DONE;

    vtable->pairing_requested(sm->layer.delegate, &sm->layer,
                              !!(sm->preq[3] & BLE_SM_REQ_BONDING));
    goto out;

  case BLE_SM_PAIRING_RESPONSE:
    if (size != 7) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm->pairing_state != BLE_SM_REQUEST_DONE) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      dprintk("Bad state %d\n", sm->pairing_state);
      goto error;
    }

    memcpy(sm->pres, data, 7);

    sm->to_expect = sm_expected_compute(sm->pres[6]);
    sm->to_distribute = sm_expected_compute(sm->pres[5]);

    sm->pairing_state = BLE_SM_REQUEST_ANSWERED;

    err = sm_mconf_setup(sm);
    if (err)
      goto error;

    rsp->data[rsp->begin] = BLE_SM_PAIRING_CONFIRM;
    memcpy(rsp->data + rsp->begin + 1, sm->mconf, 16);
    rsp->end = rsp->begin + 17;

    sm->pairing_state = BLE_SM_MCONF_SENT;

    goto send;

  case BLE_SM_PAIRING_CONFIRM:
    if (size != 17) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm_is_slave(sm)) {
      if (sm->pairing_state != BLE_SM_REQUEST_ANSWERED) {
        err = BLE_SM_REASON_INVALID_PARAMETERS;
        goto error;
      }

      dprintk("mconf stored.\n");

      rsp->data[rsp->begin] = BLE_SM_PAIRING_CONFIRM;
      // compute sconfirm, send it
      err = sm_sconf_setup(sm, rsp->data + rsp->begin + 1);
      if (err)
        goto error;

      rsp->end = rsp->begin + 17;

      memcpy(sm->mconf, data + 1, 16);

      sm->pairing_state = BLE_SM_SCONF_SENT;
    } else {
      if (sm->pairing_state != BLE_SM_MCONF_SENT) {
        err = BLE_SM_REASON_INVALID_PARAMETERS;
        goto error;
      }

      rsp->data[rsp->begin] = BLE_SM_PAIRING_RANDOM;
      memcpy(rsp->data + rsp->begin + 1, sm->mrand, 16);
      rsp->end = rsp->begin + 17;

      sm->pairing_state = BLE_SM_MRAND_SENT;
    }

    goto send;

  case BLE_SM_PAIRING_RANDOM:
    if (size != 17) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm_is_slave(sm)) {
      if (sm->pairing_state != BLE_SM_SCONF_SENT) {
        err = BLE_SM_REASON_INVALID_PARAMETERS;
        goto error;
      }

      err = sm_mconf_check(sm, data + 1);
      if (err)
        goto error;

      rsp->data[rsp->begin] = BLE_SM_PAIRING_RANDOM;
      memcpy(rsp->data + rsp->begin + 1, sm->srand, 16);
      rsp->end = rsp->begin + 17;

      sm->pairing_state = BLE_SM_STK_DONE;

      vtable->pairing_success(sm->layer.delegate, &sm->layer);

      goto send;
    } else {
      if (sm->pairing_state != BLE_SM_MRAND_SENT) {
        err = BLE_SM_REASON_INVALID_PARAMETERS;
        goto error;
      }

      err = sm_sconf_check(sm, data + 1);
      if (err)
        goto error;

      sm->pairing_state = BLE_SM_STK_DONE;

      vtable->pairing_success(sm->layer.delegate, &sm->layer);

      goto out;
    }

#if defined(CONFIG_BLE_SECURITY_DB)
  case BLE_SM_ENCRYPTION_INFORMATION:
    if (sm->pairing_state != BLE_SM_DISTRIBUTION_DONE
        || !sm->layer.context.addr.encrypted
        || (!(sm->to_expect & EXPECT_ENCRYPTION_INFORMATION))) {
      err = BLE_SM_REASON_AUTHENTICATION_REQUIREMENTS;
      goto error;
    }

    sm->to_expect &= ~EXPECT_ENCRYPTION_INFORMATION;

    goto expected_rx;

  case BLE_SM_MASTER_IDENTIFICATION:
    if (sm->pairing_state != BLE_SM_DISTRIBUTION_DONE
        || !sm->layer.context.addr.encrypted
        || (!(sm->to_expect & EXPECT_MASTER_IDENTIFICATION))) {
      err = BLE_SM_REASON_AUTHENTICATION_REQUIREMENTS;
      goto error;
    }

    sm->to_expect &= ~EXPECT_MASTER_IDENTIFICATION;

    goto expected_rx;

  case BLE_SM_IDENTITY_INFORMATION:
    if (sm->pairing_state != BLE_SM_DISTRIBUTION_DONE
        || !sm->layer.context.addr.encrypted
        || (!(sm->to_expect & EXPECT_IDENTITY_INFORMATION))) {
      err = BLE_SM_REASON_AUTHENTICATION_REQUIREMENTS;
      goto error;
    }

    ble_peer_irk_set(sm->peer, data + 1);
    sm->to_expect &= ~EXPECT_IDENTITY_INFORMATION;

    goto expected_rx;

  case BLE_SM_IDENTITY_ADDRESS_INFORMATION:
    if (sm->pairing_state != BLE_SM_DISTRIBUTION_DONE
        || !sm->layer.context.addr.encrypted
        || (!(sm->to_expect & EXPECT_IDENTITY_ADDRESS_INFORMATION))) {
      err = BLE_SM_REASON_AUTHENTICATION_REQUIREMENTS;
      goto error;
    } else {
      struct ble_addr_s addr;

      addr.type = data[1] ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
      memcpy(addr.addr, data + 2, 6);
      ble_peer_addr_set(sm->peer, &addr);

      sm->to_expect &= ~EXPECT_IDENTITY_ADDRESS_INFORMATION;

      goto expected_rx;
    }
#endif

  case BLE_SM_PAIRING_FAILED:
    dprintk("SM: Pairing failed beause of peer (%d)\n", data[1]);
    sm->pairing_state = BLE_SM_IDLE;
    goto out;

  default:
    err = BLE_SM_REASON_INVALID_PARAMETERS;
    goto error;
  }

  goto out;

 expected_rx:
  if (sm->to_expect == 0) {
    sm->pairing_state = BLE_SM_RECEPTION_DONE;

    dprintk("SM: All expected information received, now paired\n");
    ble_peer_save(sm->peer);

    if (sm_is_slave(sm)) {
      dprintk("SM: All expected information received, sending ours\n");
      sm_distribute(sm);
      sm->pairing_state = BLE_SM_DISTRIBUTION_DONE;
    }
  }
  goto out;

 error:
  rsp->end = rsp->begin + 2;;
  rsp->data[rsp->begin] = BLE_SM_PAIRING_FAILED;
  rsp->data[rsp->begin + 1] = err;
  sm->pairing_state = BLE_SM_IDLE;

  dprintk("Sm state %d sending error %d after packet %P\n", sm->pairing_state, err, data, size);

 send:
  dprintk("SM < %P\n", rsp->data + rsp->begin, rsp->end - rsp->begin);

  task = net_scheduler_task_alloc(sm->layer.scheduler);
  if (task)
    net_task_outbound_push(task,
                           sm->layer.parent, &sm->layer,
                           0, NULL, &dst, rsp);

 out:
  buffer_refdec(rsp);
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

static bool_t ble_sm_context_updated(struct net_layer_s *layer,
                                     const struct net_layer_context_s *parent_context)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  if (!layer->parent)
    return 1;

  layer->context = *parent_context;

  if (parent_context->addr.encrypted
      && sm->pairing_state == BLE_SM_STK_DONE) {

    if (sm_is_slave(sm)) {
      dprintk("SM: LL security enabled, distributing data\n");
      sm_distribute(sm);
      sm->pairing_state = BLE_SM_DISTRIBUTION_DONE;

      if (sm->to_expect == 0) {
        sm->pairing_state = BLE_SM_RECEPTION_DONE;
        dprintk("SM: All expected information received, now paired\n");
        ble_peer_save(sm->peer);
      }
    } else {
      if (sm->to_expect == 0) {
        dprintk("SM: No information to expect, sending ours\n");
        sm_distribute(sm);
        sm->pairing_state = BLE_SM_DISTRIBUTION_DONE;
        ble_peer_save(sm->peer);
      }
    }
  }

  return 1;
}

static
void sm_pairing_request(struct net_layer_s *layer,
                        bool_t mitm_protection,
                        bool_t bonding)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);
  const struct ble_sm_delegate_vtable_s *vtable
    = const_ble_sm_delegate_vtable_s_from_base(sm->layer.delegate_vtable);

  if (sm->pairing_state != BLE_SM_IDLE || !sm->layer.parent)
    return;

  if (sm_is_slave(sm)) {
    struct buffer_s *rsp = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 2);
    struct net_addr_s dst = {
      .cid = BLE_L2CAP_CID_SM,
    };

    rsp->data[rsp->begin] = BLE_SM_SECURITY_REQUEST;
    rsp->data[rsp->begin + 1] = 0
      | (_CONFIG_BLE_SECURITY_DB && bonding ? BLE_SM_REQ_BONDING : 0)
      | (mitm_protection ? BLE_SM_REQ_MITM : 0);

    sm->pairing_state = BLE_SM_REQUESTED;

    struct net_task_s *task = net_scheduler_task_alloc(sm->layer.scheduler);
    if (task)
      net_task_outbound_push(task,
                             sm->layer.parent, &sm->layer,
                             0, NULL, &dst, rsp);
    buffer_refdec(rsp);
  } else {
    sm->pairing_state = BLE_SM_REQUESTED;

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

  if (!sm->layer.parent)
    return;

  struct buffer_s *pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 7);
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
  };

  if (oob_data) {
    memcpy(sm->tk, oob_data, 16);
  } else {
    endian_le32_na_store(sm->tk, pin);
    memset(sm->tk + 4, 0, 12);
  }

  if (sm_is_slave(sm)) {
    if (sm->pairing_state != BLE_SM_REQUEST_DONE)
      return;

    sm->pres[0] = BLE_SM_PAIRING_RESPONSE;
    sm->pres[1] = sm->io_cap;
    sm->pres[2] = oob_data ? 1 : 0;

#if !defined(CONFIG_BLE_SECURITY_DB)
    sm->pres[3] = 0;
    sm->pres[4] = 16;
    sm->pres[5] = 0;
    sm->pres[6] = 0;
#else
    sm->pres[3] = (_CONFIG_BLE_SECURITY_DB ? BLE_SM_REQ_BONDING : 0)
      | (mitm_protection ? BLE_SM_REQ_MITM : 0);
    sm->pres[4] = 16;
    sm->pres[5] = sm->preq[5] & (BLE_SM_ENC_KEY | BLE_SM_ID_KEY);
    sm->pres[6] = sm->preq[6] & (BLE_SM_ENC_KEY | BLE_SM_ID_KEY);
#endif
    sm->to_distribute = sm_expected_compute(sm->pres[6]);
    sm->to_expect = sm_expected_compute(sm->pres[5]);

    memcpy(pkt->data + pkt->begin, sm->pres, 7);

    sm->pairing_state = BLE_SM_REQUEST_ANSWERED;

    dprintk("Pairing RSP sent\n");
  } else {
    if (sm->pairing_state != BLE_SM_REQUESTED)
      return;

    sm->preq[0] = BLE_SM_PAIRING_REQUEST;
    sm->preq[1] = sm->io_cap;
    sm->preq[2] = oob_data ? 1 : 0;

#if !defined(CONFIG_BLE_SECURITY_DB)
    sm->preq[3] = 0;
    sm->preq[4] = 16;
    sm->preq[5] = 0;
    sm->preq[6] = 0;
#else
    sm->preq[3] = (_CONFIG_BLE_SECURITY_DB ? BLE_SM_REQ_BONDING : 0)
      | (mitm_protection ? BLE_SM_REQ_MITM : 0);
    sm->preq[4] = 16;
    sm->preq[5] = BLE_SM_ENC_KEY | BLE_SM_ID_KEY;
    sm->preq[6] = BLE_SM_ENC_KEY | BLE_SM_ID_KEY;
#endif

    memcpy(pkt->data + pkt->begin, sm->preq, 7);

    sm->to_distribute = 0;
    sm->to_expect = 0;

    sm->pairing_state = BLE_SM_REQUEST_DONE;

    dprintk("Pairing REQ sent\n");
  }

  struct net_task_s *task = net_scheduler_task_alloc(sm->layer.scheduler);
  if (task)
    net_task_outbound_push(task,
                           sm->layer.parent, &sm->layer,
                           0, NULL, &dst, pkt);
    buffer_refdec(pkt);
}

static
void sm_pairing_abort(struct net_layer_s *layer, enum sm_reason reason)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  if (sm->pairing_state != BLE_SM_IDLE || !sm->layer.parent)
    return;

  struct buffer_s *rsp = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 2);
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
  };

  rsp->data[rsp->begin] = BLE_SM_PAIRING_FAILED;
  rsp->data[rsp->begin + 1] = reason;

  sm->pairing_state = BLE_SM_IDLE;

  struct net_task_s *task = net_scheduler_task_alloc(sm->layer.scheduler);
  if (task)
    net_task_outbound_push(net_scheduler_task_alloc(sm->layer.scheduler),
                           sm->layer.parent, &sm->layer,
                           0, NULL, &dst, rsp);
  buffer_refdec(rsp);

  dprintk("Pairing aborted\n");
}

static const struct ble_sm_handler_s sm_handler = {
  .base = {
    .destroyed = ble_sm_destroyed,
    .task_handle = ble_sm_task_handle,
    .context_updated = ble_sm_context_updated,
    .type = BLE_NET_LAYER_SM,
  },
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

  err = device_copy_accessor(&sm->aes_dev, params->crypto);
  if (err)
    goto err_out;

  err = net_layer_init(&sm->layer, &sm_handler.base, scheduler, delegate, &delegate_vtable->base);

  if (err)
    goto err_put_aes;

  sm->peer = params->peer;
  sm->io_cap = BLE_SM_IO_CAP_NO_INPUT_NO_OUTPUT;
  sm->local_addr = params->local_addr;

  return 0;

 err_put_aes:
  device_put_accessor(&sm->aes_dev);
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
