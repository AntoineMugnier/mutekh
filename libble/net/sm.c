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

#include <ble/net/sm.h>
#include <ble/net/slave.h>
#include <ble/crypto.h>
#include <ble/protocol/sm.h>
#include <ble/protocol/l2cap.h>

static
void ble_sm_destroyed(struct net_layer_s *layer)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);

  device_put_accessor(&sm->aes_dev);

  sm->handler->destroyed(sm);
}

static uint8_t sm_pairing_setup(struct ble_sm_s *sm, uint8_t *sconfirm)
{
  error_t err;

  if (sm->oob) {
    memcpy(sm->tk, sm->oob, 16);
  } else {
    memset(sm->tk + 4, 0, 12);
    endian_le32_na_store(sm->tk, sm->passkey);
  }

  err = ble_peer_reset(sm->peer);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  err = dev_rng_wait_read(sm->rng, sm->srand, sizeof(sm->srand));
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  printk("Starting pairing, new device id: %lld\n", sm->peer->id);
  printk("Generating sconf:\n");

  printk("TK:         %P\n", sm->tk, 16);
  printk("srand:      %P\n", sm->srand, 16);
  printk("preq:       %P\n", sm->preq, 7);
  printk("pres:       %P\n", sm->pres, 7);
  printk("init@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->peer->lookup_addr));
  printk("resp@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->local_addr));

  err = ble_c1(&sm->aes_dev, sm->tk, sm->srand, sm->preq, sm->pres,
               &sm->peer->lookup_addr, &sm->local_addr,
               sconfirm);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  printk("sconf_tx:   %P\n", sconfirm, 16);

  return 0;
}

static uint8_t sm_pairing_check(struct ble_sm_s *sm, const uint8_t *mrand)
{
  error_t err;
  uint8_t mconf[16];
  uint8_t authreq = sm->preq[3];
  uint8_t stk[16];

  printk("Checking mconf:\n");

  printk("mrand:      %P\n", mrand, 16);
  printk("preq:       %P\n", sm->preq, 7);
  printk("pres:       %P\n", sm->pres, 7);
  printk("init@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->peer->lookup_addr));
  printk("resp@:      "BLE_ADDR_FMT"\n", BLE_ADDR_ARG(&sm->local_addr));
  printk("mconf_rx:   %P\n", sm->mconf, 16);

  err = ble_c1(&sm->aes_dev, sm->tk, mrand, sm->preq, sm->pres,
               &sm->peer->lookup_addr, &sm->local_addr,
               mconf);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  printk("mconf_calc: %P\n", mconf, 16);

  if (memcmp(sm->mconf, mconf, 16))
    return BLE_SM_REASON_CONFIRM_VALUE_FAILED;

  printk("mconf OK\n");

  err = ble_s1(&sm->aes_dev, sm->tk, sm->srand, mrand, stk);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  printk("STK:        %P\n", stk, 16);

  err = ble_peer_paired(sm->peer,
                        (authreq & BLE_SM_REQ_BONDING_MASK) == BLE_SM_REQ_BONDING,
                        !!(authreq & BLE_SM_REQ_MITM),
                        !!(authreq & BLE_SM_REQ_SC),
                        stk);
  if (err)
    return BLE_SM_REASON_UNSPECIFIED_REASON;

  return 0;
}

static void sm_command_handle(struct ble_sm_s *sm, struct net_task_s *task)
{
  const uint8_t *data = task->inbound.buffer->data + task->inbound.buffer->begin;
  const size_t size = task->inbound.buffer->end - task->inbound.buffer->begin;
  struct buffer_s *rsp = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 0);
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
    .reliable = 1,
  };
  uint8_t err;

  switch (data[0]) {
  case BLE_SM_PAIRING_REQUEST:
    if (size != 7) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm->pairing_state != BLE_SM_IDLE) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    memcpy(sm->preq, data, 7);

    sm->pres[0] = BLE_SM_PAIRING_RESPONSE;
    sm->pres[1] = sm->io_cap;
    sm->pres[2] = sm->oob ? 1 : 0;
    sm->pres[3] = (sm->bonding_enabled ? BLE_SM_REQ_BONDING : 0)
                | (sm->mitm_protection ? BLE_SM_REQ_MITM : 0);
    sm->pres[4] = 16;
    sm->pres[5] = sm->preq[5] & (BLE_SM_ENC_KEY | BLE_SM_ID_KEY);
    sm->pres[6] = sm->preq[6] & (BLE_SM_ENC_KEY | BLE_SM_ID_KEY);

    memcpy(rsp->data + rsp->begin, sm->pres, 7);
    rsp->end = rsp->begin + 7;

    sm->pairing_state = BLE_SM_REQUEST_DONE;

    goto send;

  case BLE_SM_PAIRING_CONFIRM:
    if (size != 17) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm->pairing_state != BLE_SM_REQUEST_DONE) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    printk("mconf stored.\n");

    rsp->data[rsp->begin] = BLE_SM_PAIRING_CONFIRM;
    // compute sconfirm, send it
    err = sm_pairing_setup(sm, rsp->data + rsp->begin + 1);
    if (err)
      goto error;

    rsp->end = rsp->begin + 17;

    memcpy(sm->mconf, data + 1, 16);

    sm->pairing_state = BLE_SM_STK_DONE;

    goto send;

  case BLE_SM_PAIRING_RANDOM:
    if (size != 17) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    if (sm->pairing_state != BLE_SM_STK_DONE) {
      err = BLE_SM_REASON_INVALID_PARAMETERS;
      goto error;
    }

    err = sm_pairing_check(sm, data + 1);
    if (err)
      goto error;

    rsp->data[rsp->begin] = BLE_SM_PAIRING_RANDOM;
    memcpy(rsp->data + rsp->begin + 1, sm->srand, 16);
    rsp->end = rsp->begin + 17;

    sm->pairing_state = BLE_SM_STK_DONE;

    goto send;

  default:
    err = BLE_SM_REASON_INVALID_PARAMETERS;
    goto error;
  }

  goto out;

 error:
  rsp->end = rsp->begin + 2;;
  rsp->data[rsp->begin] = BLE_SM_PAIRING_FAILED;
  rsp->data[rsp->begin + 1] = err;
  sm->pairing_state = BLE_SM_IDLE;

 send:
  printk("Sm sending packet\n");

  net_task_inbound_push(net_scheduler_task_alloc(sm->layer.scheduler),
                        sm->layer.parent, &sm->layer,
                        0, NULL, &dst, rsp);

 out:
  buffer_refdec(rsp);
}

static
void ble_sm_task_handle(struct net_layer_s *layer,
                        struct net_task_header_s *header)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  switch (header->type) {
  case NET_TASK_INBOUND:
    printk("Sm inbound from %p (parent %p)\n", task->header.source, layer->parent);
    if (task->header.source == layer->parent)
      sm_command_handle(sm, task);
    break;

  default:
    break;
  }

  net_task_cleanup(task);
}

static bool_t ble_sm_context_updated(struct net_layer_s *layer,
                                     const struct net_layer_context_s *parent_context)
{
  struct ble_sm_s *sm = ble_sm_s_from_layer(layer);
  struct buffer_s *pkt;
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SM,
    .reliable = 1,
  };

  layer->context = *parent_context;

  if (parent_context->addr.secure
      && sm->pairing_state == BLE_SM_STK_DONE
      && sm->layer.parent) {
    uint8_t to_distribute = sm->pres[6];

    /* error_t err = ble_peer_ */

    printk("SM: LL security enabled, distributing keys\n");

    if (to_distribute & BLE_SM_ENC_KEY) {
      uint16_t ediv;

      pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 17);
      pkt->data[pkt->begin] = BLE_SM_ENCRYPTION_INFORMATION;
      ble_peer_ltk_get(sm->peer, pkt->data + pkt->begin + 1);
      net_task_inbound_push(net_scheduler_task_alloc(sm->layer.scheduler),
                            sm->layer.parent, &sm->layer,
                            0, NULL, &dst, pkt);

      pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 11);
      pkt->data[pkt->begin] = BLE_SM_MASTER_IDENTIFICATION;
      ble_peer_id_get(sm->peer, pkt->data + pkt->begin + 3, &ediv);
      endian_le16_na_store(pkt->data + pkt->begin + 1, ediv);
      net_task_inbound_push(net_scheduler_task_alloc(sm->layer.scheduler),
                            sm->layer.parent, &sm->layer,
                            0, NULL, &dst, pkt);
    }

    if (to_distribute & BLE_SM_ID_KEY) {
      pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 17);
      pkt->data[pkt->begin] = BLE_SM_IDENTITY_INFORMATION;
      memcpy(pkt->data + pkt->begin + 1, sm->peer->db->irk, 16);
      net_task_inbound_push(net_scheduler_task_alloc(sm->layer.scheduler),
                            sm->layer.parent, &sm->layer,
                            0, NULL, &dst, pkt);

      pkt = net_layer_packet_alloc(&sm->layer, sm->layer.context.prefix_size, 8);
      pkt->data[pkt->begin] = BLE_SM_IDENTITY_INFORMATION;
      pkt->data[pkt->begin + 1] = 0;
      pkt->data[pkt->begin + 2] = 0;
      pkt->data[pkt->begin + 3] = 0;
      pkt->data[pkt->begin + 4] = 0;
      pkt->data[pkt->begin + 5] = 0;
      pkt->data[pkt->begin + 6] = 0;
      pkt->data[pkt->begin + 7] = 0;
      net_task_inbound_push(net_scheduler_task_alloc(sm->layer.scheduler),
                            sm->layer.parent, &sm->layer,
                            0, NULL, &dst, pkt);
    }

    sm->pairing_state = BLE_SM_DISTRIBUTION_DONE;
  }

  return 1;
}

static const struct net_layer_handler_s sm_handler = {
  .destroyed = ble_sm_destroyed,
  .task_handle = ble_sm_task_handle,
  .context_updated = ble_sm_context_updated,
  .type = BLE_LAYER_TYPE_SM,
};

error_t ble_sm_init(
  struct ble_sm_s *sm,
  const struct ble_sm_handler_s *handler,
  struct net_scheduler_s *scheduler,
  struct ble_peer_s *peer,
  const struct ble_addr_s *local_addr,
  struct dev_rng_s *rng,
  const char *aes_dev)
{
  error_t err;

  memset(sm, 0, sizeof(*sm));

  sm->rng = rng;

  err = device_get_accessor_by_path(&sm->aes_dev, NULL, aes_dev, DRIVER_CLASS_CRYPTO);
  if (err)
    goto err_out;

  err = net_layer_init(&sm->layer, &sm_handler, scheduler);

  if (err)
    goto err_put_aes;

  sm->handler = handler;
  sm->peer = peer;
  sm->io_cap = BLE_SM_IO_CAP_NO_INPUT_NO_OUTPUT;
  sm->local_addr = *local_addr;

  return 0;

 err_put_aes:
  device_put_accessor(&sm->aes_dev);
 err_out:
  return err;
}
