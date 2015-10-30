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

#include <ble/net/att.h>
#include <ble/protocol/att.h>
#include <ble/protocol/l2cap.h>
#include <ble/net/layer.h>

#include <ble/net/generic.h>
#include <string.h>
#include "att_encoding.h"

//#define dprintk(...) do{}while(0)
#define dprintk printk

STRUCT_COMPOSE(ble_att_s, layer);

static error_t att_response_send(struct ble_att_s *att,
                                 struct ble_att_transaction_s *txn);

static void att_transaction_first_send(struct ble_att_s *att);

static void att_command_handle(struct ble_att_s *att, struct net_task_s *task)
{
  const uint8_t *data = task->inbound.buffer->data + task->inbound.buffer->begin;
  const size_t size = task->inbound.buffer->end - task->inbound.buffer->begin;
  uint8_t err;
  struct ble_att_transaction_s *txn = NULL;
  uint8_t opcode = data[0];
  struct buffer_s *rsp;

  if (size < 1) {
    err = BLE_ATT_ERR_INVALID_PDU;
    goto error;
  }

  if (ble_att_opcode_is_signed(opcode)) {
    err = BLE_ATT_ERR_UNLIKELY_ERROR;
    goto error;
  }

  switch ((enum ble_att_opcode_e)opcode) {
  case BLE_ATT_EXCHANGE_MTU_RQT:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    att->server_mtu = endian_le16_na_load(data + 1);
    att->mtu = __MIN(att->server_mtu, att->layer.context.mtu);

    rsp = net_task_inbound_buffer_steal(task, att->layer.context.prefix_size, 3);
    rsp->data[rsp->begin] = BLE_ATT_EXCHANGE_MTU_RSP;
    endian_le16_na_store(&rsp->data[rsp->begin + 1], att->layer.context.mtu);
    goto send;

  case BLE_ATT_EXCHANGE_MTU_RSP:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    att->server_mtu = endian_le16_na_load(data + 1);
    att->mtu = __MIN(att->server_mtu, att->layer.context.mtu);
    return;

  case BLE_ATT_ERROR_RSP:
  case BLE_ATT_FIND_INFORMATION_RSP:
  case BLE_ATT_FIND_BY_TYPE_VALUE_RSP:
  case BLE_ATT_READ_BY_TYPE_RSP:
  case BLE_ATT_READ_RSP:
  case BLE_ATT_READ_BLOB_RSP:
  case BLE_ATT_READ_MULTIPLE_RSP:
  case BLE_ATT_READ_BY_GROUP_TYPE_RSP:
  case BLE_ATT_WRITE_RSP:
  case BLE_ATT_PREPARE_WRITE_RSP:
  case BLE_ATT_EXECUTE_WRITE_RSP:
  case BLE_ATT_HANDLE_VALUE_CONFIRM:
    err = att_response_parse(att, task->inbound.buffer, &txn);
    if (err)
      return;

    assert(txn && txn == att->transaction_pending);
    att->transaction_pending = NULL;
    net_task_query_respond_push(&txn->task, 0);
    att_transaction_first_send(att);
    return;

  case BLE_ATT_FIND_INFORMATION_RQT:
  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT:
  case BLE_ATT_READ_BY_TYPE_RQT:
  case BLE_ATT_READ_RQT:
  case BLE_ATT_READ_BLOB_RQT:
  case BLE_ATT_READ_MULTIPLE_RQT:
  case BLE_ATT_READ_BY_GROUP_TYPE_RQT:
  case BLE_ATT_WRITE_RQT:
  case BLE_ATT_WRITE_CMD:
  case BLE_ATT_SIGNED_WRITE_CMD:
  case BLE_ATT_PREPARE_WRITE_RQT:
  case BLE_ATT_EXECUTE_WRITE_RQT:
    if (!att->server) {
      err = BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;
      goto error;
    }

    err = att_request_parse(att, task->inbound.buffer, &txn);
    if (err)
      goto error;

    net_task_query_push(&txn->task, att->server, &att->layer, BLE_ATT_REQUEST);
    return;

  case BLE_ATT_HANDLE_VALUE_NOTIF:
  case BLE_ATT_HANDLE_VALUE_INDIC:
    if (!att->client)
      return;

    err = att_request_parse(att, task->inbound.buffer, &txn);
    if (err)
      return;

    net_task_query_push(&txn->task, att->client, &att->layer, BLE_ATT_REQUEST);
    return;

  default:
    err = BLE_ATT_ERR_INVALID_PDU;
    goto error;
  }

 error:
  rsp = net_task_inbound_buffer_steal(task, att->layer.context.prefix_size, 5);
  rsp->data[rsp->begin] = BLE_ATT_ERROR_RSP;
  rsp->data[rsp->begin + 1] = opcode;
  endian_le16_na_store(&rsp->data[rsp->begin + 2], 0);
  rsp->data[rsp->begin + 4] = err;

 send:;
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };
  struct net_task_s *rsp_task = net_scheduler_task_alloc(att->layer.scheduler);
  if (rsp_task)
    dprintk("Att rsp < %P\n", rsp->data + rsp->begin, rsp->end - rsp->begin);
    net_task_inbound_push(rsp_task,
                          att->layer.parent, &att->layer,
                          0, NULL, &dst, rsp);
  buffer_refdec(rsp);
}

static
void ble_att_task_handle(struct net_layer_s *layer,
                         struct net_task_header_s *header)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  switch (header->type) {
  default:
    break;

  case NET_TASK_QUERY:
    dprintk("Att query from %p\n", task->header.source);

    if (ble_att_transaction_s_from_task(task)->command == BLE_ATT_HANDLE_VALUE_NOTIF
        && !net_task_queue_isempty(&att->transaction_queue)) {
      net_task_query_respond_push(task, -EBUSY);
      return;
    }

    net_task_queue_pushback(&att->transaction_queue, &task->header);
    att_transaction_first_send(att);
    return;

  case NET_TASK_RESPONSE:
    dprintk("Att response from %p\n", task->header.source);
    if (ble_att_opcode_is_response_expected(ble_att_transaction_s_from_task(task)->command))
      att_response_send(att, ble_att_transaction_s_from_task(task));
    break;

  case NET_TASK_INBOUND:
    dprintk("Att inbound from %p (parent %p)\n", task->header.source, layer->parent);
    dprintk("Att > %P\n", task->inbound.buffer->data + task->inbound.buffer->begin, task->inbound.buffer->end - task->inbound.buffer->begin);
    if (task->header.source == layer->parent)
      att_command_handle(att, task);
    break;
  }

  net_task_destroy(task);
}

static
void ble_att_destroyed(struct net_layer_s *layer)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);

  dprintk("Att %p destroyed\n", att);

  mem_free(att);
}

static
error_t ble_att_bound(struct net_layer_s *layer,
                      void *addr,
                      struct net_layer_s *child)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);
  uint16_t proto = addr ? *(uint16_t *)addr : BLE_ATT_CLIENT;

  switch (proto) {
  case BLE_ATT_CLIENT:
    att->client = child;
    return 0;

  case BLE_ATT_SERVER:
    att->server = child;
    return 0;

  default:
    return -EINVAL;
  }
}

static
void ble_att_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);

  if (child == att->server)
    att->server = NULL;

  if (child == att->client)
    att->client = NULL;
}

static const struct net_layer_handler_s att_handler = {
  .destroyed = ble_att_destroyed,
  .task_handle = ble_att_task_handle,
  .bound = ble_att_bound,
  .unbound = ble_att_unbound,
  .type = BLE_NET_LAYER_ATT,
};

error_t ble_att_create(
  struct net_scheduler_s *scheduler,
  struct net_layer_s **layer)
{
  struct ble_att_s *att = mem_alloc(sizeof(*att), mem_scope_sys);

  if (!att)
    return -ENOMEM;

  error_t err = net_layer_init(&att->layer, &att_handler, scheduler, NULL, NULL);

  if (err) {
    mem_free(att);
    return err;
  }

  att->client = NULL;
  att->server = NULL;
  att->transaction_pending = NULL;
  net_task_queue_init(&att->transaction_queue);

  att->mtu = 23;
  att->server_mtu = 23;
  att->layer.context.prefix_size = 0;
  att->layer.context.mtu = 0;

  *layer = &att->layer;

  return 0;
}

static
void att_request_free(struct net_task_header_s *task)
{
  memset(task, 0xaa, sizeof(struct net_task_s));
  mem_free(task);
}

struct ble_att_transaction_s *att_request_allocate(struct ble_att_s *att,
                                                   size_t total_size)
{
  struct ble_att_transaction_s *txn = mem_alloc(total_size, mem_scope_sys);
  if (!txn)
    return NULL;

  memset(txn, 0, total_size);
  txn->task.header.destroy_func = att_request_free;
  txn->task.header.allocator_data = NULL;

  return txn;
}

static void att_transaction_first_send(struct ble_att_s *att)
{
  struct buffer_s *req = NULL;
  struct net_task_s *req_task = NULL;

  const struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };
  struct ble_att_transaction_s *txn;

  while ((txn = ble_att_transaction_s_from_task(
                  net_task_s_from_header(
                    net_task_queue_head(&att->transaction_queue))))) {
    error_t err;

    if (!att->layer.parent) {
      net_task_queue_pop(&att->transaction_queue);
      net_task_query_respond_push(&txn->task, -EPIPE);
      continue;
    }

    bool_t with_response = ble_att_opcode_is_response_expected(txn->command);

    if (att->transaction_pending && with_response)
      break;

    net_task_queue_pop(&att->transaction_queue);

    if (!req) {
      req = net_layer_packet_alloc(&att->layer, att->layer.context.prefix_size, 0);

      if (!req) {
        net_task_query_respond_push(&txn->task, -ENOMEM);
        continue;
      }
    }

    if (!req_task) {
      req_task = net_scheduler_task_alloc(att->layer.scheduler);

      if (!req_task) {
        net_task_query_respond_push(&txn->task, -ENOMEM);
        continue;
      }
    }

    err = att_request_serialize(req, txn, att->mtu);
    if (err) {
      net_task_query_respond_push(&txn->task, err);
      continue;
    }

    dprintk("Att req < %P\n", req->data + req->begin, req->end - req->begin);
    net_task_inbound_push(req_task, att->layer.parent, &att->layer, 0, NULL, &dst, req);
    buffer_refdec(req);

    if (with_response)
      att->transaction_pending = txn;

    req_task = NULL;
    req = NULL;
  }
}

static error_t att_response_send(struct ble_att_s *att,
                                 struct ble_att_transaction_s *txn)
{
  struct buffer_s *rsp = NULL;
  struct net_task_s *rsp_task = NULL;
  const struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };
  error_t err = -ENOMEM;

  if (!att->layer.parent)
    return -EPIPE;

  rsp = net_layer_packet_alloc(&att->layer, att->layer.context.prefix_size, 0);
  if (!rsp)
    return -ENOMEM;

  err = att_response_serialize(rsp, txn, att->mtu);
  if (err)
    goto error_free_buffer;

  assert(rsp->end <= buffer_size(rsp));

  rsp_task = net_scheduler_task_alloc(att->layer.scheduler);
  if (!rsp_task) {
    err = -ENOMEM;
    goto error_free_buffer;
  }

  dprintk("Att rsp < %P\n", rsp->data + rsp->begin, rsp->end - rsp->begin);
  net_task_inbound_push(rsp_task, att->layer.parent, &att->layer, 0, NULL, &dst, rsp);

 error_free_buffer:
  buffer_refdec(rsp);

  return err;
}
