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

#define LOGK_MODULE_ID "att_"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/att.h>
#include <ble/net/generic.h>

#include <ble/protocol/att.h>
#include <ble/protocol/l2cap.h>

#include <string.h>
#include "att_encoding.h"

STRUCT_COMPOSE(ble_att_s, layer);

static error_t att_response_send(struct ble_att_s *att,
                                 struct ble_att_transaction_s *txn);

static void att_transaction_first_send(struct ble_att_s *att);

static void att_command_handle(struct ble_att_s *att, struct net_task_s *task)
{
  struct buffer_s *pkt = task->packet.buffer;
  const uint8_t *data = pkt->data + pkt->begin;
  const size_t size = pkt->end - pkt->begin;
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };

  uint8_t err;
  struct ble_att_transaction_s *txn = NULL;
  uint8_t opcode = data[0];

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

    pkt->data[pkt->begin] = BLE_ATT_EXCHANGE_MTU_RSP;
    endian_le16_na_store(&pkt->data[pkt->begin + 1], att->layer.context.mtu);
    pkt->end = pkt->begin + 3;
    goto respond;

  case BLE_ATT_EXCHANGE_MTU_RSP:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    att->server_mtu = endian_le16_na_load(data + 1);
    att->mtu = __MIN(att->server_mtu, att->layer.context.mtu);
    goto destroy;

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
    err = att_response_parse(att, pkt, &txn);
    if (err)
      goto destroy;

    assert(txn && txn == att->transaction_pending);
    att->transaction_pending = NULL;
    net_task_query_respond_push(&txn->task, 0);
    att_transaction_first_send(att);
    goto destroy;

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

    err = att_request_parse(att, pkt, &txn);
    if (err)
      goto error;

    net_task_query_push(&txn->task, att->server, &att->layer, BLE_ATT_REQUEST);
    goto destroy;

  case BLE_ATT_HANDLE_VALUE_INDIC:
    if (!att->client) {
      err = BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;
      goto error;
    }

  case BLE_ATT_HANDLE_VALUE_NOTIF:
    if (!att->client)
      goto destroy;

    err = att_request_parse(att, pkt, &txn);
    if (err)
      goto destroy;

    net_task_query_push(&txn->task, att->client, &att->layer, BLE_ATT_REQUEST);
    goto destroy;

  default:
    err = BLE_ATT_ERR_INVALID_PDU;
    goto error;
  }

 error:
  pkt->data[pkt->begin] = BLE_ATT_ERROR_RSP;
  pkt->data[pkt->begin + 1] = opcode;
  endian_le16_na_store(&pkt->data[pkt->begin + 2], 0);
  pkt->data[pkt->begin + 4] = err;
  pkt->end = pkt->begin + 5;

 respond:
  logk_trace("rsp < %P", pkt->data + pkt->begin, pkt->end - pkt->begin);
  net_task_packet_respond(task, att->layer.parent, 0, &dst);
  return;

 destroy:
  net_task_destroy(task);
}

static
void ble_att_task_handle(struct net_layer_s *layer,
                         struct net_task_s *task)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);

  switch (task->type) {
  default:
    break;

  case NET_TASK_QUERY:
    logk_trace("query from %p", task->source);

    if (ble_att_transaction_s_from_task(task)->command == BLE_ATT_HANDLE_VALUE_NOTIF
        && !net_task_queue_isempty(&att->transaction_queue)) {
      logk("Too many requests pending, telling we are busy");
      net_task_query_respond_push(task, -EBUSY);
      return;
    }

    net_task_queue_pushback(&att->transaction_queue, task);
    att_transaction_first_send(att);
    return;

  case NET_TASK_RESPONSE:
    logk_trace("response from %p", task->source);
    if (ble_att_opcode_is_response_expected(ble_att_transaction_s_from_task(task)->command))
      att_response_send(att, ble_att_transaction_s_from_task(task));
    break;

  case NET_TASK_INBOUND: {
    logk_trace("inb > %P", task->packet.buffer->data + task->packet.buffer->begin, task->packet.buffer->end - task->packet.buffer->begin);

    const uint8_t *header = &task->packet.buffer->data[task->packet.buffer->begin];
    uint8_t op = *header;
    struct net_layer_s *target = NULL;

    switch (op) {
    case BLE_ATT_HANDLE_VALUE_NOTIF:
      task->packet.src_addr.att = endian_le16_na_load(header + 1);
      target = att->client;
      break;

    case BLE_ATT_WRITE_CMD:
      task->packet.dst_addr.att = endian_le16_na_load(header + 1);
      target = att->server;
      break;

    default:
      att_command_handle(att, task);
      return;
    }

    if (target && (task->packet.buffer->end - task->packet.buffer->begin) >= 3) {
      task->packet.buffer->begin += 3;
      net_task_packet_forward(task, target);
      return;
    }

    break;
  }

  case NET_TASK_OUTBOUND: {
    uint8_t header[3];
    if (task->packet.src_addr.att) {
      logk_trace("not %d > %P", 
              task->packet.src_addr.att,
              task->packet.buffer->data + task->packet.buffer->begin,
              task->packet.buffer->end - task->packet.buffer->begin);
      header[0] = BLE_ATT_HANDLE_VALUE_NOTIF;
      endian_le16_na_store(header + 1, task->packet.src_addr.att);
    } else if (task->packet.dst_addr.att) {
      logk_trace("wrt %d > %P", 
              task->packet.dst_addr.att,
              task->packet.buffer->data + task->packet.buffer->begin,
              task->packet.buffer->end - task->packet.buffer->begin);
      header[0] = BLE_ATT_WRITE_CMD;
      endian_le16_na_store(header + 1, task->packet.dst_addr.att);
    } else {
      break;
    }

    buffer_prepend(task->packet.buffer, header, 3);
    task->packet.dst_addr.cid = BLE_L2CAP_CID_ATT;

    if (layer->parent) {
      net_task_packet_forward(task, layer->parent);
      return;
    }
    break;
  }
  }

  net_task_destroy(task);
}

static
void ble_att_destroyed(struct net_layer_s *layer)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);

  logk_trace("%p destroyed", att);

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

static void ble_att_dandling(struct net_layer_s *layer)
{
  struct ble_att_s *att = ble_att_s_from_layer(layer);

  if (att->transaction_pending) {
    net_task_query_respond_push(&att->transaction_pending->task, -EIO);
    att->transaction_pending = NULL;
  }

  net_task_queue_reject_all(&att->transaction_queue);
}

static
void ble_att_child_context_adjust(const struct net_layer_s *layer,
                                  struct net_layer_context_s *cc)
{
  cc->prefix_size += 3;
  cc->mtu -= 3;
}

static const struct net_layer_handler_s att_handler = {
  .destroyed = ble_att_destroyed,
  .task_handle = ble_att_task_handle,
  .bound = ble_att_bound,
  .unbound = ble_att_unbound,
  .dandling = ble_att_dandling,
  .child_context_adjust = ble_att_child_context_adjust,
};

error_t ble_att_create(
  struct net_scheduler_s *scheduler,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable,
  struct net_layer_s **layer)
{
  struct ble_att_s *att = mem_alloc(sizeof(*att), mem_scope_sys);

  if (!att)
    return -ENOMEM;

  error_t err = net_layer_init(&att->layer, &att_handler, scheduler,
                               delegate, delegate_vtable);

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

static void att_req_destroy(void *mem)
{
  struct ble_att_transaction_s *task = mem;

  logk_trace("req %p destroy", mem);

  if (task->packet)
    buffer_refdec(task->packet);

  mem_free(mem);
}

struct ble_att_transaction_s *att_request_allocate(struct ble_att_s *att,
                                                   size_t total_size)
{
  struct ble_att_transaction_s *txn = mem_alloc(total_size, mem_scope_sys);
  if (!txn)
    return NULL;

  logk_trace("req %p alloc", txn);

  txn->task.destroy_func = att_req_destroy;
  txn->packet = NULL;

  return txn;
}

static void att_transaction_first_send(struct ble_att_s *att)
{
  struct buffer_s *req = NULL;
  struct net_task_s *req_task = NULL;

  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };
  struct ble_att_transaction_s *txn;

  while ((txn = ble_att_transaction_s_from_task(
                    net_task_queue_head(&att->transaction_queue)))) {
    error_t err;

    if (!att->layer.parent) {
      net_task_queue_pop(&att->transaction_queue);
      net_task_query_respond_push(&txn->task, -EPIPE);
      continue;
    }

    bool_t with_response = ble_att_opcode_is_response_expected(txn->command);

    logk_trace("%s %p %d %s response", __FUNCTION__,
            txn, txn->command, with_response ? "with" : "without");

    if (att->transaction_pending && with_response) {
      logk_trace(" A transaction is pending and ours expects a response");
      break;
    }

    net_task_queue_pop(&att->transaction_queue);

    if (!req) {
      req = net_layer_packet_alloc(&att->layer, att->layer.context.prefix_size, 0);

      if (!req) {
        net_task_query_respond_push(&txn->task, -ENOMEM);
        continue;
      }
    } else {
      req->begin = att->layer.context.prefix_size;
      req->end = req->begin;
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

    dst.unreliable = !with_response;

    logk_trace("req < %P", req->data + req->begin, req->end - req->begin);
    net_task_outbound_push(req_task, att->layer.parent, &att->layer, 0, NULL, &dst, req);
    buffer_refdec(req);

    if (with_response)
      att->transaction_pending = txn;
    else
      net_task_query_respond_push(&txn->task, 0);

    req_task = NULL;
    req = NULL;
  }

  if (req_task)
    net_task_destroy(req_task);
  if (req)
    buffer_refdec(req);
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

  logk_trace("rsp < %P", rsp->data + rsp->begin, rsp->end - rsp->begin);
  net_task_outbound_push(rsp_task, att->layer.parent, &att->layer, 0, NULL, &dst, rsp);

 error_free_buffer:
  buffer_refdec(rsp);

  return err;
}
