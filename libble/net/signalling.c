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

#define LOGK_MODULE_ID "bsig"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/gap.h>
#include <ble/protocol/signalling.h>
#include <ble/protocol/l2cap.h>

#include <ble/net/generic.h>

struct ble_signalling_handler_s;

/**
 BLE L2CAP signalling layer.

 Handles optional/advanced features of BLE connections:
 - Disconnection requests,
 - Connection parameter update requests to master,
 - Credit-based connection flow control,
 - Ping.
 */
struct ble_signalling_s
{
  struct net_layer_s layer;
  struct net_task_s *pending_conn_params;
  uint8_t pending_conn_params_identifier;
  uint8_t identifier;
};

STRUCT_COMPOSE(ble_signalling_s, layer);

static
void ble_sig_destroyed(struct net_layer_s *layer)
{
  struct ble_signalling_s *sig = ble_signalling_s_from_layer(layer);

  if (sig->pending_conn_params)
    net_task_destroy(sig->pending_conn_params);

  mem_free(sig);
}

static void sig_command_handle(struct ble_signalling_s *sig, struct net_task_s *task)
{
  const uint8_t *data = task->packet.buffer->data + task->packet.buffer->begin;
  const size_t size = task->packet.buffer->end - task->packet.buffer->begin;
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SIGNALLING,
  };

  if (size < 4)
    goto destroy;

  switch (data[0]) {
  case BLE_SIGNALLING_CONN_PARAMS_UPDATE_RSP:
    if (sig->pending_conn_params
        && data[1] == sig->pending_conn_params_identifier) {
      if (size < 6)
        break;

      bool_t success = endian_le16_na_load(data + 4) == 0;

      net_task_query_respond_push(sig->pending_conn_params,
                                  success ? 0 : -EAGAIN);
      sig->pending_conn_params = NULL;
    }
    break;

  case BLE_SIGNALLING_CONN_PARAMS_UPDATE_REQ:
    task->packet.buffer->data[task->packet.buffer->begin]
      = BLE_SIGNALLING_CONN_PARAMS_UPDATE_RSP;
    endian_le16_na_store(task->packet.buffer->data + task->packet.buffer->begin + 2, 2);
    endian_le16_na_store(task->packet.buffer->data + task->packet.buffer->begin + 4, 0);
    task->packet.buffer->end = task->packet.buffer->begin + 6;
    net_task_packet_respond(task, sig->layer.parent, 0, &dst);
    return;

  default:
    break;
  }

 destroy:
  net_task_destroy(task);
}

static
uint8_t sig_pkt_send(
  struct ble_signalling_s *sig,
  struct buffer_s *pkt,
  uint8_t command)
{
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SIGNALLING,
  };

  uint16_t length = pkt->end - pkt->begin;
  uint8_t identifier = sig->identifier++;

  if (!sig->layer.parent)
    return 0;

  pkt->begin -= 4;
  pkt->data[pkt->begin] = command;
  pkt->data[pkt->begin + 1] = identifier;
  endian_le16_na_store(&pkt->data[pkt->begin + 2], length);

  struct net_task_s *task = net_scheduler_task_alloc(sig->layer.scheduler);

  if (task) {
    net_task_outbound_push(task,
                           sig->layer.parent, &sig->layer,
                           0, NULL, &dst, pkt);
  }

  return identifier;
}

static
void ble_sig_task_handle(struct net_layer_s *layer,
                         struct net_task_s *task)
{
  struct ble_signalling_s *sig = ble_signalling_s_from_layer(layer);

  switch (task->type) {
  default:
    break;

  case NET_TASK_INBOUND:
    sig_command_handle(sig, task);
    return;

  case NET_TASK_QUERY:
    logk_trace("SIG Query, %x, current %p", task->query.opcode, sig->pending_conn_params);

    switch (task->query.opcode) {
    case BLE_GAP_CONN_PARAMS_UPDATE: {
      if (sig->pending_conn_params) {
        net_task_query_respond_push(task, -EBUSY);
        return;
      }

      struct ble_gap_conn_params_update_s *up;
      struct buffer_s *pkt;

      up = ble_gap_conn_params_update_s_from_task(task);
      pkt = net_layer_packet_alloc(&sig->layer, sig->layer.context.prefix_size + 4, 8);

      endian_le16_na_store(&pkt->data[pkt->begin], up->interval_min);
      endian_le16_na_store(&pkt->data[pkt->begin + 2], up->interval_max);
      endian_le16_na_store(&pkt->data[pkt->begin + 4], up->slave_latency);
      endian_le16_na_store(&pkt->data[pkt->begin + 6], up->timeout);

      sig->pending_conn_params = task;
      sig->pending_conn_params_identifier
        = sig_pkt_send(sig, pkt, BLE_SIGNALLING_CONN_PARAMS_UPDATE_REQ);

      buffer_refdec(pkt);

      // Dont destroy task
      return;
    }
    }

    break;
  }

  net_task_destroy(task);
}

static
void ble_sig_dandling(struct net_layer_s *layer)
{
  struct ble_signalling_s *sig = ble_signalling_s_from_layer(layer);

  if (sig->pending_conn_params) {
    net_task_query_respond_push(sig->pending_conn_params, -EIO);
    sig->pending_conn_params = NULL;
  }
}

static const struct net_layer_handler_s sig_handler = {
  .destroyed = ble_sig_destroyed,
  .task_handle = ble_sig_task_handle,
  .dandling = ble_sig_dandling,
};

error_t ble_signalling_create(struct net_scheduler_s *scheduler,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer)
{
 struct ble_signalling_s *sig = mem_alloc(sizeof(*sig), mem_scope_sys);

  if (!sig)
    return -ENOMEM;

  memset(sig, 0, sizeof(*sig));

  error_t err = net_layer_init(&sig->layer, &sig_handler, scheduler,
                               delegate, delegate_vtable);
  if (err)
    mem_free(sig);
  else
    *layer = &sig->layer;

  return err;
}

