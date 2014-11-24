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

#include <ble/net/signalling.h>
#include <ble/protocol/signalling.h>
#include <ble/protocol/l2cap.h>

static
void ble_sig_destroyed(struct net_layer_s *layer)
{
  struct ble_signalling_s *sig = ble_signalling_s_from_layer(layer);

  sig->handler->destroyed(sig);
}

static void sig_command_handle(struct ble_signalling_s *sig, struct net_task_s *task)
{
  const uint8_t *data = task->inbound.buffer->data + task->inbound.buffer->begin;
  const size_t size = task->inbound.buffer->end - task->inbound.buffer->begin;

  if (size < 4)
    return;

  if (sig->pending_conn_params && data[1] == sig->pending_conn_params_identifier) {
    if (size < 6)
      return;

    bool_t success = data[0] == BLE_SIGNALLING_CONN_PARAMS_UPDATE_RSP
      && endian_le16_na_load(data + 4) == 0;

    net_task_query_respond_push(sig->pending_conn_params,
                                success ? 0 : -EAGAIN);
    sig->pending_conn_params = NULL;
    return;
  }
}

void sig_pkt_send(
  struct ble_signalling_s *sig,
  struct buffer_s *pkt,
  uint8_t command)
{
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_SIGNALLING,
    .reliable = 1,
  };

  uint16_t length = pkt->end - pkt->begin;

  if (!sig->layer.parent)
    return;

  pkt->begin -= 4;
  pkt->data[pkt->begin] = BLE_SIGNALLING_CONN_PARAMS_UPDATE_REQ;
  pkt->data[pkt->begin + 1] = sig->identifier++;
  endian_le16_na_store(&pkt->data[pkt->begin + 2], length);

  net_task_inbound_push(net_scheduler_task_alloc(sig->layer.scheduler),
                        sig->layer.parent, &sig->layer,
                        0, NULL, &dst, pkt);
}

static
void ble_sig_task_handle(struct net_layer_s *layer,
                         struct net_task_header_s *header)
{
  struct ble_signalling_s *sig = ble_signalling_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  switch (header->type) {
  default:
    break;

  case NET_TASK_INBOUND:
    if (task->header.source == layer->parent)
      sig_command_handle(sig, task);
    break;

  case NET_TASK_QUERY:
    switch (task->query.opcode) {
    case BLE_SIG_CONN_PARAMS_UPDATE: {
      if (sig->pending_conn_params) {
        net_task_query_respond_push(task, -EBUSY);
        return;
      }

      struct ble_signalling_conn_params_update_task_s *up
        = ble_signalling_conn_params_update_task_s_from_task(task);
      struct buffer_s *pkt = net_layer_packet_alloc(&sig->layer, sig->layer.context.prefix_size + 4, 8);

      sig->pending_conn_params = task;
      sig->pending_conn_params_identifier = sig->identifier;

      endian_le16_na_store(&pkt->data[pkt->begin], up->interval_min);
      endian_le16_na_store(&pkt->data[pkt->begin + 2], up->interval_max);
      endian_le16_na_store(&pkt->data[pkt->begin + 4], up->slave_latency);
      endian_le16_na_store(&pkt->data[pkt->begin + 6], up->timeout);

      sig_pkt_send(sig, pkt, BLE_SIGNALLING_CONN_PARAMS_UPDATE_REQ);

      buffer_refdec(pkt);

      // Dont destroy task
      return;
    }
    }

    break;
  }

  net_task_cleanup(task);
}

static const struct net_layer_handler_s sig_handler = {
  .destroyed = ble_sig_destroyed,
  .task_handle = ble_sig_task_handle,
  .type = BLE_LAYER_TYPE_SIGNALLING,
};

error_t ble_signalling_init(
  struct ble_signalling_s *sig,
  const struct ble_signalling_handler_s *handler,
  struct net_scheduler_s *scheduler)
{
  sig->handler = handler;

  return net_layer_init(&sig->layer, &sig_handler, scheduler);
}
