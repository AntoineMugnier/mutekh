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

#include <ble/net/l2cap.h>
#include <ble/net/sm.h>
#include <ble/net/signalling.h>
#include <ble/net/att.h>
#include <ble/protocol/data.h>
#include <ble/protocol/l2cap.h>

//#define dprintk printk
#define dprintk(...) do{}while(0)

static
void ble_l2cap_destroyed(struct net_layer_s *layer)
{
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  l2cap->handler->destroyed(l2cap);
}

static
void ble_l2cap_task_handle(struct net_layer_s *layer,
                           struct net_task_header_s *header)
{
  struct net_task_s *task = net_task_s_from_header(header);
  const uint8_t *data = task->inbound.buffer->data + task->inbound.buffer->begin;
  const size_t size = task->inbound.buffer->end - task->inbound.buffer->begin;
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  switch (header->type) {
  case NET_TASK_INBOUND:
    dprintk("L2CAP inbound from %S %p parent %p\n",
           &task->inbound.source->handler->type, 4,
           task->inbound.source, layer->parent);

    if (task->header.source == layer->parent) {
      uint16_t length = endian_le16_na_load(data);
      uint16_t cid = endian_le16_na_load(data + 2);

      // TODO: Handle fragmentation

      dprintk("L2CAP rx ll %d, size %d, length %d\n",
             task->inbound.dst_addr.llid, size, length);

      if (task->inbound.dst_addr.llid != BLE_LL_DATA_START)
        break;

      if (size - 4 != length)
        break;

      task->inbound.buffer->begin += 4;
      task->inbound.dst_addr.cid = cid;

      switch (cid) {
      case BLE_L2CAP_CID_ATT:
        if (l2cap->att) {
          net_task_inbound_forward(task, l2cap->att);
          return;
        }
        break;

      case BLE_L2CAP_CID_SM:
        if (l2cap->sm) {
          net_task_inbound_forward(task, l2cap->sm);
          return;
        }
        break;

      case BLE_L2CAP_CID_SIGNALLING:
        if (l2cap->signalling) {
          net_task_inbound_forward(task, l2cap->signalling);
          return;
        }
        break;
      }
    } else {
      uint16_t cid = task->inbound.dst_addr.cid;
      uint8_t header[] = {size & 0xff, size >> 8, cid & 0xff, cid >> 8};

      dprintk("l2cap output packet cid %d off %d %P\n", cid,
              task->inbound.buffer->begin,
              task->inbound.buffer->data + task->inbound.buffer->begin,
              task->inbound.buffer->end - task->inbound.buffer->begin);

      buffer_prepend(task->inbound.buffer, header, 4);
      task->inbound.dst_addr.llid = BLE_LL_DATA_START;

      if (layer->parent) {
        net_task_inbound_forward(task, layer->parent);
        return;
      }
    }
    break;

  default:
    break;
  }

  net_task_cleanup(task);
}

static
error_t ble_l2cap_bound(struct net_layer_s *layer,
                               void *addr,
                               struct net_layer_s *child)
{
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  switch (child->handler->type) {
  case BLE_LAYER_TYPE_SM:
    l2cap->sm = child;
    return 0;

  case BLE_LAYER_TYPE_ATT:
    l2cap->att = child;
    return 0;

  case BLE_LAYER_TYPE_SIGNALLING:
    l2cap->signalling = child;
    return 0;
  }

  return -EINVAL;
}

static
void ble_l2cap_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  if (child == l2cap->sm)
    l2cap->sm = NULL;

  if (child == l2cap->att)
    l2cap->att = NULL;

  if (child == l2cap->signalling)
    l2cap->signalling = NULL;
}

static
bool_t ble_l2cap_context_updated(struct net_layer_s *layer,
                                 const struct net_layer_context_s *parent_context)
{
  layer->context.addr = parent_context->addr;
  layer->context.prefix_size = parent_context->prefix_size + 4;
  layer->context.mtu = parent_context->mtu - 4;

  return 1;
}

static const struct net_layer_handler_s l2cap_handler = {
  .destroyed = ble_l2cap_destroyed,
  .task_handle = ble_l2cap_task_handle,
  .bound = ble_l2cap_bound,
  .unbound = ble_l2cap_unbound,
  .context_updated = ble_l2cap_context_updated,
  .type = BLE_LAYER_TYPE_L2CAP,
};

error_t ble_l2cap_init(
  struct ble_l2cap_s *l2cap,
  const struct ble_l2cap_handler_s *handler,
  struct net_scheduler_s *scheduler)
{
  error_t err = net_layer_init(&l2cap->layer, &l2cap_handler, scheduler);

  l2cap->sm = NULL;
  l2cap->att = NULL;
  l2cap->signalling = NULL;
  l2cap->handler = handler;

  return err;
}
