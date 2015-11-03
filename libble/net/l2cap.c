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
#include <ble/net/layer.h>
#include <ble/net/signalling.h>
#include <ble/net/att.h>
#include <ble/protocol/data.h>
#include <ble/protocol/l2cap.h>

#include <ble/net/generic.h>

//#define dprintk printk
#define dprintk(...) do{}while(0)

struct ble_l2cap_s;
struct ble_sm_s;
struct ble_att_s;

struct ble_l2cap_handler_s;

/**
 BLE L2CAP layer.

 Handles fragmentation and CID multiplexing.  For a compliant LE
 device, this layer expects SM and ATT layers.
 */
struct ble_l2cap_s
{
  struct net_layer_s layer;

  const struct ble_l2cap_handler_s *handler;
  struct net_layer_s *att;
  struct net_layer_s *sm;
  struct net_layer_s *signalling;
};

STRUCT_COMPOSE(ble_l2cap_s, layer);

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
    if (task->header.source == layer->parent) {
      uint16_t length = endian_le16_na_load(data);
      uint16_t cid = endian_le16_na_load(data + 2);
      struct net_layer_s *target = NULL;

      // TODO: Handle fragmentation

      dprintk("L2CAP rx ll %d, size %d, length %d, cid %d\n",
              task->inbound.dst_addr.llid, size, length, cid);

      if (task->inbound.dst_addr.llid != BLE_LL_DATA_START)
        break;

      if (size - 4 != length)
        break;

      task->inbound.buffer->begin += 4;
      task->inbound.dst_addr.cid = cid;

      switch (cid) {
      case BLE_L2CAP_CID_ATT:
        target = l2cap->att;
        break;

      case BLE_L2CAP_CID_SM:
        target = l2cap->sm;
        break;

      case BLE_L2CAP_CID_SIGNALLING:
        target = l2cap->signalling;
        break;
      }

      if (target) {
        dprintk("L2CAP %d > %P\n",
               target->handler->type,
               task->inbound.buffer->data + task->inbound.buffer->begin,
               task->inbound.buffer->end - task->inbound.buffer->begin);
        net_task_inbound_forward(task, target);
        return;
      }
    } else {
      uint16_t cid = task->inbound.dst_addr.cid;
      uint8_t header[] = {size & 0xff, size >> 8, cid & 0xff, cid >> 8};

      dprintk("L2CAP %d < %P\n",
             task->header.source->handler->type,
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

  net_task_destroy(task);
}

static
error_t ble_l2cap_bound(struct net_layer_s *layer,
                               void *addr,
                               struct net_layer_s *child)
{
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);
  uint16_t proto = *(uint16_t *)addr;

  switch (proto) {
  case BLE_L2CAP_CID_SM:
    l2cap->sm = child;
    return 0;

  case BLE_L2CAP_CID_ATT:
    l2cap->att = child;
    return 0;

  case BLE_L2CAP_CID_SIGNALLING:
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

static
void ble_l2cap_destroyed(struct net_layer_s *layer)
{
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  dprintk("L2cap %p destroyed\n", l2cap);

  mem_free(l2cap);
}

static const struct net_layer_handler_s l2cap_handler = {
  .destroyed = ble_l2cap_destroyed,
  .task_handle = ble_l2cap_task_handle,
  .bound = ble_l2cap_bound,
  .unbound = ble_l2cap_unbound,
  .context_updated = ble_l2cap_context_updated,
  .type = BLE_NET_LAYER_L2CAP,
};

error_t ble_l2cap_create(struct net_scheduler_s *scheduler,
                         void *delegate,
                         const struct net_layer_delegate_vtable_s *delegate_vtable,
                         struct net_layer_s **layer)
{
  struct ble_l2cap_s *l2cap = mem_alloc(sizeof(*l2cap), mem_scope_sys);

  if (!l2cap)
    return -ENOMEM;

  error_t err = net_layer_init(&l2cap->layer, &l2cap_handler, scheduler,
                               delegate, delegate_vtable);

  l2cap->sm = NULL;
  l2cap->att = NULL;
  l2cap->signalling = NULL;

  if (err)
    mem_free(l2cap);
  else
    *layer = &l2cap->layer;

  return err;
}
