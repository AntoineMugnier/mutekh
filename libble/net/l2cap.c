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

#define LOGK_MODULE_ID "l2cp"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/sm.h>
#include <ble/net/att.h>
#include <ble/protocol/data.h>
#include <ble/protocol/l2cap.h>

#include <ble/net/generic.h>

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
                           struct net_task_s *task)
{
  const uint8_t *data = task->packet.buffer->data + task->packet.buffer->begin;
  const size_t size = task->packet.buffer->end - task->packet.buffer->begin;
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_INBOUND: {
    assert(task->packet.buffer);

    uint16_t length = endian_le16_na_load(data);
    uint16_t cid = endian_le16_na_load(data + 2);
    struct net_layer_s *target = NULL;

    // TODO: Handle fragmentation

    logk_trace("L2CAP rx ll %d, size %d, length %d, cid %d",
               task->packet.dst_addr.llid, size, length, cid);

    if (task->packet.dst_addr.llid != BLE_LL_DATA_START)
      break;

    if (size - 4 != length)
      break;

    task->packet.buffer->begin += 4;
    task->packet.dst_addr.cid = cid;

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
      logk_trace("L2CAP %d > %P",
              cid,
              task->packet.buffer->data + task->packet.buffer->begin,
              task->packet.buffer->end - task->packet.buffer->begin);
      net_task_packet_forward(task, target);
      return;
    }

    break;
  }

  case NET_TASK_OUTBOUND: {
    assert(task->packet.buffer);

    uint16_t cid = task->packet.dst_addr.cid;
    uint8_t header[] = {size & 0xff, size >> 8, cid & 0xff, cid >> 8};

    logk_trace("L2CAP %d < %P",
            cid,
            task->packet.buffer->data + task->packet.buffer->begin,
            task->packet.buffer->end - task->packet.buffer->begin);

    buffer_prepend(task->packet.buffer, header, 4);
    task->packet.dst_addr.llid = BLE_LL_DATA_START;

    if (layer->parent) {
      net_task_packet_forward(task, layer->parent);
      return;
    }
    break;
  }

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
void ble_l2cap_child_context_adjust(const struct net_layer_s *layer,
                                    struct net_layer_context_s *cc)
{
  cc->prefix_size += 4;
  cc->mtu -= 4;
}

static
void ble_l2cap_destroyed(struct net_layer_s *layer)
{
  struct ble_l2cap_s *l2cap = ble_l2cap_s_from_layer(layer);

  logk_debug("L2cap %p destroyed", l2cap);

  mem_free(l2cap);
}

static const struct net_layer_handler_s l2cap_handler = {
  .destroyed = ble_l2cap_destroyed,
  .task_handle = ble_l2cap_task_handle,
  .bound = ble_l2cap_bound,
  .unbound = ble_l2cap_unbound,
  .child_context_adjust = ble_l2cap_child_context_adjust,
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
