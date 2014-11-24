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

#define dprintk(...) do{}while(0)

/**
 BLE Attribute protocol layer.
 */
struct ble_att_s
{
  struct net_layer_s layer;

  uint16_t server_mtu;
};

STRUCT_COMPOSE(ble_att_s, layer);

static void att_command_handle(struct ble_att_s *att, struct net_task_s *task)
{
  const uint8_t *data = task->inbound.buffer->data + task->inbound.buffer->begin;
  const size_t size = task->inbound.buffer->end - task->inbound.buffer->begin;
  struct buffer_s *rsp = NULL;
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };
  uint8_t err;
  uint16_t handle = 0;

  switch (data[0]) {
  case BLE_ATT_EXCHANGE_MTU_RQT:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    rsp = net_task_inbound_buffer_steal(task, att->layer.context.prefix_size, 3);

    att->server_mtu = endian_le16_na_load(data + 1);

    rsp->data[rsp->begin] = BLE_ATT_EXCHANGE_MTU_RSP;
    endian_le16_na_store(&rsp->data[rsp->begin + 1],
                         net_scheduler_packet_mtu(att->layer.scheduler) - 2 - 4 - 3);

    goto send;

  case BLE_ATT_EXCHANGE_MTU_RSP:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    att->server_mtu = endian_le16_na_load(data + 1);
    break;

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT:
    handle = endian_le16_na_load(data + 1);
    err = BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;
    goto error;

  default:
    err = BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;
    goto error;
  }

  goto out;

 error:
  assert(!rsp);
  rsp = net_task_inbound_buffer_steal(task, att->layer.context.prefix_size, 5);
  rsp->data[rsp->begin] = BLE_ATT_ERROR_RSP;
  rsp->data[rsp->begin + 1] = data[0];
  endian_le16_na_store(&rsp->data[rsp->begin + 2], handle);
  rsp->data[rsp->begin + 4] = err;

 send:
  printk("Att sending packet\n");

  net_task_inbound_push(net_scheduler_task_alloc(att->layer.scheduler),
                        att->layer.parent, &att->layer,
                        0, NULL, &dst, rsp);

 out:
  buffer_refdec(rsp);
  net_task_destroy(task);
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

  case NET_TASK_INBOUND:
    printk("Att inbound from %p (parent %p)\n", task->header.source, layer->parent);
    if (task->header.source == layer->parent) {
      att_command_handle(att, task);
      return;
    }
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

static const struct net_layer_handler_s att_handler = {
  .destroyed = ble_att_destroyed,
  .task_handle = ble_att_task_handle,
  .type = BLE_NET_LAYER_ATT,
};

static
error_t ble_att_init(
  struct ble_att_s *att,
  struct net_scheduler_s *scheduler)
{
  error_t err = net_layer_init(&att->layer, &att_handler, scheduler, NULL, NULL);

  att->server_mtu = 23;
  att->layer.context.prefix_size = 0;
  att->layer.context.mtu = 0;

  return err;
}

error_t ble_att_create(
  struct net_scheduler_s *scheduler,
  struct net_layer_s **layer)
{
  struct ble_att_s *att = mem_alloc(sizeof(*att), mem_scope_sys);

  if (!att)
    return -ENOMEM;

  error_t err = ble_att_init(att, scheduler);
  if (err)
    mem_free(att);
  else
    *layer = &att->layer;

  return err;
}
