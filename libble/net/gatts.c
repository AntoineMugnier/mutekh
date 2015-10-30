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

#include <ble/net/gatts.h>
#include <ble/net/att.h>
#include <ble/net/layer.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>
#include <ble/protocol/l2cap.h>
#include <ble/protocol/gatt.h>

#include <ble/gatt/db.h>
#include <ble/security_db.h>
#include <ble/gatt/client.h>

#include <ble/net/generic.h>

//#define dprintk(...) do{}while(0)
#define dprintk printk

struct ble_gatts_s
{
  struct net_layer_s layer;
  struct ble_gatt_client_s client;
  struct ble_peer_s *peer;
  struct net_task_s *delayed_client_update;
};

STRUCT_COMPOSE(ble_gatts_s, layer);
STRUCT_COMPOSE(ble_gatts_s, client);

error_t gatts_find_information(struct ble_gatts_s *gatt,
                               struct ble_att_find_information_task_s *task);
error_t gatts_find_by_type_value(struct ble_gatts_s *gatt,
                                 struct ble_att_find_by_type_value_task_s *task);
error_t gatts_read_by_type(struct ble_gatts_s *gatt,
                           struct ble_att_read_by_type_task_s *task);
error_t gatts_read(struct ble_gatts_s *gatt,
                   struct ble_att_read_task_s *task);
error_t gatts_read_multiple(struct ble_gatts_s *gatt,
                            struct ble_att_read_multiple_task_s *task);
error_t gatts_read_by_group_type(struct ble_gatts_s *gatt,
                                 struct ble_att_read_by_group_type_task_s *task);
error_t gatts_write(struct ble_gatts_s *gatt,
                    struct ble_att_write_task_s *task);

static
void ble_gatts_destroyed(struct net_layer_s *layer)
{
  struct ble_gatts_s *gatt = ble_gatts_s_from_layer(layer);

  ble_gatt_client_db_close(&gatt->client);

  mem_free(gatt);
}

static void gatts_query_handle(struct ble_gatts_s *gatt, struct ble_att_transaction_s *txn)
{
  uint8_t err;

  switch (txn->command) {
  case BLE_ATT_FIND_INFORMATION_RQT:
    err = gatts_find_information(gatt, ble_att_find_information_task_s_from_base(txn));
    break;

  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT:
    err = gatts_find_by_type_value(gatt, ble_att_find_by_type_value_task_s_from_base(txn));
    break;

  case BLE_ATT_READ_BY_TYPE_RQT:
    err = gatts_read_by_type(gatt, ble_att_read_by_type_task_s_from_base(txn));
    break;

  case BLE_ATT_READ_RQT:
  case BLE_ATT_READ_BLOB_RQT:
    err = gatts_read(gatt, ble_att_read_task_s_from_base(txn));
    break;

  case BLE_ATT_READ_MULTIPLE_RQT:
    err = gatts_read_multiple(gatt, ble_att_read_multiple_task_s_from_base(txn));
    break;

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT:
    err = gatts_read_by_group_type(gatt, ble_att_read_by_group_type_task_s_from_base(txn));
    break;

  case BLE_ATT_WRITE_RQT:
  case BLE_ATT_WRITE_CMD:
    err = gatts_write(gatt, ble_att_write_task_s_from_base(txn));
    break;

  default:
    err = -EINVAL;
    break;
  }

  txn->error_handle = ble_gatt_client_tell(&gatt->client);

  net_task_query_respond_push(&txn->task, err);
}

static
void ble_gatts_task_handle(struct net_layer_s *layer,
                         struct net_task_header_s *header)
{
  struct ble_gatts_s *gatt = ble_gatts_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  switch (header->type) {
  case NET_TASK_QUERY:
    if (task->header.source == layer->parent) {
      gatts_query_handle(gatt, ble_att_transaction_s_from_task(task));
      return;
    }
    break;

  case NET_TASK_RESPONSE:
    break;

  case NET_TASK_TIMEOUT: {
    if (task == gatt->delayed_client_update) {
      gatt->delayed_client_update = NULL;

      struct ble_subscription_s subscriptions[BLE_SUBSCRIBED_CHAR_COUNT];

      ble_gatt_client_subscription_get(&gatt->client, subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);
      ble_peer_subscriptions_set(gatt->peer, subscriptions);
#if defined(CONFIG_BLE_CRYPTO)
      ble_peer_save(gatt->peer);
#endif
      break;
    }
    break;
  }

  default:
    break;
  }

  net_task_destroy(task);
}

static
void ble_gatts_att_value_changed(struct ble_gatt_client_s *client,
                                uint16_t value_handle, uint16_t mode,
                                const void *data, size_t size)
{
  struct ble_gatts_s *gatt = ble_gatts_s_from_client(client);

  if (!gatt->layer.parent)
    return;

  struct buffer_s *pkt = net_layer_packet_alloc(&gatt->layer,
                                                gatt->layer.context.prefix_size,
                                                0);
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };

  size = __MIN(gatt->layer.context.mtu - 3, size);

  pkt->end = pkt->begin + size + 3;

  pkt->data[pkt->begin] = mode & BLE_GATT_CCCD_NOTIFICATION
    ? BLE_ATT_HANDLE_VALUE_NOTIF
    : BLE_ATT_HANDLE_VALUE_INDIC;

  dst.unreliable = pkt->data[pkt->begin] == BLE_ATT_HANDLE_VALUE_NOTIF;

  endian_le16_na_store(pkt->data + pkt->begin + 1, value_handle);

  memcpy(pkt->data + pkt->begin + 3, data, size);

  net_task_inbound_push(net_scheduler_task_alloc(gatt->layer.scheduler),
                        gatt->layer.parent, &gatt->layer,
                        0, NULL, &dst, pkt);

  dprintk("Gatt Notif/Indic %02x (%d) %P\n", pkt->data[pkt->begin],
         pkt->end - pkt->begin,
         &pkt->data[pkt->begin + 1],
         pkt->end - pkt->begin - 1);

  buffer_refdec(pkt);
}

static void gatts_save_peer_later(struct ble_gatts_s *gatt)
{
  if (gatt->delayed_client_update)
    net_scheduler_task_cancel(gatt->layer.scheduler,
                              &gatt->delayed_client_update->header);

  struct net_task_s *timeout = net_scheduler_task_alloc(gatt->layer.scheduler);
  dev_timer_delay_t ticks;
  dev_timer_init_sec(&gatt->layer.scheduler->timer, &ticks, NULL, 2, 1);
  net_task_timeout_push(timeout, &gatt->layer,
                        net_scheduler_time_get(gatt->layer.scheduler) + ticks, 0);
  gatt->delayed_client_update = timeout;
}

static
void ble_gatts_att_subscription_changed(struct ble_gatt_client_s *client)
{
  struct ble_gatts_s *gatt = ble_gatts_s_from_client(client);

  gatts_save_peer_later(gatt);
}

static bool_t ble_gatts_context_updated(
    struct net_layer_s *layer,
    const struct net_layer_context_s *parent_context)
{
  struct ble_gatts_s *gatt = ble_gatts_s_from_layer(layer);

  gatt->layer.context = *parent_context;
  gatt->client.encrypted = parent_context->addr.encrypted;
  gatt->client.authenticated = parent_context->addr.authenticated;

  dprintk("Gatt client layer now %s\n", gatt->client.encrypted ? "encrypted" : "clear text");

  ble_gatt_client_subscription_set(&gatt->client, gatt->peer->subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);

  return 1;
}

static const struct net_layer_handler_s gatts_handler = {
  .destroyed = ble_gatts_destroyed,
  .task_handle = ble_gatts_task_handle,
  .context_updated = ble_gatts_context_updated,
  .type = BLE_NET_LAYER_GATT,
};

static const struct ble_gatt_client_handler_s gatts_db_handler = {
  .att_value_changed = ble_gatts_att_value_changed,
  .att_subscription_changed = ble_gatts_att_subscription_changed,
};

error_t gatts_find_information(struct ble_gatts_s *gatt,
                               struct ble_att_find_information_task_s *task)
{
  uint8_t err;
  bool_t once = 0;

  dprintk("Find information %d-%d\n",
          task->start, task->end);

  for (err = ble_gatt_client_seek(&gatt->client, task->start);
       err == 0
         && task->information_count < task->information_max_count
         && ble_gatt_client_tell(&gatt->client) <= task->end;
       err = ble_gatt_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    struct ble_att_information_s *ai = task->information + task->information_count;
    uint16_t handle = ble_gatt_client_tell(&gatt->client);

    err = ble_gatt_client_type_get(&gatt->client, &type);
    if (err)
      break;

    dprintk("  %d " BLE_UUID_FMT "\n", handle, BLE_UUID_ARG(type));

    ai->handle = handle;
    ai->type = *type;

    task->information_count++;
    once = 1;
  }

  if (!once)
    task->base.error = err;

  return 0;
}

error_t gatts_find_by_type_value(struct ble_gatts_s *gatt,
                                 struct ble_att_find_by_type_value_task_s *task)
{
  uint8_t err;
  uint8_t *tmp_data = alloca(task->value_size + 1);
  bool_t once = 0;

  dprintk("Find by type value %d-%d " BLE_UUID_FMT " %P\n",
          task->start, task->end,
          BLE_UUID_ARG(&task->type),
          task->value, task->value_size);

  for (err = ble_gatt_client_seek(&gatt->client, task->start);
       err == 0
         && task->information_count < task->information_max_count
         && ble_gatt_client_tell(&gatt->client) <= task->end;
       err = ble_gatt_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    struct ble_att_handle_information_s *hi = task->information + task->information_count;

    err = ble_gatt_client_type_get(&gatt->client, &type);
    if (err)
      break;

    if (!ble_uuid_is_uuid16(type) || ble_uuid_cmp(&task->type, type))
      continue;

    size_t rsize = task->value_size + 1;

    err = ble_gatt_client_read(&gatt->client, 0, tmp_data, &rsize);
    if (err || task->value_size != rsize || memcmp(tmp_data, task->value, task->value_size))
      continue;

    uint16_t handle = ble_gatt_client_tell(&gatt->client);
    uint16_t end_group = gatt->client.cursor.service->start_handle
      + gatt->client.cursor.service->handle_count - 1;

    dprintk(" from %d to %d\n", handle, end_group);

    hi->found = handle;
    hi->end_group = end_group;

    task->information_count++;
    once = 1;
  }

  if (!once)
    task->base.error = err;

  return 0;
}

error_t gatts_read_by_type(struct ble_gatts_s *gatt,
                           struct ble_att_read_by_type_task_s *task)
{
  bool_t once = 0;
  uint8_t err;

  dprintk("Read by type %d-%d " BLE_UUID_FMT "\n",
          task->start, task->end,
          BLE_UUID_ARG(&task->type));

  for (err = ble_gatt_client_seek(&gatt->client, task->start);
       err == 0
         && task->handle_value_size < task->handle_value_size_max
         && ble_gatt_client_tell(&gatt->client) <= task->end;
       err = ble_gatt_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    uint16_t handle = ble_gatt_client_tell(&gatt->client);
    err = ble_gatt_client_type_get(&gatt->client, &type);

    /* dprintk(" % 4d err %d: " BLE_UUID_FMT "\n", */
    /*         handle, err, BLE_UUID_ARG(type)); */

    if (err)
      break;

    if (ble_uuid_cmp(type, &task->type))
      continue;

    size_t written;

    if (task->handle_value_stride)
      written = task->handle_value_stride - 2;
    else
      written = task->handle_value_size_max - 2;

    endian_16_na_store(task->handle_value + task->handle_value_size, handle);
    err = ble_gatt_client_read(&gatt->client, 0,
                               (uint8_t *)task->handle_value + task->handle_value_size + 2,
                               &written);

    dprintk(" %d: %P\n",
            ble_gatt_client_tell(&gatt->client),
            (uint8_t *)task->handle_value + task->handle_value_size + 2,
            written);

    if (!task->handle_value_stride)
      task->handle_value_stride = written + 2;

    if (err == BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN
        || task->handle_value_stride != written + 2) {
      if (once)
        return 0;
      break;
    }

    once = 1;

    task->handle_value_size += task->handle_value_stride;
  }

  task->base.error = once ? 0 : BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;

  return 0;
}

error_t gatts_read(struct ble_gatts_s *gatt,
                   struct ble_att_read_task_s *task)
{
  size_t written;

  dprintk("Read %d\n", task->handle);

  task->base.error = ble_gatt_client_seek(&gatt->client, task->handle);
  if (task->base.error)
    return 0;

  written = task->value_size_max - task->offset;
  task->base.error = ble_gatt_client_read(&gatt->client, task->offset, task->value, &written);
  if (task->base.error)
    return 0;

  task->value_size = task->offset + written;

  if (written == 0 && task->offset && !task->base.error)
    task->base.error = BLE_ATT_ERR_ATTRIBUTE_NOT_LONG;

  return 0;
}

error_t gatts_read_multiple(struct ble_gatts_s *gatt,
                            struct ble_att_read_multiple_task_s *task)
{
  size_t written;

  dprintk("Read multiple", __FUNCTION__);

  for (size_t i = 0; i < task->handle_count && task->buffer_size < task->buffer_size_max; ++i) {
    dprintk(" %d", task->handle[i]);
    task->base.error = ble_gatt_client_seek(&gatt->client, task->handle[i]);
    if (task->base.error)
      return 0;

    written = task->buffer_size_max - task->buffer_size;
    task->base.error = ble_gatt_client_read(&gatt->client, 0,
                                       (uint8_t*)task->buffer + task->buffer_size, &written);
    if (task->base.error)
      return 0;

    task->buffer_size += written;
  }

  dprintk("\n");

  return 0;
}

error_t gatts_read_by_group_type(struct ble_gatts_s *gatt,
                                 struct ble_att_read_by_group_type_task_s *task)
{
  uint8_t err;
  bool_t once = 0;

  dprintk("Read by group type %d-%d " BLE_UUID_FMT "\n",
          task->start, task->end, BLE_UUID_ARG(&task->type));

  task->base.error = 0;

  for (err = ble_gatt_client_seek(&gatt->client, task->start);
       err == 0
         && (uint16_t)(task->attribute_data_size_max - task->attribute_data_size) > 4
         && ble_gatt_client_tell(&gatt->client) <= task->end;
       err = ble_gatt_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    uint16_t handle = ble_gatt_client_tell(&gatt->client);

    err = ble_gatt_client_type_get(&gatt->client, &type);

    if (err) {
      if (!once)
        task->base.error = err;
      break;
    }

    if (ble_uuid_cmp(type, &task->type))
      continue;

    size_t written = task->attribute_data_size_max - task->attribute_data_size - 4;

    if (task->attribute_data_stride)
      written = __MIN(task->attribute_data_stride, written);

    struct ble_att_data_s *ad = (void*)((uint8_t*)task->attribute_data
                                        + task->attribute_data_size);

    err = ble_gatt_client_read(&gatt->client, 0, ad->value, &written);
    if (err) {
      if (!once)
        task->base.error = err;
      break;
    }

    endian_16_na_store(&ad->handle, handle);
    endian_16_na_store(&ad->end_group, gatt->client.cursor.service->start_handle
                       + gatt->client.cursor.service->handle_count
                       - 1);

    dprintk(" %d - %d %P\n",
            handle, gatt->client.cursor.service->start_handle
            + gatt->client.cursor.service->handle_count
            - 1, ad->value, written);

    if (!task->attribute_data_stride)
      task->attribute_data_stride = written + 4;

    task->attribute_data_size += task->attribute_data_stride;
  }

  return 0;
}

error_t gatts_write(struct ble_gatts_s *gatt,
                    struct ble_att_write_task_s *task)
{
  dprintk("Write %d %P\n", task->handle, task->value, task->value_size);

  task->base.error = ble_gatt_client_seek(&gatt->client, task->handle);
  if (task->base.error)
    return 0;

  task->base.error = ble_gatt_client_write(&gatt->client, task->value, task->value_size);
  return 0;
}

error_t ble_gatts_create(struct net_scheduler_s *scheduler,
                        const void *params_,
                        struct net_layer_s **layer)
{
  struct ble_gatts_s *gatt = mem_alloc(sizeof(*gatt), mem_scope_sys);
  const struct ble_gatts_params_s *params = params_;

  if (!gatt)
    return -ENOMEM;

  error_t err = net_layer_init(&gatt->layer, &gatts_handler, scheduler, NULL, NULL);
  if (err) {
    mem_free(gatt);
    return err;
  }

  gatt->peer = params->peer;
  gatt->delayed_client_update = NULL;

  ble_gatt_client_db_open(&gatt->client, &gatts_db_handler, params->db);
  ble_gatt_client_subscription_set(&gatt->client, gatt->peer->subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);

  *layer = &gatt->layer;

  return 0;
}
