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

#define LOGK_MODULE_ID "gatt"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/gatt.h>
#include <ble/net/att.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>
#include <ble/protocol/l2cap.h>
#include <ble/protocol/gatt.h>

#include <ble/gattdb/db.h>
#include <ble/gattdb/client.h>
#include <ble/security_db.h>

#include <ble/net/generic.h>

struct ble_gatt_s
{
  struct net_layer_s layer;
  struct ble_gattdb_client_s client;
  struct ble_peer_s *peer;
  struct net_task_s *delayed_client_update;
#if defined(CONFIG_BLE_GATTDB_STREAM)
  uint32_t stream_pending;
#endif
};

STRUCT_COMPOSE(ble_gatt_s, layer);
STRUCT_COMPOSE(ble_gatt_s, client);

error_t _gatt_write(struct ble_gatt_s *gatt, uint16_t handle, const void *data, size_t size);

error_t gatt_find_information(struct ble_gatt_s *gatt,
                               struct ble_att_transaction_s *task);
error_t gatt_find_by_type_value(struct ble_gatt_s *gatt,
                                 struct ble_att_transaction_s *task);
error_t gatt_read_by_type(struct ble_gatt_s *gatt,
                           struct ble_att_transaction_s *task);
error_t gatt_read(struct ble_gatt_s *gatt,
                   struct ble_att_transaction_s *task);
error_t gatt_read_multiple(struct ble_gatt_s *gatt,
                            struct ble_att_transaction_s *task);
error_t gatt_read_by_group_type(struct ble_gatt_s *gatt,
                                 struct ble_att_transaction_s *task);
error_t gatt_write(struct ble_gatt_s *gatt,
                    struct ble_att_transaction_s *task);

static
void ble_gatt_destroyed(struct net_layer_s *layer)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  ble_gattdb_client_close(&gatt->client);

  mem_free(gatt);
}

static void gatt_query_handle(struct ble_gatt_s *gatt, struct ble_att_transaction_s *txn)
{
  uint8_t err;

  switch (txn->command) {
  case BLE_ATT_FIND_INFORMATION_RQT:
    err = gatt_find_information(gatt, txn);
    break;

  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT:
    err = gatt_find_by_type_value(gatt, txn);
    break;

  case BLE_ATT_READ_BY_TYPE_RQT:
    err = gatt_read_by_type(gatt, txn);
    break;

  case BLE_ATT_READ_RQT:
  case BLE_ATT_READ_BLOB_RQT:
    err = gatt_read(gatt, txn);
    break;

  case BLE_ATT_READ_MULTIPLE_RQT:
    err = gatt_read_multiple(gatt, txn);
    break;

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT:
    err = gatt_read_by_group_type(gatt, txn);
    break;

  case BLE_ATT_WRITE_RQT:
  case BLE_ATT_WRITE_CMD:
    err = gatt_write(gatt, txn);
    break;

  default:
    err = -EINVAL;
    break;
  }

  txn->error_handle = ble_gattdb_client_tell(&gatt->client);

  net_task_query_respond_push(&txn->task, err);
}

static
void ble_gatt_task_handle(struct net_layer_s *layer,
                         struct net_task_s *task)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_QUERY:
    if (task->source == layer->parent) {
      gatt_query_handle(gatt, ble_att_transaction_s_from_task(task));
      return;
    }
    break;

  case NET_TASK_INBOUND:
    _gatt_write(gatt, task->packet.dst_addr.att,
                task->packet.buffer->data + task->packet.buffer->begin,
                task->packet.buffer->end - task->packet.buffer->begin);
    break;

  case NET_TASK_RESPONSE:
    break;

  case NET_TASK_TIMEOUT: {
    if (task == gatt->delayed_client_update) {
      gatt->delayed_client_update = NULL;

      struct ble_subscription_s subscriptions[BLE_SUBSCRIBED_CHAR_COUNT];

      ble_gattdb_client_subscription_get(&gatt->client, subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);
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

static void att_req_destroy(void *mem)
{
  struct ble_att_transaction_s *task = mem;

  logk_trace("Gatt txn %p destroy", mem);

  if (task->packet)
    buffer_refdec(task->packet);

  mem_free(mem);
}

static
error_t ble_gatt_att_value_changed(struct ble_gattdb_client_s *client,
                                   uint16_t value_handle, uint16_t mode,
                                   const void *data, size_t size)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_client(client);

  size = __MIN(gatt->layer.context.mtu, size);

  if (!gatt->layer.parent)
    return -EIO;

  if (mode & BLE_GATT_CCCD_INDICATION) {
    struct ble_att_transaction_s *txn;

    txn = mem_alloc(sizeof(*txn) + size, mem_scope_sys);
    if (!txn)
      return -ENOMEM;

    logk_trace("Gatt txn %p alloc", txn);

    txn->task.destroy_func = att_req_destroy;
    txn->packet = NULL;

    txn->write.value = (void*)(txn + 1);
    txn->write.value_size = size;
    txn->write.handle = value_handle;
    memcpy(txn->write.value, data, size);

    txn->write.authenticated = 0;
    txn->command = BLE_ATT_HANDLE_VALUE_INDIC;

    net_task_query_push(&txn->task, gatt->layer.parent, &gatt->layer, BLE_ATT_REQUEST);
  } else if  (mode & BLE_GATT_CCCD_NOTIFICATION) {
    struct net_task_s *task;
    struct buffer_s *buffer;

    buffer = net_layer_packet_alloc(&gatt->layer, gatt->layer.context.prefix_size, size);
    if (!buffer)
      return -ENOMEM;

    task = net_scheduler_task_alloc(gatt->layer.scheduler);
    if (!task) {
      buffer_refdec(buffer);
      return -ENOMEM;
    }

    struct net_addr_s src_addr = {
      .att = value_handle,
    };
    struct net_addr_s dst_addr = {
      .unreliable = 1,
    };

    memcpy(buffer->data + buffer->begin, data, size);

    net_task_outbound_push(task, gatt->layer.parent, &gatt->layer, 0, &src_addr, &dst_addr, buffer);
    buffer_refdec(buffer);
  } else {
    return -EINVAL;
  }

  return 0;
}

static void gatt_save_peer_later(struct ble_gatt_s *gatt)
{
  if (gatt->delayed_client_update) {
    net_scheduler_task_cancel(gatt->layer.scheduler,
                              gatt->delayed_client_update);
    gatt->delayed_client_update = NULL;
  }

  struct net_task_s *timeout = net_scheduler_task_alloc(gatt->layer.scheduler);
  if (!timeout)
    return;
  dev_timer_delay_t ticks;
  dev_timer_init_sec(&gatt->layer.scheduler->timer, &ticks, NULL, 2, 1);
  net_task_timeout_push(timeout, &gatt->layer,
                        net_scheduler_time_get(gatt->layer.scheduler) + ticks, 0);
  gatt->delayed_client_update = timeout;
}

static
void ble_gatt_att_subscription_changed(struct ble_gattdb_client_s *client)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_client(client);

  gatt_save_peer_later(gatt);
}

static void ble_gatt_context_changed(struct net_layer_s *layer)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  gatt->client.encrypted = layer->context.addr.encrypted;
  gatt->client.authenticated = layer->context.addr.authenticated;

  logk_trace("Gatt client layer now %s", gatt->client.encrypted ? "encrypted" : "clear text");

  ble_gattdb_client_subscription_set(&gatt->client, gatt->peer->subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);
}

static void ble_gatt_dangling(struct net_layer_s *layer)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  if (gatt->delayed_client_update) {
    net_scheduler_task_cancel(gatt->layer.scheduler,
                              gatt->delayed_client_update);
    gatt->delayed_client_update = NULL;
  }
}

#if defined(CONFIG_BLE_GATTDB_STREAM)
static void ble_gatt_att_stream_resume(struct ble_gattdb_client_s *client,
                                       uint16_t value_handle);

static void stream_task_free(struct net_task_s *task)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(task->source);
  struct net_scheduler_s *sched = task->source->scheduler;
  uint16_t value_handle = task->packet.src_addr.att;

  gatt->stream_pending--;

  slab_free(&sched->task_pool, task);
  ble_gatt_att_stream_resume(&gatt->client, value_handle);
}

static void ble_gatt_att_stream_resume(struct ble_gattdb_client_s *client,
                                       uint16_t value_handle)
{
  struct net_task_s *task;
  struct buffer_s *buffer;
  error_t err;
  struct net_addr_s src_addr = {
    .att = value_handle,
  };
  struct net_addr_s dst_addr = {
  };

  struct ble_gatt_s *gatt = ble_gatt_s_from_client(client);

  if (gatt->stream_pending > 4)
    return;

  gatt->stream_pending++;

  buffer = net_layer_packet_alloc(&gatt->layer,
                                  gatt->layer.context.prefix_size,
                                  gatt->layer.context.mtu);
  if (!buffer)
    return;

  task = slab_alloc(&gatt->layer.scheduler->task_pool);
  if (!task) {
    buffer_refdec(buffer);
    return;
  }

  task->destroy_func = (void*)stream_task_free;

  err = ble_gattdb_client_att_stream_get(client, value_handle, buffer);
  if (!err) {
    net_task_outbound_push(task, gatt->layer.parent, &gatt->layer,
                           0, &src_addr, &dst_addr, buffer);
  } else {
    slab_free(&gatt->layer.scheduler->task_pool, task);
    gatt->stream_pending--;
  }
  buffer_refdec(buffer);
}
#endif

static const struct net_layer_handler_s gatt_handler = {
  .destroyed = ble_gatt_destroyed,
  .task_handle = ble_gatt_task_handle,
  .context_changed = ble_gatt_context_changed,
  .dangling = ble_gatt_dangling,
};

static const struct ble_gattdb_client_handler_s gatt_db_handler = {
  .att_value_changed = ble_gatt_att_value_changed,
#if defined(CONFIG_BLE_GATTDB_STREAM)
  .att_stream_resume = ble_gatt_att_stream_resume,
#endif
  .att_subscription_changed = ble_gatt_att_subscription_changed,
};

error_t gatt_find_information(struct ble_gatt_s *gatt,
                               struct ble_att_transaction_s *task)
{
  uint8_t err;
  bool_t once = 0;

  logk_trace("Find information %d-%d",
          task->find_information.start, task->find_information.end);

  for (err = ble_gattdb_client_seek(&gatt->client, task->find_information.start);
       err == 0
         && task->find_information.information_count < task->find_information.information_max_count
         && ble_gattdb_client_tell(&gatt->client) <= task->find_information.end;
       err = ble_gattdb_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    struct ble_att_information_s *ai = task->find_information.information + task->find_information.information_count;
    uint16_t handle = ble_gattdb_client_tell(&gatt->client);

    err = ble_gattdb_client_type_get(&gatt->client, &type);
    if (err)
      break;

    logk_trace("  %d " BLE_UUID_FMT "", handle, BLE_UUID_ARG(type));

    ai->handle = handle;
    ai->type = *type;

    task->find_information.information_count++;
    once = 1;
  }

  if (!once)
    task->error = err;

  return 0;
}

error_t gatt_find_by_type_value(struct ble_gatt_s *gatt,
                                 struct ble_att_transaction_s *task)
{
  uint8_t err;
  uint8_t *tmp_data = alloca(task->find_by_type_value.value_size + 1);
  bool_t once = 0;

  logk_trace("Find by type value %d-%d " BLE_UUID_FMT " %P",
          task->find_by_type_value.start, task->find_by_type_value.end,
          BLE_UUID_ARG(&task->find_by_type_value.type),
          task->find_by_type_value.value, task->find_by_type_value.value_size);

  for (err = ble_gattdb_client_seek(&gatt->client, task->find_by_type_value.start);
       err == 0
         && task->find_by_type_value.information_count < task->find_by_type_value.information_max_count
         && ble_gattdb_client_tell(&gatt->client) <= task->find_by_type_value.end;
       err = ble_gattdb_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    struct ble_att_handle_information_s *hi = task->find_by_type_value.information + task->find_by_type_value.information_count;

    err = ble_gattdb_client_type_get(&gatt->client, &type);
    if (err)
      break;

    if (!ble_uuid_is_uuid16(type) || ble_uuid_cmp(&task->find_by_type_value.type, type))
      continue;

    size_t rsize = task->find_by_type_value.value_size + 1;

    err = ble_gattdb_client_read(&gatt->client, 0, tmp_data, &rsize);
    if (err || task->find_by_type_value.value_size != rsize || memcmp(tmp_data, task->find_by_type_value.value, task->find_by_type_value.value_size))
      continue;

    uint16_t handle = ble_gattdb_client_tell(&gatt->client);
    uint16_t end_group = gatt->client.cursor.registry->start_handle
      + gatt->client.cursor.registry->handle_count - 1;

    logk_trace(" from %d to %d", handle, end_group);

    hi->found = handle;
    hi->end_group = end_group;

    task->find_by_type_value.information_count++;
    once = 1;
  }

  if (!once)
    task->error = err;

  return 0;
}

error_t gatt_read_by_type(struct ble_gatt_s *gatt,
                           struct ble_att_transaction_s *task)
{
  bool_t once = 0;
  uint8_t err;

  logk_trace("Read by type %d-%d " BLE_UUID_FMT "",
          task->read_by_type.start, task->read_by_type.end,
          BLE_UUID_ARG(&task->read_by_type.type));

  for (err = ble_gattdb_client_seek(&gatt->client, task->read_by_type.start);
       err == 0
         && task->read_by_type.handle_value_size < task->read_by_type.handle_value_size_max
         && ble_gattdb_client_tell(&gatt->client) <= task->read_by_type.end;
       err = ble_gattdb_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    uint16_t handle = ble_gattdb_client_tell(&gatt->client);
    err = ble_gattdb_client_type_get(&gatt->client, &type);

    logk_trace(" % 4d err %d: " BLE_UUID_FMT "",
            handle, err, BLE_UUID_ARG(type));

    if (err)
      break;

    if (ble_uuid_cmp(type, &task->read_by_type.type))
      continue;

    size_t written;

    if (task->read_by_type.handle_value_stride) {
      written = task->read_by_type.handle_value_stride - 2;
      if (task->read_by_type.handle_value_size + task->read_by_type.handle_value_stride > task->read_by_type.handle_value_size_max)
        break;
    } else {
      written = task->read_by_type.handle_value_size_max - 2;
    }

    endian_16_na_store((uint8_t *)task->read_by_type.handle_value + task->read_by_type.handle_value_size, handle);
    err = ble_gattdb_client_read(&gatt->client, 0,
                               (uint8_t *)task->read_by_type.handle_value + task->read_by_type.handle_value_size + 2,
                               &written);

    logk_trace(" %d: %P",
            ble_gattdb_client_tell(&gatt->client),
            (uint8_t *)task->read_by_type.handle_value + task->read_by_type.handle_value_size + 2,
            written);

    if (!task->read_by_type.handle_value_stride)
      task->read_by_type.handle_value_stride = written + 2;

    if (err == BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN
        || task->read_by_type.handle_value_stride != written + 2) {
      if (once)
        return 0;
      break;
    }

    once = 1;

    task->read_by_type.handle_value_size += task->read_by_type.handle_value_stride;
  }

  task->error = once ? 0 : BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;

  return 0;
}

error_t gatt_read(struct ble_gatt_s *gatt,
                   struct ble_att_transaction_s *task)
{
  size_t written;

  logk_trace("Read %d", task->read.handle);

  task->error = ble_gattdb_client_seek(&gatt->client, task->read.handle);
  if (task->error)
    return 0;

  written = task->read.value_size_max - task->read.offset;
  task->error = ble_gattdb_client_read(&gatt->client, task->read.offset, task->read.value + task->read.offset, &written);
  if (task->error)
    return 0;

  task->read.value_size = task->read.offset + written;

  if (written == 0 && task->read.offset && !task->error)
    task->error = BLE_ATT_ERR_INVALID_OFFSET;

  return 0;
}

error_t gatt_read_multiple(struct ble_gatt_s *gatt,
                            struct ble_att_transaction_s *task)
{
  size_t written;

  logk_trace("Read multiple", __FUNCTION__);

  for (size_t i = 0; i < task->read_multiple.handle_count && task->read_multiple.buffer_size < task->read_multiple.buffer_size_max; ++i) {
    logk_trace(" %d", task->read_multiple.handle[i]);
    task->error = ble_gattdb_client_seek(&gatt->client, task->read_multiple.handle[i]);
    if (task->error)
      return 0;

    written = task->read_multiple.buffer_size_max - task->read_multiple.buffer_size;
    task->error = ble_gattdb_client_read(&gatt->client, 0,
                                       (uint8_t*)task->read_multiple.buffer + task->read_multiple.buffer_size, &written);
    if (task->error)
      return 0;

    task->read_multiple.buffer_size += written;
  }

  logk_trace("");

  return 0;
}

error_t gatt_read_by_group_type(struct ble_gatt_s *gatt,
                                 struct ble_att_transaction_s *task)
{
  uint8_t err;
  bool_t once = 0;

  logk_trace("Read by group type %d-%d " BLE_UUID_FMT "",
          task->read_by_group_type.start, task->read_by_group_type.end, BLE_UUID_ARG(&task->read_by_group_type.type));

  task->error = 0;

  for (err = ble_gattdb_client_seek(&gatt->client, task->read_by_group_type.start);
       err == 0
         && (uint16_t)(task->read_by_group_type.attribute_data_size_max - task->read_by_group_type.attribute_data_size) > 4
         && ble_gattdb_client_tell(&gatt->client) <= task->read_by_group_type.end;
       err = ble_gattdb_client_next(&gatt->client)) {
    const struct ble_uuid_s *type;
    uint16_t handle = ble_gattdb_client_tell(&gatt->client);

    err = ble_gattdb_client_type_get(&gatt->client, &type);

    if (err) {
      if (!once)
        task->error = err;
      break;
    }

    if (ble_uuid_cmp(type, &task->read_by_group_type.type))
      continue;

    size_t written;

    if (task->read_by_group_type.attribute_data_stride) {
      written = task->read_by_group_type.attribute_data_stride - 4;
      if (task->read_by_group_type.attribute_data_size + task->read_by_group_type.attribute_data_stride > task->read_by_group_type.attribute_data_size_max)
        break;
    } else {
      written = task->read_by_group_type.attribute_data_size_max - 4;
    }

    logk_trace(" reading handle %d, %d bytes max",
            handle, written);

    struct ble_att_data_s *ad = (void*)((uint8_t*)task->read_by_group_type.attribute_data
                                        + task->read_by_group_type.attribute_data_size);

    err = ble_gattdb_client_read(&gatt->client, 0, ad->value, &written);
    if (err) {
      if (!once)
        task->error = err;
      break;
    }

    endian_16_na_store(&ad->handle, handle);
    endian_16_na_store(&ad->end_group, gatt->client.cursor.registry->start_handle
                       + gatt->client.cursor.registry->handle_count
                       - 1);

    logk_trace(" %d - %d %P",
            handle, gatt->client.cursor.registry->start_handle
            + gatt->client.cursor.registry->handle_count
            - 1, ad->value, written);

    if (!task->read_by_group_type.attribute_data_stride)
      task->read_by_group_type.attribute_data_stride = written + 4;

    task->read_by_group_type.attribute_data_size += task->read_by_group_type.attribute_data_stride;

    once = 1;
  }

  return 0;
}

error_t _gatt_write(struct ble_gatt_s *gatt, uint16_t handle, const void *data, size_t size)
{
  error_t err;
  logk_trace("Write %d %P", handle, data, size);

  err = ble_gattdb_client_seek(&gatt->client, handle);
  if (err)
    return err;

  return ble_gattdb_client_write(&gatt->client, data, size);
}

error_t gatt_write(struct ble_gatt_s *gatt,
                    struct ble_att_transaction_s *task)
{
  task->error = _gatt_write(gatt, task->write.handle, task->write.value, task->write.value_size);

  return 0;
}

error_t ble_gatt_create(struct net_scheduler_s *scheduler,
                        const void *params_,
                        void *delegate,
                        const struct net_layer_delegate_vtable_s *delegate_vtable,
                        struct net_layer_s **layer)
{
  struct ble_gatt_s *gatt = mem_alloc(sizeof(*gatt), mem_scope_sys);
  const struct ble_gatt_params_s *params = params_;

  if (!gatt)
    return -ENOMEM;

  error_t err = net_layer_init(&gatt->layer, &gatt_handler, scheduler,
                               delegate, delegate_vtable);
  if (err) {
    mem_free(gatt);
    return err;
  }

  gatt->peer = params->peer;
  gatt->delayed_client_update = NULL;

#if defined(CONFIG_BLE_GATTDB_STREAM)
  gatt->stream_pending = 0;
#endif

  ble_gattdb_client_open(&gatt->client, &gatt_db_handler, params->db);
  ble_gattdb_client_subscription_set(&gatt->client, gatt->peer->subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);

  *layer = &gatt->layer;

  return 0;
}
