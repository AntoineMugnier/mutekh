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

#include <ble/net/gatt.h>
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

#define dprintk(...) do{}while(0)
//#define dprintk printk

static enum ble_att_error_e gatt_read_by_group_type(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end,
  const struct ble_uuid_s *type);

static enum ble_att_error_e gatt_find_information(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end);

static enum ble_att_error_e gatt_read(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t handle);

static enum ble_att_error_e gatt_read_blob(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t offset,
  uint16_t handle);

static void gatt_read_multiple(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  const uint16_t *handle, size_t handle_count);

static enum ble_att_error_e gatt_write(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t handle,
  const void *data, size_t size);

static enum ble_att_error_e gatt_read_by_type(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end,
  const struct ble_uuid_s *type);

static enum ble_att_error_e gatt_find_by_type_value(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end, uint16_t type,
  const void *value, size_t size);

static
void ble_gatt_destroyed(struct net_layer_s *layer)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  ble_gatt_client_db_close(&gatt->client);

  mem_free(gatt);
}

static
void gatt_error_set(struct buffer_s *rsp, uint8_t cmd, uint16_t handle, uint8_t err)
{
  rsp->begin = 6;
  rsp->end = 11;
  rsp->data[rsp->begin] = BLE_ATT_ERROR_RSP;
  rsp->data[rsp->begin + 1] = cmd;
  endian_le16_na_store(&rsp->data[rsp->begin + 2], handle);
  rsp->data[rsp->begin + 4] = err;
}

static void gatt_command_handle(struct ble_gatt_s *gatt, struct net_task_s *task)
{
  const uint8_t *data = task->inbound.buffer->data + task->inbound.buffer->begin;
  const size_t size = task->inbound.buffer->end - task->inbound.buffer->begin;
  struct buffer_s *rsp = task->inbound.buffer;
  struct net_addr_s dst = {
    .cid = BLE_L2CAP_CID_ATT,
  };
  uint8_t err;
  uint16_t handle = 0;
  struct ble_uuid_s type;
  uint8_t cmd = data[0];

  dprintk("Gatt command %02x (%d) %P\n", cmd, size, data + 1, size - 1);

  switch (cmd) {
  case BLE_ATT_EXCHANGE_MTU_RQT:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    if (endian_le16_na_load(data + 1) < gatt->server_mtu)
      gatt->server_mtu = endian_le16_na_load(data + 1);

    rsp->begin = gatt->layer.context.prefix_size;
    rsp->data[rsp->begin] = BLE_ATT_EXCHANGE_MTU_RSP;
    endian_le16_na_store(&rsp->data[rsp->begin + 1],
                         gatt->layer.context.mtu - 3);
    rsp->end = rsp->begin + 3;

    goto send;

  case BLE_ATT_EXCHANGE_MTU_RSP:
    if (size != 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    gatt->server_mtu = endian_le16_na_load(data + 1);
    break;

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT:
    if (size == 21)
      memcpy(&type, data + 5, 16);
    else if (size == 7)
      ble_uuid_bluetooth_based(&type, endian_le16_na_load(data + 5));
    else {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    {
      uint16_t end = endian_le16_na_load(data + 3);
      handle = endian_le16_na_load(data + 1);

      if (handle > end || handle == 0) {
        err = BLE_ATT_ERR_INVALID_HANDLE;
        goto error;
      }

      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      err = gatt_read_by_group_type(gatt, rsp, handle, end, &type);

      if (err)
        goto error;
      goto send;
    }

  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT: {
    handle = endian_le16_na_load(data + 1);

    void *tmp = alloca(size - 7);
    memcpy(tmp, data + 7, size - 7);

    rsp->begin = rsp->end = gatt->layer.context.prefix_size;
    err = gatt_find_by_type_value(gatt, rsp, handle,
                                  endian_le16_na_load(data + 3),
                                  endian_le16_na_load(data + 5),
                                  tmp, size - 7);
    if (err)
      goto error;

    goto send;
  }

  case BLE_ATT_READ_RQT:
    if (size < 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    } else {
      handle = endian_le16_na_load(data + 1);
      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      err = gatt_read(gatt, rsp, handle);

      if (err)
        goto error;
      goto send;
    }

  case BLE_ATT_READ_MULTIPLE_RQT:
    if (size < 5) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    } else {
      uint16_t *tmp = alloca(size - 1);
      memcpy(tmp, data + 1, size - 1);

      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      gatt_read_multiple(gatt, rsp, tmp, (size - 1) / 2);
      goto send;
    }

  case BLE_ATT_READ_BLOB_RQT:
    if (size < 5) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    } else {
      handle = endian_le16_na_load(data + 1);
      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      err = gatt_read_blob(gatt, rsp, endian_le16_na_load(data + 3), handle);
      if (err)
        goto error;
      goto send;
    }

  case BLE_ATT_FIND_INFORMATION_RQT:
    if (size != 5) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    } else {
      handle = endian_le16_na_load(data + 1);
      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      err = gatt_find_information(gatt, rsp, handle, endian_le16_na_load(data + 3));
      if (err)
        goto error;
      goto send;
    }

  case BLE_ATT_WRITE_CMD:
    if (size < 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    } else {
      void *tmp = alloca(size - 3);
      memcpy(tmp, data + 3, size - 3);

      handle = endian_le16_na_load(data + 1);
      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      gatt_write(gatt, rsp, handle, tmp, size - 3);

      goto out;
    }

  case BLE_ATT_WRITE_RQT:
    if (size < 3) {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    } else {
      void *tmp = alloca(size - 3);
      memcpy(tmp, data + 3, size - 3);

      handle = endian_le16_na_load(data + 1);
      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      err = gatt_write(gatt, rsp, handle, tmp, size - 3);

      if (err)
        goto error;

      goto send;
    }

  case BLE_ATT_READ_BY_TYPE_RQT:
    if (size == 21)
      memrevcpy((uint8_t *)&type, data + 5, 16);
    else if (size == 7)
      ble_uuid_bluetooth_based(&type, endian_le16_na_load(data + 5));
    else {
      err = BLE_ATT_ERR_INVALID_PDU;
      goto error;
    }

    {
      uint16_t end = endian_le16_na_load(data + 3);
      handle = endian_le16_na_load(data + 1);

      if (handle > end || handle == 0) {
        err = BLE_ATT_ERR_INVALID_HANDLE;
        goto error;
      }

      rsp->begin = rsp->end = gatt->layer.context.prefix_size;
      err = gatt_read_by_type(gatt, rsp, handle, end, &type);

      if (err)
        goto error;
      goto send;
    }

  case BLE_ATT_HANDLE_VALUE_CONFIRM:
    goto out;

  default:
    err = BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;
    goto error;
  }

  goto out;

 error:
  rsp->begin = rsp->end = gatt->layer.context.prefix_size;
  gatt_error_set(rsp, cmd, handle, err);

 send:
  dprintk("Gatt response %02x (%d) %P\n", rsp->data[rsp->begin],
         rsp->end - rsp->begin,
         &rsp->data[rsp->begin + 1],
         rsp->end - rsp->begin - 1);

  if (gatt->layer.parent) {
    net_task_inbound_respond(task, 0, &dst);
    return;
  }

 out:
  net_task_destroy(task);
}

static
void ble_gatt_task_handle(struct net_layer_s *layer,
                         struct net_task_s *task)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_INBOUND:
    if (task->source == layer->parent) {
      gatt_command_handle(gatt, task);
      return;
    }
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
  }

  default:
    break;
  }

  net_task_destroy(task);
}

static
void ble_gatt_att_value_changed(struct ble_gatt_client_s *client,
                                uint16_t value_handle, uint16_t mode,
                                const void *data, size_t size)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_client(client);

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

static void gatt_save_peer_later(struct ble_gatt_s *gatt)
{
  if (gatt->delayed_client_update)
    net_scheduler_task_cancel(gatt->layer.scheduler,
                              gatt->delayed_client_update);

  struct net_task_s *timeout = net_scheduler_task_alloc(gatt->layer.scheduler);
  dev_timer_delay_t ticks;
  dev_timer_init_sec(&gatt->layer.scheduler->timer, &ticks, NULL, 2, 1);
  net_task_timeout_push(timeout, &gatt->layer,
                        net_scheduler_time_get(gatt->layer.scheduler) + ticks, 0);
  gatt->delayed_client_update = timeout;
}

static
void ble_gatt_att_subscription_changed(struct ble_gatt_client_s *client)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_client(client);

  gatt_save_peer_later(gatt);
}

static bool_t ble_gatt_context_updated(
    struct net_layer_s *layer,
    const struct net_layer_context_s *parent_context)
{
  struct ble_gatt_s *gatt = ble_gatt_s_from_layer(layer);

  gatt->layer.context = *parent_context;
  gatt->client.encrypted = parent_context->addr.encrypted;

  printk("Gatt client layer now %s\n", gatt->client.encrypted ? "encrypted" : "clear text");

  ble_gatt_client_subscription_set(&gatt->client, gatt->peer->subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);

  return 1;
}

static const struct net_layer_handler_s gatt_handler = {
  .destroyed = ble_gatt_destroyed,
  .task_handle = ble_gatt_task_handle,
  .context_updated = ble_gatt_context_updated,
  .type = BLE_NET_LAYER_GATT,
};

static const struct ble_gatt_client_handler_s gatt_db_handler = {
  .att_value_changed = ble_gatt_att_value_changed,
  .att_subscription_changed = ble_gatt_att_subscription_changed,
};

static enum ble_att_error_e gatt_read_by_group_type(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end,
  const struct ble_uuid_s *type)
{
  enum ble_att_error_e err;
  const struct ble_uuid_s *tmp;
  int16_t available = gatt->server_mtu - 2;
  uint8_t *unit_size = rsp->data + rsp->begin + 1;
  size_t written;

  dprintk("Read by group type %d-%d " BLE_UUID_FMT "\n", start, end, BLE_UUID_ARG(type));

  *unit_size = 0;
  rsp->data[rsp->begin] = BLE_ATT_READ_BY_GROUP_TYPE_RSP;
  rsp->end = rsp->begin + 2;

  assert(buffer_available(rsp) >= available);

  for (err = ble_gatt_client_seek(&gatt->client, start);
       err == 0 && ble_gatt_client_tell(&gatt->client) <= end;
       err = ble_gatt_client_next(&gatt->client)) {
    err = ble_gatt_client_type_get(&gatt->client, &tmp);

    dprintk(" type for handle %d: " BLE_UUID_FMT " / err %d\n",
           ble_gatt_client_tell(&gatt->client),
           BLE_UUID_ARG(tmp), err);

    if (err)
      break;

    if (ble_uuid_cmp(tmp, type))
      continue;

    if (available < *unit_size)
      break;

    written = available - 4;
    err = ble_gatt_client_read(&gatt->client, 0,
                               rsp->data + rsp->end + 4,
                               &written);

    dprintk(" reading handle %d, %d bytes\n", ble_gatt_client_tell(&gatt->client), written);

    if (err)
      break;

    if (*unit_size) {
      if (written + 4 != *unit_size)
        break;
    } else {
      *unit_size = written + 4;
      dprintk(" unit size now %d\n", *unit_size);
    }

    endian_le16_na_store(rsp->data + rsp->end, ble_gatt_client_tell(&gatt->client));
    endian_le16_na_store(rsp->data + rsp->end + 2,
                         gatt->client.cursor.service->start_handle
                         + gatt->client.cursor.service->handle_count
                         - 1);
    rsp->end += 4 + written;
    available -= 4 + written;
  }

  if (((err == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
       || (ble_gatt_client_tell(&gatt->client) > end))
      && *unit_size)
    endian_le16_na_store(rsp->data + rsp->end - *unit_size + 2, end);

  dprintk(" loop end with error %d\n", err);

  return *unit_size ? 0 : err;
}

static enum ble_att_error_e gatt_read(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t handle)
{
  enum ble_att_error_e err;
  size_t written;

  err = ble_gatt_client_seek(&gatt->client, handle);
  if (err)
    return err;

  rsp->data[rsp->begin] = BLE_ATT_READ_RSP;
  rsp->end = rsp->begin + 1;

  written = gatt->server_mtu - 1;
  err = ble_gatt_client_read(&gatt->client, 0, rsp->data + rsp->end, &written);
  rsp->end += written;

  return err;
}

static enum ble_att_error_e gatt_read_blob(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t offset,
  uint16_t handle)
{
  enum ble_att_error_e err;
  size_t written;

  err = ble_gatt_client_seek(&gatt->client, handle);
  if (err)
    return err;

  rsp->data[rsp->begin] = BLE_ATT_READ_BLOB_RSP;
  rsp->end = rsp->begin + 1;

  written = gatt->server_mtu - 1;
  err = ble_gatt_client_read(&gatt->client, offset, rsp->data + rsp->end, &written);
  rsp->end += written;

  return err;
}

static enum ble_att_error_e gatt_write(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t handle,
  const void *data, size_t size)
{
  enum ble_att_error_e err;

  rsp->data[rsp->begin] = BLE_ATT_WRITE_RSP;
  rsp->end = rsp->begin + 1;

  err = ble_gatt_client_seek(&gatt->client, handle);
  if (err)
    return err;

  return ble_gatt_client_write(&gatt->client, data, size);
}

static enum ble_att_error_e gatt_find_information(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end)
{
  enum ble_att_error_e err;
  const struct ble_uuid_s *tmp;
  int16_t available = gatt->server_mtu - 2;
  uint8_t *format = rsp->data + rsp->begin + 1;

  *format = 0;
  rsp->data[rsp->begin] = BLE_ATT_FIND_INFORMATION_RSP;
  rsp->end = rsp->begin + 2;

  assert(buffer_available(rsp) >= available);

  for (err = ble_gatt_client_seek(&gatt->client, start);
       err == 0 && available >= 4 && ble_gatt_client_tell(&gatt->client) <= end;
       err = ble_gatt_client_next(&gatt->client)) {
    err = ble_gatt_client_type_get(&gatt->client, &tmp);

    dprintk("%s %d " BLE_UUID_FMT " %d\n", __FUNCTION__, ble_gatt_client_tell(&gatt->client), 
           BLE_UUID_ARG(tmp), err);

    if (*format != BLE_GATT_NONE) {
      if (ble_uuid_is_uuid16(tmp) != (*format == BLE_GATT_HANDLE_UUID16))
        return 0;
    }

    if (ble_uuid_is_uuid16(tmp)) {
      *format = BLE_GATT_HANDLE_UUID16;
      if (available < 4)
        return 0;
      endian_le16_na_store(rsp->data + rsp->end, ble_gatt_client_tell(&gatt->client));
      endian_le16_na_store(rsp->data + rsp->end + 2, ble_uuid_uuid16_get(tmp));
      rsp->end += 4;
      available -= 4;
    } else {
      *format = BLE_GATT_HANDLE_UUID128;
      if (available < 18)
        return 0;
      endian_le16_na_store(rsp->data + rsp->end, ble_gatt_client_tell(&gatt->client));
      memcpy(rsp->data + rsp->end + 2, tmp, 16);
      rsp->end += 18;
      available -= 18;
    }

    dprintk("%s at %d / %d, %d available\n", __FUNCTION__,
           ble_gatt_client_tell(&gatt->client), end, available);
  }

  if (err) {
    if (*format != BLE_GATT_NONE)
      return 0;
    else
      return err;
  }

  return err;
}

static enum ble_att_error_e gatt_find_by_type_value(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end, uint16_t type,
  const void *value, size_t size)
{
  enum ble_att_error_e err;
  const struct ble_uuid_s *tmp;
  int16_t available = gatt->server_mtu - 1;
  uint8_t *tmp_data = alloca(size + 1);

  rsp->data[rsp->begin] = BLE_ATT_FIND_BY_TYPE_VALUE_RSP;
  rsp->end = rsp->begin + 1;

  assert(buffer_available(rsp) >= available);

  for (err = ble_gatt_client_seek(&gatt->client, start);
       err == 0 && available >= 4 && ble_gatt_client_tell(&gatt->client) <= end;
       err = ble_gatt_client_next(&gatt->client)) {
    size_t rsize = size + 1;

    err = ble_gatt_client_type_get(&gatt->client, &tmp);
    if (err)
      break;

    if (!ble_uuid_is_uuid16(tmp) || ble_uuid_uuid16_get(tmp) != type)
      continue;

    err = ble_gatt_client_read(&gatt->client, 0, tmp_data, &rsize);
    if (err || size != rsize || memcmp(tmp_data, value, size))
      continue;

    dprintk("%s at %d -> %d, %d available\n", __FUNCTION__,
           ble_gatt_client_tell(&gatt->client),
            gatt->client.cursor.service->start_handle
            + gatt->client.cursor.service->handle_count - 1, available);

    endian_le16_na_store(rsp->data + rsp->end, ble_gatt_client_tell(&gatt->client));
    endian_le16_na_store(rsp->data + rsp->end + 2,
                         gatt->client.cursor.service->start_handle
                         + gatt->client.cursor.service->handle_count - 1);
    rsp->end += 4;
    available -= 4;
  }

  if (rsp->end == rsp->begin + 1)
    return BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;
  else
    return 0;
}

static enum ble_att_error_e gatt_read_by_type(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  uint16_t start, uint16_t end,
  const struct ble_uuid_s *type)
{
  enum ble_att_error_e err;
  const struct ble_uuid_s *tmp;
  int16_t available = gatt->server_mtu - 2;
  uint8_t *unit_size = rsp->data + rsp->begin + 1;
  size_t written;

  dprintk("Read by type %d-%d " BLE_UUID_FMT "\n", start, end, BLE_UUID_ARG(type));

  *unit_size = 0;
  rsp->data[rsp->begin] = BLE_ATT_READ_BY_TYPE_RSP;
  rsp->end = rsp->begin + 2;

  assert(buffer_available(rsp) >= available);

  for (err = ble_gatt_client_seek(&gatt->client, start);
       err == 0 && available > *unit_size && ble_gatt_client_tell(&gatt->client) <= end;
       err = ble_gatt_client_next(&gatt->client)) {
    err = ble_gatt_client_type_get(&gatt->client, &tmp);

    dprintk(" type for handle %d: " BLE_UUID_FMT " / err %d\n",
           ble_gatt_client_tell(&gatt->client),
           BLE_UUID_ARG(tmp), err);

    if (err)
      break;

    if (ble_uuid_cmp(tmp, type))
      continue;

    dprintk(" reading handle %d\n", ble_gatt_client_tell(&gatt->client));

    written = available - 2;
    err = ble_gatt_client_read(&gatt->client, 0,
                               rsp->data + rsp->end + 2, &written);

    if (err == BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN)
      break;

    if (*unit_size) {
      if (written + 2 != *unit_size)
        break;
    } else {
      *unit_size = written + 2;
      dprintk(" unit size now %d\n", *unit_size);
    }

    endian_le16_na_store(rsp->data + rsp->end, ble_gatt_client_tell(&gatt->client));
    rsp->end += 2 + written;
    available -= 2 + written;
  }

  return *unit_size ? 0 : BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;
}

static void gatt_read_multiple(
  struct ble_gatt_s *gatt,
  struct buffer_s *rsp,
  const uint16_t *handle_array, size_t handle_count)
{
  enum ble_att_error_e err;
  int16_t available = gatt->server_mtu - 1;
  size_t written;
  size_t i;
  uint16_t handle = 0;

  rsp->data[rsp->begin] = BLE_ATT_READ_MULTIPLE_RSP;
  rsp->end = rsp->begin + 1;

  assert(buffer_available(rsp) >= available);

  for (i = 0; i < handle_count && available; ++i) {
    handle = endian_le16_na_load(&handle_array[i]);

    err = ble_gatt_client_seek(&gatt->client, handle);
    if (err)
      goto error;

    written = available;
    err = ble_gatt_client_read(&gatt->client, 0, rsp->data + rsp->end, &written);

    if (err == BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN)
      break;

    if (err)
      goto error;

    rsp->end += written;
    available -= written;
  }
  return;

 error:
  gatt_error_set(rsp, BLE_ATT_READ_MULTIPLE_RQT, handle, err);
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

  gatt->server_mtu = 23;
  gatt->peer = params->peer;

  ble_gatt_client_db_open(&gatt->client, &gatt_db_handler, params->db);
  ble_gatt_client_subscription_set(&gatt->client, params->peer->subscriptions, BLE_SUBSCRIBED_CHAR_COUNT);

  *layer = &gatt->layer;

  return err;
}
