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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#define LOGK_MODULE_ID "gatC"

#include <mutek/buffer_pool.h>

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gattdb/client.h>

#include <ble/protocol/gatt.h>
#include <ble/protocol/att.h>
#include <ble/uuid.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <mutek/printk.h>

#include <ble/peer.h>

GCT_CONTAINER_FCNS(ble_gattdb_client_sl, ALWAYS_INLINE, ble_gattdb_client_sl,
                   init, destroy, pushback, remove, isempty, head, tail, next);

static
enum ble_att_error_e ble_gattdb_client_register(struct ble_gattdb_client_s *client,
                                              uint16_t mode);

static
uint16_t ble_gattdb_client_register_mode_get(struct ble_gattdb_client_s *client);

void ble_gattdb_client_open(struct ble_gattdb_client_s *client,
                             const struct ble_gattdb_client_handler_s *handler,
                             struct ble_gattdb_s *db)
{
  client->db = db;
  client->encrypted = 0;
  client->authenticated = 0;
  client->cursor.registry = NULL;
  client->cursor.handle = 0;
  ble_gattdb_client_sl_init(&client->subs_list);
  client->handler = handler;
}

void ble_gattdb_client_close(struct ble_gattdb_client_s *client)
{
  GCT_FOREACH(ble_gattdb_client_sl, &client->subs_list, sub,
              ble_gattdb_client_sl_remove(&client->subs_list, sub);
              ble_gattdb_listener_unregister(sub->listener.registry, &sub->listener);
              mem_free(sub);
              );

  ble_gattdb_client_sl_destroy(&client->subs_list);
}

enum ble_att_error_e ble_gattdb_client_seek(struct ble_gattdb_client_s *client,
                                          uint16_t handle)
{
  struct ble_gattdb_registry_s *s;

  if (client->cursor.registry) {
    if (client->cursor.handle == handle)
      return 0;

    s = client->cursor.registry;
    if (s->start_handle <= handle && handle < s->start_handle + s->handle_count) {
      client->cursor.handle = handle;
      return 0;
    }
  }

  client->cursor.registry = NULL;
  client->cursor.handle = 0;

  for (s = ble_gattdb_registry_list_head(&client->db->registry_list);
       s;
       s = ble_gattdb_registry_list_next(&client->db->registry_list, s)) {
    if (s->start_handle <= handle && handle < s->start_handle + s->handle_count)
      break;
  }

  if (!s)
    return BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;

  client->cursor.registry = s;
  client->cursor.handle = handle;

  return 0;
}

enum ble_att_error_e ble_gattdb_client_next(struct ble_gattdb_client_s *client)
{
  client->cursor.handle++;

  if (client->cursor.registry->start_handle <= client->cursor.handle
      && client->cursor.handle < client->cursor.registry->start_handle + client->cursor.registry->handle_count)
    return 0;

  client->cursor.registry
    = ble_gattdb_registry_list_next(&client->db->registry_list,
                                 client->cursor.registry);

  if (!client->cursor.registry)
    return BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;

  return 0;
}

static const struct ble_uuid_s ble_gatt_att_service_primary
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_SERVICE_PRIMARY);
static const struct ble_uuid_s ble_gatt_att_service_secondary
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_SERVICE_SECONDARY);
static const struct ble_uuid_s ble_gatt_att_service_include
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_SERVICE_INCLUDE);
static const struct ble_uuid_s ble_gatt_att_characteristic
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_CHARACTERISTIC);
static const struct ble_uuid_s ble_gatt_att_desc_cccd
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_DESC_CCCD);
static const struct ble_uuid_s ble_gatt_att_desc_sccd
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_DESC_SCCD);
static const struct ble_uuid_s ble_gatt_att_desc_cep
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_DESC_CEP);

enum ble_att_error_e ble_gattdb_client_type_get(struct ble_gattdb_client_s *client,
                                              const struct ble_uuid_s **type)
{
  const struct ble_gattdb_registry_s *reg = client->cursor.registry;
  const struct ble_gattdb_hndl_s *hndl = &reg->handle[client->cursor.handle - reg->start_handle];

  switch ((enum ble_gattdb_hndl_type_e)hndl->type) {
  case BLE_GATTDB_HNDL_SERVICE:
    if (reg->service->flags & BLE_GATTDB_SERVICE_PRIMARY)
      *type = &ble_gatt_att_service_primary;
    else
      *type = &ble_gatt_att_service_secondary;
    return 0;

#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  case BLE_GATTDB_HNDL_INCLUDE:
    *type = &ble_gatt_att_service_include;
    return 0;
#endif

  case BLE_GATTDB_HNDL_CHAR_TYPE:
    *type = &ble_gatt_att_characteristic;
    return 0;

  case BLE_GATTDB_HNDL_CHAR_VALUE:
    *type = reg->service->characteristic[hndl->index].type;
    return 0;

  case BLE_GATTDB_HNDL_CCCD:
    *type = &ble_gatt_att_desc_cccd;
    return 0;

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
  case BLE_GATTDB_HNDL_SCCD:
    *type = &ble_gatt_att_desc_sccd;
    return 0;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATTDB_HNDL_CEP:
    *type = &ble_gatt_att_desc_cep;
    return 0;
#endif

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  case BLE_GATTDB_HNDL_CHAR_DESC:
    *type = reg->service->characteristic[hndl->index]
               .descriptor[hndl->descriptor].type;
    return 0;
#endif
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}

enum ble_att_error_e ble_gattdb_client_read(struct ble_gattdb_client_s *client,
                                          size_t offset,
                                          void *data, size_t *size)
{
  struct ble_gattdb_registry_s *reg = client->cursor.registry;
  const struct ble_gattdb_hndl_s *hndl = &reg->handle[client->cursor.handle - reg->start_handle];
  const struct ble_gattdb_characteristic_s *chr = &reg->service->characteristic[hndl->index];

  switch ((enum ble_gattdb_hndl_type_e)hndl->type) {
  case BLE_GATTDB_HNDL_SERVICE:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    if (ble_uuid_is_uuid16(reg->service->type)) {
      if (*size < 2)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      endian_le16_na_store(data, ble_uuid_uuid16_get(reg->service->type));
      *size = 2;
    } else {
      if (*size < 16)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      memcpy(data, reg->service->type, 16);
      *size = 16;
    }
    return 0;

#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  case BLE_GATTDB_HNDL_INCLUDE: {
    struct ble_gattdb_registry_s *ireg;

    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;

    ireg = ble_gattdb_registry_get_by_service(client->db, reg->service->include[hndl->index], 0);
    if (!ireg)
      return BLE_ATT_ERR_UNLIKELY_ERROR;

    endian_le16_na_store(data, ireg->start_handle);
    endian_le16_na_store(data + 2, ireg->start_handle + ireg->handle_count - 1);
    if (ble_uuid_is_uuid16(ireg->service->type)) {
      *size = 6;
      endian_le16_na_store(data + 4, ble_uuid_uuid16_get(ireg->service->type));
    } else {
      *size = 4;
    }

    return 0;
  }
#endif

  case BLE_GATTDB_HNDL_CHAR_TYPE: {
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    if ((!ble_uuid_is_uuid16(chr->type) && *size < 19)
        || *size < 5)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t props = 0;
#if defined(CONFIG_BLE_GATTDB_BROADCAST)
    if (chr->permissions & BLE_GATTDB_BROADCASTABLE)
      props |= BLE_GATT_BROADCAST;
#endif
    if (chr->permissions & (BLE_GATTDB_PERM_OTHER_READ | BLE_GATTDB_PERM_ENC_READ | BLE_GATTDB_PERM_AUTH_READ))
      props |= BLE_GATT_READ;
    if (chr->permissions & (BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_ENC_WRITE | BLE_GATTDB_PERM_AUTH_WRITE))
      props |= BLE_GATT_WRITE | BLE_GATT_WRITE_WO_RSP;
    if (chr->permissions & BLE_GATTDB_NOTIFIABLE)
      props |= BLE_GATT_NOTIFY;
    if (chr->permissions & BLE_GATTDB_INDICABLE)
      props |= BLE_GATT_INDICATE;

    ((uint8_t *)data)[0] = props;
    endian_le16_na_store(data + 1, client->cursor.handle + 1);
    if (ble_uuid_is_uuid16(chr->type)) {
      endian_le16_na_store(data + 3, ble_uuid_uuid16_get(chr->type));
      *size = 5;
    } else {
      memcpy(data + 3, (const void *)chr->type, 16);
      *size = 19;
    }
    return 0;
  }

  case BLE_GATTDB_HNDL_CHAR_VALUE:
    if (chr->permissions & BLE_GATTDB_PERM_OTHER_READ)
      goto read;

    if (client->encrypted && (chr->permissions & BLE_GATTDB_PERM_ENC_READ))
        goto read;

    if (client->authenticated && (chr->permissions & BLE_GATTDB_PERM_AUTH_READ))
      goto read;

    if (client->encrypted && (chr->permissions & BLE_GATTDB_PERM_ENC_READ))
      return BLE_ATT_ERR_INSUF_ENCRYPTION;
    return BLE_ATT_ERR_INSUF_AUTHENTICATION;
  read:

    return ble_gattdb_char_read(client, reg, hndl->index, offset, data, size);

  case BLE_GATTDB_HNDL_CCCD:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    if (*size < 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    endian_le16_na_store(data, ble_gattdb_client_register_mode_get(client));
    *size = 2;
    return 0;

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
  case BLE_GATTDB_HNDL_SCCD:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATTDB_HNDL_CEP:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  case BLE_GATTDB_HNDL_CHAR_DESC: {
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    const struct ble_gattdb_descriptor_s *desc =
      &reg->service->characteristic[hndl->index].descriptor[hndl->descriptor];
    size_t transferred = __MIN(__MAX(0, offset + desc->size), *size);

    memcpy(data, desc->data + offset, transferred);
    *size = transferred;
    return 0;
  }
#endif
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}

enum ble_att_error_e ble_gattdb_client_write(struct ble_gattdb_client_s *client,
                                           const void *data, size_t size)
{
  struct ble_gattdb_registry_s *reg = client->cursor.registry;
  const struct ble_gattdb_hndl_s *hndl = &reg->handle[client->cursor.handle - reg->start_handle];
  const struct ble_gattdb_characteristic_s *chr = &reg->service->characteristic[hndl->index];

  switch ((enum ble_gattdb_hndl_type_e)hndl->type) {
  case BLE_GATTDB_HNDL_SERVICE:
#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  case BLE_GATTDB_HNDL_INCLUDE:
#endif
  case BLE_GATTDB_HNDL_CHAR_TYPE:
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

  case BLE_GATTDB_HNDL_CHAR_VALUE:
    if (chr->permissions & BLE_GATTDB_PERM_OTHER_WRITE)
      goto write;

    if (client->encrypted && (chr->permissions & BLE_GATTDB_PERM_ENC_WRITE))
        goto write;

    if (client->authenticated && (chr->permissions & BLE_GATTDB_PERM_AUTH_WRITE))
      goto write;

    if (client->encrypted && (chr->permissions & BLE_GATTDB_PERM_ENC_WRITE))
      return BLE_ATT_ERR_INSUF_ENCRYPTION;
    return BLE_ATT_ERR_INSUF_AUTHENTICATION;
  write:

    return ble_gattdb_char_write(client, reg, hndl->index, data, size);

  case BLE_GATTDB_HNDL_CCCD:
    if (size != 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    return ble_gattdb_client_register(client, endian_le16_na_load(data));

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
  case BLE_GATTDB_HNDL_SCCD:
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATTDB_HNDL_CEP:
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  case BLE_GATTDB_HNDL_CHAR_DESC: {
    const struct ble_gattdb_descriptor_s *desc =
      &reg->service->characteristic[hndl->index].descriptor[hndl->descriptor];
    if (!desc->writable)
      return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    if (size != desc->size)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    memcpy((uint8_t *)desc->data, data, size);
    return 0;
  }
#endif
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}

static BLE_GATTDB_LISTENER_FUNC(ble_gattdb_client_value_changed_handler)
{
  struct ble_gattdb_client_subs_s *sub = ble_gattdb_client_subs_s_from_listener(listener);
  const struct ble_gattdb_hndl_s *hndl = registry->handle;
  struct ble_gattdb_client_s *client = sub->client;

  uint16_t handle_index = 0;
  while (hndl[handle_index].index < listener->chr_index
         || hndl[handle_index].type != BLE_GATTDB_HNDL_CHAR_VALUE)
    handle_index++;

  const struct ble_gattdb_characteristic_s *chr
    = &registry->service->characteristic[listener->chr_index];

  logk("Characteristic %d of service changed, value handle %d",
         listener->chr_index, handle_index);

  assert(hndl[handle_index].index == listener->chr_index
         && hndl[handle_index].type == BLE_GATTDB_HNDL_CHAR_VALUE);

  if (!(chr->permissions & (BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_INDICABLE)))
    return;

  if (chr->permissions & BLE_GATTDB_PERM_OTHER_READ)
    goto ok;

  if (client->encrypted && (chr->permissions & BLE_GATTDB_PERM_ENC_READ))
    goto ok;

  if (client->authenticated && (chr->permissions & BLE_GATTDB_PERM_AUTH_READ))
    goto ok;

  return;
 ok:

#if defined(CONFIG_BLE_GATTDB_STREAM)
  if (!data && chr->permissions & BLE_GATTDB_NOTIFIABLE)
    client->handler->att_stream_resume(client,
                                       registry->start_handle + handle_index);
  else
#endif
    client->handler->att_value_changed(client,
                                       registry->start_handle + handle_index,
                                       sub->mode, data, size);
}

#if defined(CONFIG_BLE_GATTDB_STREAM)
error_t ble_gattdb_client_att_stream_get(struct ble_gattdb_client_s *client,
                                         uint16_t value_handle,
                                         struct buffer_s *buffer)
{
  error_t err;

  err = ble_gattdb_client_seek(client, value_handle);

  if (err)
    return err;

  struct ble_gattdb_registry_s *reg
    = client->cursor.registry;
  const struct ble_gattdb_hndl_s *hndl
    = &reg->handle[client->cursor.handle - reg->start_handle];
  const struct ble_gattdb_characteristic_s *chr
    = &reg->service->characteristic[hndl->index];

  if (chr->mode != BLE_GATTDB_CHARACTERISTIC_STREAM)
    return -ENOTSUP;

  return chr->data.stream.on_get_data(client, reg, hndl->index, buffer);
}
#endif

enum ble_att_error_e ble_gattdb_client_register(struct ble_gattdb_client_s *client,
                                              uint16_t mode)
{
  struct ble_gattdb_registry_s *reg
    = client->cursor.registry;
  const struct ble_gattdb_hndl_s *hndl
    = &reg->handle[client->cursor.handle - reg->start_handle];
  const struct ble_gattdb_characteristic_s *chr
    = &reg->service->characteristic[hndl->index];

  if (!(chr->permissions & (BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_INDICABLE)))
    return BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;

  if (!(chr->permissions & BLE_GATTDB_PERM_OTHER_READ)) {
    if (!client->encrypted && (chr->permissions & BLE_GATTDB_PERM_ENC_READ))
      return BLE_ATT_ERR_INSUF_ENCRYPTION;

    if (!client->authenticated && (chr->permissions & BLE_GATTDB_PERM_AUTH_READ))
      return BLE_ATT_ERR_INSUF_AUTHENTICATION;
  }

  GCT_FOREACH(ble_gattdb_client_sl, &client->subs_list, sub,
              if (sub->listener.chr_index == hndl->index
                  && sub->listener.registry == reg) {
                if (mode) {
                  sub->mode = mode;
                } else {
                  ble_gattdb_client_sl_remove(&client->subs_list, sub);
                  ble_gattdb_listener_unregister(sub->listener.registry, &sub->listener);
                  mem_free(sub);
                }
                client->handler->att_subscription_changed(client);
                return 0;
              }
              );

  if (!mode)
    return 0;

  struct ble_gattdb_client_subs_s *sub = mem_alloc(sizeof(*sub), mem_scope_sys);

  if (!sub)
    return BLE_ATT_ERR_UNLIKELY_ERROR;

  sub->client = client;
  sub->mode = mode;

  ble_gattdb_listener_register(reg, &sub->listener,
                                ble_gattdb_client_value_changed_handler,
                                hndl->index);
  ble_gattdb_client_sl_pushback(&client->subs_list, sub);
  client->handler->att_subscription_changed(client);

  return 0;
}

uint16_t ble_gattdb_client_register_mode_get(
  struct ble_gattdb_client_s *client)
{
  struct ble_gattdb_registry_s *reg
    = client->cursor.registry;
  const struct ble_gattdb_hndl_s *hndl
    = &reg->handle[client->cursor.handle - reg->start_handle];

  GCT_FOREACH(ble_gattdb_client_sl, &client->subs_list, sub,
              if (sub->listener.chr_index == hndl->index
                  && sub->listener.registry == reg)
                return sub->mode;
              );

  return 0;
}

static
int_fast8_t subscription_cmp(const void *a, const void *b)
{
  return memcmp(a, b, sizeof(struct ble_subscription_s));
}

void ble_gattdb_client_subscription_get(struct ble_gattdb_client_s *client,
                                      struct ble_subscription_s *subscriptions,
                                      size_t count)
{
  struct ble_gattdb_client_subs_s *sub;
  size_t i = 0;

  logk_debug("Saving subscriptions:");

  for (sub = ble_gattdb_client_sl_head(&client->subs_list);
       sub && i < count;
       sub = ble_gattdb_client_sl_next(&client->subs_list, sub)) {
    subscriptions[i].registry_index = sub->listener.registry->index;
    subscriptions[i].char_index = sub->listener.chr_index;
    subscriptions[i].mode = sub->mode == 2;

    logk_debug(" reg %d chr %d mode %d",
           subscriptions[i].registry_index,
           subscriptions[i].char_index,
           subscriptions[i].mode);

    i++;
  }

  qsort(subscriptions, i, sizeof(*subscriptions), subscription_cmp);

  memset(subscriptions + i, 0xff, sizeof(*subscriptions) * (count - i));

  logk_debug(" -> %P", subscriptions, sizeof(*subscriptions) * count);
}

void ble_gattdb_client_subscription_set(struct ble_gattdb_client_s *client,
                                      const struct ble_subscription_s *subscriptions,
                                      size_t count)
{
  GCT_FOREACH(ble_gattdb_client_sl, &client->subs_list, sub,
              ble_gattdb_client_sl_remove(&client->subs_list, sub);
              ble_gattdb_listener_unregister(sub->listener.registry, &sub->listener);
              mem_free(sub);
              );

  assert(ble_gattdb_client_sl_isempty(&client->subs_list));

  if (memcstcmp(subscriptions, 0xff, count) == 0)
    return;

  logk_debug("Restoring subscriptions: %P", subscriptions, count);

  for (size_t i = 0; i < count; ++i) {
    struct ble_gattdb_registry_s *reg = ble_gattdb_registry_get_by_index(client->db,
                                                                         subscriptions[i].registry_index);
    if (!reg)
      continue;

    GCT_FOREACH(ble_gattdb_client_sl, &client->subs_list, ss,
                if (ss->listener.chr_index == subscriptions[i].char_index
                    && ss->listener.registry == reg) {
                  ss->mode = 1 << subscriptions[i].mode;
                  goto next;
                });

    struct ble_gattdb_client_subs_s *sub = mem_alloc(sizeof(*sub), mem_scope_sys);

    if (!sub)
      goto out;

    if (subscriptions[i].char_index >= reg->service->characteristic_count)
      continue;

    if (!(reg->service->characteristic[subscriptions[i].char_index].permissions
          & (BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_INDICABLE)))
      continue;

    logk_debug(" reg %d chr %d mode %d",
           subscriptions[i].registry_index,
           subscriptions[i].char_index,
           subscriptions[i].mode);

    sub->client = client;
    sub->mode = 1 << subscriptions[i].mode;

    ble_gattdb_listener_register(reg, &sub->listener,
                                  ble_gattdb_client_value_changed_handler,
                                  subscriptions[i].char_index);
    ble_gattdb_client_sl_pushback(&client->subs_list, sub);

  next:
    ;
  }

 out:
  client->handler->att_subscription_changed(client);
}
