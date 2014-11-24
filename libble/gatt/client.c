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

#include <mutek/buffer_pool.h>

#include <ble/gatt/db.h>
#include <ble/gatt/service.h>
#include <ble/gatt/client.h>

#include <ble/protocol/gatt.h>
#include <ble/protocol/att.h>
#include <ble/uuid.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <mutek/printk.h>

#include <ble/peer.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

GCT_CONTAINER_FCNS(ble_gatt_client_sl, ALWAYS_INLINE, ble_gatt_client_sl,
                   init, destroy, pushback, remove, isempty, head, tail, next);

static const struct ble_uuid_s ble_uuid_gatt_service_primary = BLE_UUID_SHORT(0x2800);
static const struct ble_uuid_s ble_uuid_gatt_service_secondary = BLE_UUID_SHORT(0x2801);
static const struct ble_uuid_s ble_uuid_gatt_service_include = BLE_UUID_SHORT(0x2802);
static const struct ble_uuid_s ble_uuid_gatt_characteristic = BLE_UUID_SHORT(0x2803);
static const struct ble_uuid_s ble_uuid_gatt_desc_cep = BLE_UUID_SHORT(0x2900);
static const struct ble_uuid_s ble_uuid_gatt_desc_cud = BLE_UUID_SHORT(0x2901);
static const struct ble_uuid_s ble_uuid_gatt_desc_cccd = BLE_UUID_SHORT(0x2902);
static const struct ble_uuid_s ble_uuid_gatt_desc_sccd = BLE_UUID_SHORT(0x2903);
static const struct ble_uuid_s ble_uuid_gatt_desc_cf = BLE_UUID_SHORT(0x2904);
static const struct ble_uuid_s ble_uuid_gatt_desc_caf = BLE_UUID_SHORT(0x2905);

static
enum ble_att_error_e ble_gatt_client_register(struct ble_gatt_client_s *client,
                                              uint16_t mode);

static
uint16_t ble_gatt_client_register_mode_get(struct ble_gatt_client_s *client);

void ble_gatt_client_db_open(struct ble_gatt_client_s *client,
                             const struct ble_gatt_client_handler_s *handler,
                             struct ble_gatt_db_s *db)
{
  client->db = db;
  client->encrypted = 0;
  client->authenticated = 0;
  client->cursor.service = NULL;
  client->cursor.handle = 0;
  ble_gatt_client_sl_init(&client->subs_list);
  client->handler = handler;
}

void ble_gatt_client_db_close(struct ble_gatt_client_s *client)
{
  GCT_FOREACH(ble_gatt_client_sl, &client->subs_list, sub,
              ble_gatt_client_sl_remove(&client->subs_list, sub);
              ble_gatt_db_listener_unregister(sub->listener.service, &sub->listener);
              mem_free(sub);
              );

  ble_gatt_client_sl_destroy(&client->subs_list);
}

enum ble_att_error_e ble_gatt_client_seek(struct ble_gatt_client_s *client,
                                          uint16_t handle)
{
  struct ble_gatt_db_service_s *s;

  if (client->cursor.service) {
    if (client->cursor.handle == handle)
      return 0;

    s = client->cursor.service;
    if (s->start_handle <= handle && handle < s->start_handle + s->handle_count) {
      client->cursor.handle = handle;
      return 0;
    }
  }

  client->cursor.service = NULL;
  client->cursor.handle = 0;

  for (s = ble_gatt_service_list_head(&client->db->service_list);
       s;
       s = ble_gatt_service_list_next(&client->db->service_list, s)) {
    if (s->start_handle <= handle && handle < s->start_handle + s->handle_count)
      break;
  }

  if (!s)
    return BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;

  client->cursor.service = s;
  client->cursor.handle = handle;

  return 0;
}

enum ble_att_error_e ble_gatt_client_next(struct ble_gatt_client_s *client)
{
  client->cursor.handle++;

  if (client->cursor.service->start_handle <= client->cursor.handle
      && client->cursor.handle < client->cursor.service->start_handle + client->cursor.service->handle_count)
    return 0;

  client->cursor.service
    = ble_gatt_service_list_next(&client->db->service_list,
                                 client->cursor.service);

  if (!client->cursor.service)
    return BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND;

  return 0;
}

enum ble_att_error_e ble_gatt_client_type_get(struct ble_gatt_client_s *client,
                                              const struct ble_uuid_s **type)
{
  const struct ble_gatt_db_service_s *srv = client->cursor.service;
  const struct ble_gatt_db_hndl_s *hndl = &srv->handle[client->cursor.handle - srv->start_handle];

  switch ((enum ble_gatt_db_hndl_type_e)hndl->type) {
  case BLE_GATT_HNDL_SERVICE:
    if (srv->service->flags & BLE_GATT_SERVICE_PRIMARY)
      *type = &ble_uuid_gatt_service_primary;
    else
      *type = &ble_uuid_gatt_service_secondary;
    return 0;

#if defined(CONFIG_BLE_GATT_INCLUDE)
  case BLE_GATT_HNDL_INCLUDE:
    *type = &ble_uuid_gatt_service_include;
    return 0;
#endif

  case BLE_GATT_HNDL_CHAR_TYPE:
    *type = &ble_uuid_gatt_characteristic;
    return 0;

  case BLE_GATT_HNDL_CHAR_VALUE:
    *type = srv->service->characteristic[hndl->index].type;
    return 0;

  case BLE_GATT_HNDL_CCCD:
    *type = &ble_uuid_gatt_desc_cccd;
    return 0;

#if defined(CONFIG_BLE_GATT_BROADCAST)
  case BLE_GATT_HNDL_SCCD:
    *type = &ble_uuid_gatt_desc_sccd;
    return 0;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATT_HNDL_CEP:
    *type = &ble_uuid_gatt_desc_cep;
    return 0;
#endif

#if defined(CONFIG_BLE_GATT_DESCRIPTOR)
  case BLE_GATT_HNDL_CHAR_DESC:
    *type = srv->service->characteristic[hndl->index]
               .descriptor[hndl->descriptor].type;
    return 0;
#endif
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}

enum ble_att_error_e ble_gatt_client_read(struct ble_gatt_client_s *client,
                                          size_t offset,
                                          void *data, size_t *size)
{
  struct ble_gatt_db_service_s *srv = client->cursor.service;
  const struct ble_gatt_db_hndl_s *hndl = &srv->handle[client->cursor.handle - srv->start_handle];
  const struct ble_gatt_characteristic_s *chr = &srv->service->characteristic[hndl->index];

  switch ((enum ble_gatt_db_hndl_type_e)hndl->type) {
  case BLE_GATT_HNDL_SERVICE:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    if (ble_uuid_is_uuid16(srv->service->type)) {
      if (*size < 2)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      endian_le16_na_store(data, ble_uuid_uuid16_get(srv->service->type));
      *size = 2;
    } else {
      if (*size < 16)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      memrevcpy(data, srv->service->type, 16);
      *size = 16;
    }
    return 0;

#if defined(CONFIG_BLE_GATT_INCLUDE)
  case BLE_GATT_HNDL_INCLUDE: {
    struct ble_gatt_db_service_s *isrv;

    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;

    isrv = ble_gatt_db_service_get_by_service(client->db, srv->service->include[hndl->index], 0);
    if (!isrv)
      return BLE_ATT_ERR_UNLIKELY_ERROR;

    endian_le16_na_store(data, isrv->start_handle);
    endian_le16_na_store(data + 2, isrv->start_handle + isrv->handle_count - 1);
    if (ble_uuid_is_uuid16(isrv->service->type)) {
      *size = 6;
      endian_le16_na_store(data + 4, ble_uuid_uuid16_get(isrv->service->type));
    } else {
      *size = 4;
    }

    return 0;
  }
#endif

  case BLE_GATT_HNDL_CHAR_TYPE: {
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    if ((!ble_uuid_is_uuid16(chr->type) && *size < 19)
        || *size < 5)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t props = 0;
#if defined(CONFIG_BLE_GATT_BROADCAST)
    if (chr->permissions & BLE_GATT_BROADCASTABLE)
      props |= BLE_GATT_BROADCAST;
#endif
    if (chr->permissions & (BLE_GATT_PERM_OTHER_READ | BLE_GATT_PERM_ENC_READ | BLE_GATT_PERM_AUTH_READ))
      props |= BLE_GATT_READ;
    if (chr->permissions & (BLE_GATT_PERM_OTHER_WRITE | BLE_GATT_PERM_ENC_WRITE | BLE_GATT_PERM_AUTH_WRITE))
      props |= BLE_GATT_WRITE | BLE_GATT_WRITE_WO_RSP;
    if (chr->permissions & BLE_GATT_NOTIFIABLE)
      props |= BLE_GATT_NOTIFY;
    if (chr->permissions & BLE_GATT_INDICABLE)
      props |= BLE_GATT_INDICATE;

    ((uint8_t *)data)[0] = props;
    endian_le16_na_store(data + 1, client->cursor.handle + 1);
    if (ble_uuid_is_uuid16(chr->type)) {
      endian_le16_na_store(data + 3, ble_uuid_uuid16_get(chr->type));
      *size = 5;
    } else {
      memrevcpy(data + 3, (const void *)chr->type, 16);
      *size = 19;
    }
    return 0;
  }

  case BLE_GATT_HNDL_CHAR_VALUE:
    if (chr->permissions & BLE_GATT_PERM_OTHER_READ)
      goto read;

    if (client->encrypted && (chr->permissions & BLE_GATT_PERM_ENC_READ))
        goto read;

    if (client->authenticated && (chr->permissions & BLE_GATT_PERM_AUTH_READ))
      goto read;

    if (client->encrypted && (chr->permissions & BLE_GATT_PERM_ENC_READ))
      return BLE_ATT_ERR_INSUF_ENCRYPTION;
    return BLE_ATT_ERR_INSUF_AUTHENTICATION;
  read:

    return ble_gatt_db_char_read(client, srv, hndl->index, offset, data, size);

  case BLE_GATT_HNDL_CCCD:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    if (*size < 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    endian_le16_na_store(data, ble_gatt_client_register_mode_get(client));
    *size = 2;
    return 0;

#if defined(CONFIG_BLE_GATT_BROADCAST)
  case BLE_GATT_HNDL_SCCD:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATT_HNDL_CEP:
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_GATT_DESCRIPTOR)
  case BLE_GATT_HNDL_CHAR_DESC: {
    if (offset)
        return BLE_ATT_ERR_INVALID_OFFSET;
    const struct ble_gatt_descriptor_s *desc =
      &srv->service->characteristic[hndl->index].descriptor[hndl->descriptor];
    size_t transferred = __MIN(__MAX(0, offset + desc->size), *size);

    memcpy(data, desc->data + offset, transferred);
    *size = transferred;
    return 0;
  }
#endif
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}

enum ble_att_error_e ble_gatt_client_write(struct ble_gatt_client_s *client,
                                           const void *data, size_t size)
{
  struct ble_gatt_db_service_s *srv = client->cursor.service;
  const struct ble_gatt_db_hndl_s *hndl = &srv->handle[client->cursor.handle - srv->start_handle];
  const struct ble_gatt_characteristic_s *chr = &srv->service->characteristic[hndl->index];

  switch ((enum ble_gatt_db_hndl_type_e)hndl->type) {
  case BLE_GATT_HNDL_SERVICE:
#if defined(CONFIG_BLE_GATT_INCLUDE)
  case BLE_GATT_HNDL_INCLUDE:
#endif
  case BLE_GATT_HNDL_CHAR_TYPE:
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

  case BLE_GATT_HNDL_CHAR_VALUE:
    if (chr->permissions & BLE_GATT_PERM_OTHER_WRITE)
      goto write;

    if (client->encrypted && (chr->permissions & BLE_GATT_PERM_ENC_WRITE))
        goto write;

    if (client->authenticated && (chr->permissions & BLE_GATT_PERM_AUTH_WRITE))
      goto write;

    if (client->encrypted && (chr->permissions & BLE_GATT_PERM_ENC_WRITE))
      return BLE_ATT_ERR_INSUF_ENCRYPTION;
    return BLE_ATT_ERR_INSUF_AUTHENTICATION;
  write:

    return ble_gatt_db_char_write(client, srv, hndl->index, data, size);

  case BLE_GATT_HNDL_CCCD:
    if (size != 2)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    return ble_gatt_client_register(client, endian_le16_na_load(data));

#if defined(CONFIG_BLE_GATT_BROADCAST)
  case BLE_GATT_HNDL_SCCD:
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATT_HNDL_CEP:
# error implement me
    break;
#endif

#if defined(CONFIG_BLE_GATT_DESCRIPTOR)
  case BLE_GATT_HNDL_CHAR_DESC: {
    const struct ble_gatt_descriptor_s *desc =
      &srv->service->characteristic[hndl->index].descriptor[hndl->descriptor];
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

static BLE_GATT_DB_LISTENER_FUNC(ble_gatt_client_value_changed_handler)
{
  struct ble_gatt_client_subs_s *sub = ble_gatt_client_subs_s_from_listener(listener);
  const struct ble_gatt_db_hndl_s *hndl = service->handle;
  struct ble_gatt_client_s *client = sub->client;

  uint16_t handle_index = 0;
  while (hndl[handle_index].index < listener->chr_index
         || hndl[handle_index].type != BLE_GATT_HNDL_CHAR_VALUE)
    handle_index++;

  const struct ble_gatt_characteristic_s *chr
    = &service->service->characteristic[listener->chr_index];

  dprintk("Characteristic %d of service changed, value handle %d\n",
         listener->chr_index, handle_index);

  assert(hndl[handle_index].index == listener->chr_index
         && hndl[handle_index].type == BLE_GATT_HNDL_CHAR_VALUE);

  if (!(chr->permissions & (BLE_GATT_NOTIFIABLE | BLE_GATT_INDICABLE)))
    return;

  if (chr->permissions & BLE_GATT_PERM_OTHER_READ)
    goto ok;

  if (client->encrypted && (chr->permissions & BLE_GATT_PERM_ENC_READ))
    goto ok;

  if (client->authenticated && (chr->permissions & BLE_GATT_PERM_AUTH_READ))
    goto ok;

  return;
 ok:

  client->handler->att_value_changed(client, service->start_handle + handle_index,
                                     sub->mode, data, size);
}

enum ble_att_error_e ble_gatt_client_register(struct ble_gatt_client_s *client,
                                              uint16_t mode)
{
  struct ble_gatt_db_service_s *srv
    = client->cursor.service;
  const struct ble_gatt_db_hndl_s *hndl
    = &srv->handle[client->cursor.handle - srv->start_handle];
  const struct ble_gatt_characteristic_s *chr
    = &srv->service->characteristic[hndl->index];

  if (!(chr->permissions & (BLE_GATT_NOTIFIABLE | BLE_GATT_INDICABLE)))
    return BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;

  if (!(chr->permissions & BLE_GATT_PERM_OTHER_READ)) {
    if (!client->encrypted && (chr->permissions & BLE_GATT_PERM_ENC_READ))
      return BLE_ATT_ERR_INSUF_ENCRYPTION;

    if (!client->authenticated && (chr->permissions & BLE_GATT_PERM_AUTH_READ))
      return BLE_ATT_ERR_INSUF_AUTHENTICATION;
  }

  GCT_FOREACH(ble_gatt_client_sl, &client->subs_list, sub,
              if (sub->listener.chr_index == hndl->index
                  && sub->listener.service == srv) {
                if (mode) {
                  sub->mode = mode;
                } else {
                  ble_gatt_client_sl_remove(&client->subs_list, sub);
                  ble_gatt_db_listener_unregister(sub->listener.service, &sub->listener);
                  mem_free(sub);
                }
                client->handler->att_subscription_changed(client);
                return 0;
              }
              );

  if (!mode)
    return 0;

  struct ble_gatt_client_subs_s *sub = mem_alloc(sizeof(*sub), mem_scope_sys);

  if (!sub)
    return BLE_ATT_ERR_UNLIKELY_ERROR;

  sub->client = client;
  sub->mode = mode;

  ble_gatt_db_listener_register(srv, &sub->listener,
                                ble_gatt_client_value_changed_handler,
                                hndl->index);
  ble_gatt_client_sl_pushback(&client->subs_list, sub);
  client->handler->att_subscription_changed(client);

  return 0;
}

uint16_t ble_gatt_client_register_mode_get(
  struct ble_gatt_client_s *client)
{
  struct ble_gatt_db_service_s *srv
    = client->cursor.service;
  const struct ble_gatt_db_hndl_s *hndl
    = &srv->handle[client->cursor.handle - srv->start_handle];

  GCT_FOREACH(ble_gatt_client_sl, &client->subs_list, sub,
              if (sub->listener.chr_index == hndl->index
                  && sub->listener.service == srv)
                return sub->mode;
              );

  return 0;
}

static
int_fast8_t subscription_cmp(const void *a, const void *b)
{
  return memcmp(a, b, sizeof(struct ble_subscription_s));
}

void ble_gatt_client_subscription_get(struct ble_gatt_client_s *client,
                                      struct ble_subscription_s *subscriptions,
                                      size_t count)
{
  struct ble_gatt_client_subs_s *sub;
  size_t i = 0;

  printk("Saving subscriptions:\n");

  for (sub = ble_gatt_client_sl_head(&client->subs_list);
       sub && i < count;
       sub = ble_gatt_client_sl_next(&client->subs_list, sub)) {
    subscriptions[i].service_index = sub->listener.service->index;
    subscriptions[i].char_index = sub->listener.chr_index;
    subscriptions[i].mode = sub->mode == 2;

    printk(" srv %d chr %d mode %d\n",
           subscriptions[i].service_index,
           subscriptions[i].char_index,
           subscriptions[i].mode);

    i++;
  }

  qsort(subscriptions, i, sizeof(*subscriptions), subscription_cmp);

  memset(subscriptions + i, 0xff, sizeof(*subscriptions) * (count - i));

  printk(" -> %P\n", subscriptions, sizeof(*subscriptions) * count);
}

void ble_gatt_client_subscription_set(struct ble_gatt_client_s *client,
                                      const struct ble_subscription_s *subscriptions,
                                      size_t count)
{
  GCT_FOREACH(ble_gatt_client_sl, &client->subs_list, sub,
              ble_gatt_client_sl_remove(&client->subs_list, sub);
              ble_gatt_db_listener_unregister(sub->listener.service, &sub->listener);
              mem_free(sub);
              );

  assert(ble_gatt_client_sl_isempty(&client->subs_list));

  if (memcstcmp(subscriptions, 0xff, count) == 0)
    return;

  printk("Restoring subscriptions: %P\n", subscriptions, count);

  for (size_t i = 0; i < count; ++i) {
    struct ble_gatt_db_service_s *srv = ble_gatt_db_service_get_by_index(client->db,
                                                                         subscriptions[i].service_index);
    if (!srv)
      continue;

    GCT_FOREACH(ble_gatt_client_sl, &client->subs_list, ss,
                if (ss->listener.chr_index == subscriptions[i].char_index
                    && ss->listener.service == srv) {
                  ss->mode = 1 << subscriptions[i].mode;
                  goto next;
                });

    struct ble_gatt_client_subs_s *sub = mem_alloc(sizeof(*sub), mem_scope_sys);

    if (!sub)
      goto out;

    if (subscriptions[i].char_index >= srv->service->characteristic_count)
      continue;

    if (!(srv->service->characteristic[subscriptions[i].char_index].permissions
          & (BLE_GATT_NOTIFIABLE | BLE_GATT_INDICABLE)))
      continue;

    printk(" srv %d chr %d mode %d\n",
           subscriptions[i].service_index,
           subscriptions[i].char_index,
           subscriptions[i].mode);

    sub->client = client;
    sub->mode = 1 << subscriptions[i].mode;

    ble_gatt_db_listener_register(srv, &sub->listener,
                                  ble_gatt_client_value_changed_handler,
                                  subscriptions[i].char_index);
    ble_gatt_client_sl_pushback(&client->subs_list, sub);

  next:
    ;
  }

 out:
  client->handler->att_subscription_changed(client);
}
