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

#define LOGK_MODULE_ID "gatD"

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/buffer_pool.h>

#include <ble/gattdb/service.h>
#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gattdb/db.h>
#include <ble/gattdb/client.h>

#include <ble/protocol/gatt.h>
#include <ble/uuid.h>

#include <hexo/decls.h>

GCT_CONTAINER_FCNS(ble_gattdb_listener_list, static, ble_gattdb_listener_list,
                   init, destroy, pushback, remove);

BLE_GATTDB_SERVICE_DECL(gattdb_profile_service,
                        BLE_GATTDB_SERVICE_PRIMARY,
                        BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_GENERIC_ATTRIBUTE),
                        NULL,
                        BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_GATT_SERVICE_CHANGED),
                                        BLE_GATTDB_INDICABLE,
                                        BLE_GATTDB_CHAR_DATA_PLAIN(NULL, 0, NULL, NULL)),
                        );

error_t ble_gattdb_init(struct ble_gattdb_s *db)
{
  ble_gattdb_registry_list_init(&db->registry_list);

  return ble_gattdb_service_register(&db->gattdb_registry, db, &gattdb_profile_service);
}

void ble_gattdb_cleanup(struct ble_gattdb_s *db)
{
  ble_gattdb_registry_list_destroy(&db->registry_list);
}

void ble_gattdb_char_changed(struct ble_gattdb_registry_s *reg,
                              uint8_t charid, bool_t reliable,
                              const void *data, size_t size)
{
  GCT_FOREACH(ble_gattdb_listener_list, &reg->listener_list, item,
              if (item->chr_index == charid)
                item->value_changed(item, reg, reliable, data, size);
              );
}

#if defined(CONFIG_BLE_GATTDB_STREAM)
void ble_gattdb_char_stream_resume(struct ble_gattdb_registry_s *reg,
                                   uint8_t charid)
{
  ble_gattdb_char_changed(reg, charid, 0, 0, 0);
}
#endif

static void ble_gattdb_registry_changed(struct ble_gattdb_s *db,
                                     uint16_t start, uint16_t end)
{
  uint8_t data[4];

  endian_le16_na_store(data, start);
  endian_le16_na_store(data + 2, end);

  ble_gattdb_char_changed(&db->gattdb_registry, 0, 1, data, 4);
}


error_t ble_gattdb_service_register(struct ble_gattdb_registry_s *reg,
                                     struct ble_gattdb_s *db,
                                     const struct ble_gattdb_service_s *service)
{
  size_t att_count = 1, att;

#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  reg->include_count = 0;

  if (service->include) {
    while (service->include[reg->include_count]) {
      reg->include_count++;
      att_count++;
    }
  }
#endif

  for (uint8_t charid = 0; charid < service->characteristic_count; ++charid) {
    const struct ble_gattdb_characteristic_s *chr = &service->characteristic[charid];

    att_count += 2;

    if (chr->permissions & (BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_INDICABLE))
      att_count++; // For CCCD

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
    if (chr->permissions & BLE_GATTDB_BROADCASTABLE)
      att_count++; // For SCCD
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    if (chr->mode == BLE_GATTDB_CHARACTERISTIC_DYNAMIC_PREPARED)
      att_count++; // For CEP
#endif

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
    att_count += chr->descriptor_count;
#endif
  }

  reg->handle = mem_alloc(sizeof(reg->handle[0]) * att_count, mem_scope_sys);

  if (!reg->handle)
    return -ENOMEM;

  logk_trace("DB service " BLE_UUID_FMT " append %d attrs",
         BLE_UUID_ARG(service->type), att_count);

  ble_gattdb_listener_list_init(&reg->listener_list);
  reg->service = service;
  reg->handle_count = att_count;
  reg->db = db;

  if (ble_gattdb_registry_list_isempty(&db->registry_list)) {
    reg->start_handle = 1;
  } else {
    struct ble_gattdb_registry_s *last = ble_gattdb_registry_list_tail(&db->registry_list);

    reg->start_handle = last->start_handle + last->handle_count;
  }

  reg->index = 0;
  GCT_FOREACH(ble_gattdb_registry_list, &db->registry_list, s,
              if (reg->index <= s->index)
                reg->index = s->index + 1;
              );

  ble_gattdb_registry_list_pushback(&db->registry_list, reg);

  att = 0;

  reg->handle[att].type = BLE_GATTDB_HNDL_SERVICE;
  reg->handle[att].index = 0;
#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
  reg->handle[att].descriptor = 0;
#endif

  att++;

#if defined(CONFIG_BLE_GATTDB_INCLUDE)
  for (uint8_t srv = 0; srv < reg->include_count; ++srv) {
    reg->handle[att].type = BLE_GATTDB_HNDL_INCLUDE;
    reg->handle[att].index = srv;
    reg->handle[att].descriptor = 0;

    logk_debug("Include %d/%d: %p", srv, reg->include_count, reg->service->include[srv]);

    att++;
  }
#endif

  for (uint8_t charid = 0; charid < service->characteristic_count; ++charid) {
    const struct ble_gattdb_characteristic_s *chr = &service->characteristic[charid];

    reg->handle[att].type = BLE_GATTDB_HNDL_CHAR_TYPE;
    reg->handle[att].index = charid;

    att++;

    reg->handle[att].type = BLE_GATTDB_HNDL_CHAR_VALUE;
    reg->handle[att].index = charid;

    att++;

    if (chr->permissions & (BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_INDICABLE)) {
      reg->handle[att].type = BLE_GATTDB_HNDL_CCCD;
      reg->handle[att].index = charid;

      att++;
    }

#if defined(CONFIG_BLE_GATTDB_BROADCAST)
    if (chr->permissions & BLE_GATTDB_BROADCASTABLE) {
      reg->handle[att].type = BLE_GATTDB_HNDL_SCCD;
      reg->handle[att].index = charid;

      att++;
    }
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    if (chr->mode == BLE_GATTDB_CHARACTERISTIC_DYNAMIC_PREPARED) {
      reg->handle[att].type = BLE_GATTDB_HNDL_CEP;
      reg->handle[att].index = charid;

      att++;
    }
#endif

#if defined(CONFIG_BLE_GATTDB_DESCRIPTOR)
    for (uint8_t desc = 0; desc < chr->descriptor_count; desc++) {
      reg->handle[att].type = BLE_GATTDB_HNDL_CHAR_DESC;
      reg->handle[att].index = charid;
      reg->handle[att].descriptor = desc;

      att++;
    }
#endif
  }

  assert(att_count == att);

  ble_gattdb_registry_changed(db,
                           reg->start_handle,
                           reg->start_handle + reg->handle_count);

  return 0;
}

void ble_gattdb_service_unregister(struct ble_gattdb_registry_s *reg)
{
  struct ble_gattdb_s *db = reg->db;
  struct ble_gattdb_registry_s *last;
  uint16_t last_handle;

  last = ble_gattdb_registry_list_tail(&db->registry_list);
  last_handle = last->start_handle + last->handle_count - 1;

  while (last && last != reg) {
    last->start_handle -= reg->handle_count;
    last = ble_gattdb_registry_list_prev(&db->registry_list, last);
  }

  ble_gattdb_registry_list_remove(&db->registry_list, reg);

  ble_gattdb_registry_changed(db, reg->start_handle, last_handle);

  ble_gattdb_listener_list_destroy(&reg->listener_list);

  mem_free(reg->handle);
}


struct ble_gattdb_registry_s *
ble_gattdb_registry_get_by_type(struct ble_gattdb_s *db,
                                const struct ble_uuid_s *type,
                                uint_fast8_t index)
{
  struct ble_gattdb_registry_s *s;

  for (s = ble_gattdb_registry_list_head(&db->registry_list);
       s;
       s = ble_gattdb_registry_list_next(&db->registry_list, s)) {
    if (ble_uuid_cmp(s->service->type, type))
      continue;

    if (index == 0)
      return s;

    --index;
  }

  return NULL;
}

struct ble_gattdb_registry_s *
ble_gattdb_registry_get_by_index(struct ble_gattdb_s *db,
    uint_fast8_t index)
{
  struct ble_gattdb_registry_s *s;

  for (s = ble_gattdb_registry_list_head(&db->registry_list);
       s;
       s = ble_gattdb_registry_list_next(&db->registry_list, s)) {
    if (s->index == index)
      return s;
  }

  return NULL;
}

struct ble_gattdb_registry_s *
ble_gattdb_registry_get_by_service(struct ble_gattdb_s *db,
                                   const struct ble_gattdb_service_s *service,
                                   uint_fast8_t index)
{
  struct ble_gattdb_registry_s *s;

  for (s = ble_gattdb_registry_list_head(&db->registry_list);
       s;
       s = ble_gattdb_registry_list_next(&db->registry_list, s)) {
    if (s->service != service)
      continue;

    if (index == 0)
      return s;

    --index;
  }

  return NULL;
}

static void listener_notify_if_0(struct ble_gattdb_registry_s *reg,
                                 uint8_t chr_index, bool_t subsc)
{
  size_t count = 0;
  const struct ble_gattdb_characteristic_s *chr = &reg->service->characteristic[chr_index];

  GCT_FOREACH(ble_gattdb_listener_list, &reg->listener_list, item,
              if (item->chr_index == chr_index)
                count++;
              );

  if (count == 0) {
    switch (chr->mode) {
    case BLE_GATTDB_CHARACTERISTIC_CONSTANT:
      break;

    case BLE_GATTDB_CHARACTERISTIC_PLAIN:
      if (chr->data.plain.on_subscribe) {
        chr->data.plain.on_subscribe(reg, chr_index, subsc);
      }
      break;

    case BLE_GATTDB_CHARACTERISTIC_DYNAMIC:
      if (chr->data.dynamic.on_subscribe) {
        chr->data.dynamic.on_subscribe(reg, chr_index, subsc);
      }
      break;

#if defined(CONFIG_BLE_GATTDB_STREAM)
    case BLE_GATTDB_CHARACTERISTIC_STREAM:
      if (chr->data.stream.on_subscribe) {
        chr->data.stream.on_subscribe(reg, chr_index, subsc);
      }
      break;
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    case BLE_GATTDB_CHARACTERISTIC_DYNAMIC_PREPARED:
      if (chr->data.dynamic_prepared.on_subscribe)
        chr->data.dynamic_prepared.on_subscribe(reg, chr_index, subsc);
      break;
#endif
    }
  }
}

void ble_gattdb_listener_register(struct ble_gattdb_registry_s *reg,
                                   struct ble_gattdb_listener_s *listener,
                                   ble_gattdb_registry_value_changed_func_t *func,
                                   uint8_t chr_index)
{
  listener->value_changed = func;
  listener->registry = reg;
  listener->chr_index = chr_index;

  listener_notify_if_0(reg, chr_index, 1);

  ble_gattdb_listener_list_pushback(&reg->listener_list, listener);
}

void ble_gattdb_listener_unregister(struct ble_gattdb_registry_s *reg,
                                     struct ble_gattdb_listener_s *listener)
{
  ble_gattdb_listener_list_remove(&reg->listener_list, listener);

  listener_notify_if_0(reg, listener->chr_index, 0);
}

error_t ble_gattdb_std_char_read(struct ble_gattdb_s *db,
                                  uint16_t srv_uuid16, uint16_t chr_uuid16,
                                  const void **data, size_t *size)
{
  GCT_FOREACH(ble_gattdb_registry_list, &db->registry_list, s,
              if (!ble_uuid_is_uuid16(s->service->type) ||
                  ble_uuid_uuid16_get(s->service->type) != srv_uuid16)
                GCT_FOREACH_CONTINUE;

              for (uint8_t i = 0; i < s->service->characteristic_count; ++i) {
                const struct ble_gattdb_characteristic_s *chr = &s->service->characteristic[i];

                if (!ble_uuid_is_uuid16(chr->type) || ble_uuid_uuid16_get(chr->type) != chr_uuid16)
                  continue;

                switch (chr->mode) {
                case BLE_GATTDB_CHARACTERISTIC_CONSTANT:
                  *size = chr->data.constant.size;
                  *data = chr->data.constant.data;
                  return 0;

                case BLE_GATTDB_CHARACTERISTIC_PLAIN:
                  *size = chr->data.plain.size;
                  *data = chr->data.plain.data;
                  return 0;

                default:
                  return -ENOTSUP;
                }
              }
              return -ENOENT;
              );
  return -ENOENT;
}

size_t ble_gattdb_srv16_list_get(struct ble_gattdb_s *db,
    void *srvlist_, size_t size)
{
  uintptr_t srvlist = (uintptr_t)srvlist_;
  size_t off = 0;

  GCT_FOREACH(ble_gattdb_registry_list, &db->registry_list, s,
              if (!(s->service->flags & BLE_GATTDB_SERVICE_ADVERTISED))
                GCT_FOREACH_CONTINUE;

              if (!ble_uuid_is_uuid16(s->service->type))
                GCT_FOREACH_CONTINUE;

              if (off + 2 <= size)
                endian_le16_na_store((void*)(srvlist + off),
                                     ble_uuid_uuid16_get(s->service->type));
              off += 2;
              );

  return off;
}

size_t ble_gattdb_srv128_list_get(struct ble_gattdb_s *db,
    void *srvlist_, size_t size)
{
  uintptr_t srvlist = (uintptr_t)srvlist_;
  size_t off = 0;

  GCT_FOREACH(ble_gattdb_registry_list, &db->registry_list, s,
              if (!(s->service->flags & BLE_GATTDB_SERVICE_ADVERTISED))
                GCT_FOREACH_CONTINUE;

              if (ble_uuid_is_uuid16(s->service->type))
                GCT_FOREACH_CONTINUE;

              if (off + 16 <= size)
                memcpy((void*)(srvlist + off), (const void *)s->service->type, 16);
              off += 16;
              );

  return off;
}
