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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/buffer_pool.h>

#include <ble/gatt/service.h>
#include <ble/gatt/db.h>
#include <ble/gatt/client.h>

#include <ble/protocol/gatt.h>
#include <ble/uuid.h>

#include <hexo/decls.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

GCT_CONTAINER_FCNS(ble_gatt_listener_list, static, ble_gatt_listener_list,
                   init, destroy, pushback, remove);

static const struct ble_uuid_s ble_uuid_gatt_service = BLE_UUID_SHORT(0x1801);
static const struct ble_uuid_s ble_uuid_gatt_service_changed = BLE_UUID_SHORT(0x2a05);

static const struct ble_gatt_characteristic_s gatt_profile_characteristic_array[] = {
    {
      .mode = BLE_GATT_CHARACTERISTIC_PLAIN,
      .permissions = BLE_GATT_INDICABLE,
      .type = &ble_uuid_gatt_service,
    },
};

static const struct ble_gatt_service_s gatt_profile_service = {
    .type = &ble_uuid_gatt_service,
    .characteristic = gatt_profile_characteristic_array,
    .characteristic_count = ARRAY_SIZE(gatt_profile_characteristic_array),
    .primary = 1,
};

error_t ble_gatt_db_init(struct ble_gatt_db_s *db)
{
  ble_gatt_service_list_init(&db->service_list);

  return ble_gatt_db_service_register(&db->gatt_service, db, &gatt_profile_service);
}

void ble_gatt_db_cleanup(struct ble_gatt_db_s *db)
{
  ble_gatt_service_list_destroy(&db->service_list);
}

void ble_gatt_db_char_changed(struct ble_gatt_db_s *db,
                              struct ble_gatt_db_service_s *dbs,
                              uint8_t charid,
                              const void *data, size_t size)
{
  GCT_FOREACH(ble_gatt_listener_list, &dbs->listener_list, item,
              if (item->chr_index == charid)
                item->handler(item, dbs, data, size);
              );
}

static void ble_gatt_service_changed(struct ble_gatt_db_s *db,
                                     uint16_t start, uint16_t end)
{
  uint8_t data[4];

  endian_le16_na_store(data, start);
  endian_le16_na_store(data + 2, end);

  ble_gatt_db_char_changed(db, &db->gatt_service, 0, data, 4);
}


error_t ble_gatt_db_service_register(struct ble_gatt_db_service_s *dbs,
                                     struct ble_gatt_db_s *db,
                                     const struct ble_gatt_service_s *service)
{
  size_t att_count = 1, att;

#if defined(CONFIG_BLE_GATT_INCLUDE)
  att_count += service->include_count;
#endif

  for (uint8_t charid = 0; charid < service->characteristic_count; ++charid) {
    const struct ble_gatt_characteristic_s *chr = &service->characteristic[charid];

    att_count += 2;

    if (chr->permissions & (BLE_GATT_NOTIFIABLE | BLE_GATT_INDICABLE))
      att_count++; // For CCCD

#if defined(CONFIG_BLE_GATT_BROADCAST)
    if (chr->permissions & BLE_GATT_BROADCASTABLE)
      att_count++; // For SCCD
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    if (chr->mode == BLE_GATT_CHARACTERISTIC_DYNAMIC_PREPARED)
      att_count++; // For CEP
#endif

#if defined(CONFIG_BLE_GATT_DESCRIPTOR)
    att_count += chr->descriptor_count;
#endif
  }

  dbs->handle = mem_alloc(sizeof(dbs->handle[0]) * att_count, mem_scope_sys);

  if (!dbs->handle)
    return -ENOMEM;

  dprintk("DB service " BLE_UUID_FMT " append %d attrs\n",
         BLE_UUID_ARG(service->type), att_count);

  ble_gatt_listener_list_init(&dbs->listener_list);
  dbs->service = service;
  dbs->handle_count = att_count;
  dbs->db = db;

  if (ble_gatt_service_list_isempty(&db->service_list)) {
    dbs->start_handle = 1;
  } else {
    struct ble_gatt_db_service_s *last = ble_gatt_service_list_tail(&db->service_list);

    dbs->start_handle = last->start_handle + last->handle_count;
  }

  ble_gatt_service_list_pushback(&db->service_list, dbs);

  att = 0;

  dbs->handle[att].type = BLE_GATT_HNDL_SERVICE;
  dbs->handle[att].index = 0;
#if defined(CONFIG_BLE_GATT_DESCRIPTOR)
  dbs->handle[att].descriptor = 0;
#endif

  att++;

#if defined(CONFIG_BLE_GATT_INCLUDE)
  for (uint8_t srv = 0; srv < service->include_count; ++srv) {
    dbs->handle[att].type = BLE_GATT_HNDL_INCLUDE;
    dbs->handle[att].index = srv;
    dbs->handle[att].descriptor = 0;

    att++;
  }
#endif

  for (uint8_t charid = 0; charid < service->characteristic_count; ++charid) {
    const struct ble_gatt_characteristic_s *chr = &service->characteristic[charid];

    dbs->handle[att].type = BLE_GATT_HNDL_CHAR_TYPE;
    dbs->handle[att].index = charid;

    att++;

    dbs->handle[att].type = BLE_GATT_HNDL_CHAR_VALUE;
    dbs->handle[att].index = charid;

    att++;

    if (chr->permissions & (BLE_GATT_NOTIFIABLE | BLE_GATT_INDICABLE)) {
      dbs->handle[att].type = BLE_GATT_HNDL_CCCD;
      dbs->handle[att].index = charid;

      att++;
    }

#if defined(CONFIG_BLE_GATT_BROADCAST)
    if (chr->permissions & BLE_GATT_BROADCASTABLE) {
      dbs->handle[att].type = BLE_GATT_HNDL_SCCD;
      dbs->handle[att].index = charid;

      att++;
    }
#endif

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    if (chr->mode == BLE_GATT_CHARACTERISTIC_DYNAMIC_PREPARED) {
      dbs->handle[att].type = BLE_GATT_HNDL_CEP;
      dbs->handle[att].index = charid;

      att++;
    }
#endif

#if defined(CONFIG_BLE_GATT_DESCRIPTOR)
    for (uint8_t desc = 0; desc < chr->descriptor_count; desc++) {
      dbs->handle[att].type = BLE_GATT_HNDL_CHAR_DESC;
      dbs->handle[att].index = charid;
      dbs->handle[att].descriptor = desc;

      att++;
    }
#endif
  }

  assert(att_count == att);

  ble_gatt_service_changed(db,
                           dbs->start_handle,
                           dbs->start_handle + dbs->handle_count);

  return 0;
}

void ble_gatt_db_service_unregister(struct ble_gatt_db_service_s *dbs)
{
  struct ble_gatt_db_s *db = dbs->db;
  struct ble_gatt_db_service_s *last;
  uint16_t last_handle;

  if (ble_gatt_service_list_isempty(&db->service_list))
    return;

  last = ble_gatt_service_list_tail(&db->service_list);
  last_handle = last->start_handle + last->handle_count - 1;

  while (last && last != dbs) {
    last->start_handle -= dbs->handle_count;
    last = ble_gatt_service_list_prev(&db->service_list, last);
  }

  ble_gatt_service_list_remove(&db->service_list, dbs);

  ble_gatt_service_changed(db, dbs->start_handle, last_handle);

  ble_gatt_listener_list_destroy(&dbs->listener_list);

  mem_free(dbs->handle);
}


struct ble_gatt_db_service_s *
ble_gatt_db_service_get_by_type(struct ble_gatt_db_s *db,
                                const struct ble_uuid_s *type,
                                uint8_t index)
{
  struct ble_gatt_db_service_s *s;

  for (s = ble_gatt_service_list_head(&db->service_list);
       s;
       s = ble_gatt_service_list_next(&db->service_list, s)) {
    if (ble_uuid_cmp(s->service->type, type))
      continue;

    if (index == 0)
      return s;

    --index;
  }

  return NULL;
}

static void listener_notify_if_0(struct ble_gatt_db_service_s *dbs,
                                 uint8_t chr_index, bool_t subsc)
{
  size_t count = 0;
  const struct ble_gatt_characteristic_s *chr = &dbs->service->characteristic[chr_index];

  GCT_FOREACH(ble_gatt_listener_list, &dbs->listener_list, item,
              if (item->chr_index == chr_index)
                count++;
              );

  if (count == 0) {
    switch (chr->mode) {
    default:
      break;

    case BLE_GATT_CHARACTERISTIC_PLAIN:
      if (chr->data.plain.on_subscribe) {
        chr->data.plain.on_subscribe(dbs, chr_index, subsc);
      }
      break;

    case BLE_GATT_CHARACTERISTIC_DYNAMIC:
      if (chr->data.dynamic.on_subscribe) {
        chr->data.dynamic.on_subscribe(dbs, chr_index, subsc);
      }
      break;

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
    case BLE_GATT_CHARACTERISTIC_DYNAMIC_PREPARED:
      if (chr->data.dynamic_prepared.on_subscribe)
        chr->data.dynamic_prepared.on_subscribe(dbs, chr_index, subsc);
      break;
#endif
    }
  }
}

void ble_gatt_db_listener_register(struct ble_gatt_db_service_s *dbs,
                                   struct ble_gatt_db_listener_s *listener,
                                   ble_gatt_db_service_handler_func_t *func,
                                   uint8_t chr_index)
{
  listener->handler = func;
  listener->service = dbs;
  listener->chr_index = chr_index;

  listener_notify_if_0(dbs, chr_index, 1);

  ble_gatt_listener_list_pushback(&dbs->listener_list, listener);
}

void ble_gatt_db_listener_unregister(struct ble_gatt_db_service_s *dbs,
                                     struct ble_gatt_db_listener_s *listener)
{
  ble_gatt_listener_list_remove(&dbs->listener_list, listener);

  listener_notify_if_0(dbs, listener->chr_index, 0);
}
