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

#define LOGK_MODULE_ID "bscn"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/scan_filter.h>
#include <ble/net/generic.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/advertise.h>
#include <ble/security_db.h>
#include <ble/peer.h>

#include <gct_platform.h>
#include <gct/container_avl_p.h>

#define GCT_CONTAINER_ALGO_ble_scan_list AVL_P

struct ble_scan_item_s
{
  GCT_CONTAINER_ENTRY(ble_scan_list, entry);

  uint8_t adv_ad_len;
  uint8_t scan_ad_len;
  bool_t rescan;
  struct ble_scan_filter_device_s device;
  enum ble_scan_filter_policy_e policy;
};

GCT_CONTAINER_TYPES(ble_scan_list, struct ble_scan_item_s *, entry);
GCT_CONTAINER_KEY_TYPES(ble_scan_list, CUSTOM, BLOB, &ble_scan_list_item->device.addr, addr, sizeof(struct ble_addr_s));
GCT_CONTAINER_KEY_FCNS(ble_scan_list, ASC, static, ble_scan_list, addr,
                       init, destroy, remove, insert, lookup);

/**
   BLE scanner layer
 */
struct ble_scan_filter_s
{
  struct net_layer_s layer;

  ble_scan_list_root_t devices;
  dev_timer_delay_t lifetime_tk;
  uint32_t next_id;

  struct ble_scanner_param_s scan_params;

  struct net_task_s cleanup_task;

  struct ble_security_db_s *peerdb;
};

STRUCT_COMPOSE(ble_scan_filter_s, cleanup_task);
STRUCT_COMPOSE(ble_scan_filter_s, layer);

static void ble_scan_filter_cleanup_later(struct ble_scan_filter_s *sf);

static
void ble_scan_filter_destroyed(struct net_layer_s *layer)
{
  struct ble_scan_filter_s *sf = ble_scan_filter_s_from_layer(layer);

  GCT_FOREACH(ble_scan_list, &sf->devices, item,
              ble_scan_list_remove(&sf->devices, item);
              mem_free(item);
              );

  ble_scan_list_destroy(&sf->devices);

  logk_trace("Scan filter %p destroyed", sf);

  mem_free(sf);
}

static
struct ble_scan_item_s *ble_scan_filter_device_get(struct ble_scan_filter_s *sf,
                                                   const struct ble_addr_s *addr,
                                                   bool_t create)
{
  struct ble_scan_item_s *item;

  item = ble_scan_list_lookup(&sf->devices, addr);
  if (item || !create)
    return item;

  item = mem_alloc(sizeof(*item), mem_scope_sys);
  if (!item)
    return item;

  memset(item, 0, sizeof(*item));

  item->device.id = sf->next_id++;
  item->device.addr = *addr;
  item->device.active = 1;
  item->device.first_seen = net_scheduler_time_get(sf->layer.scheduler);
  ble_scan_list_insert(&sf->devices, item);
  item->policy = BLE_SCAN_FILTER_MONITOR;
  item->device.known = ble_security_db_contains(sf->peerdb, addr);

  ble_scan_filter_cleanup_later(sf);

  return item;
}

static
bool_t ble_scan_filter_address_resolve(struct ble_scan_filter_s *sf,
                                       struct ble_addr_s *adva)
{
  struct ble_peer_s peer;
  error_t err;

  err = ble_security_db_load(sf->peerdb, adva, &peer);
  if (err)
    return 0;

  if (!peer.identity_present)
    return 0;

  logk_trace("Resolved address "BLE_ADDR_FMT" to "BLE_ADDR_FMT"",
         BLE_ADDR_ARG(adva),
         BLE_ADDR_ARG(&peer.addr));

  *adva = peer.addr;
  return 1;
}

static
void ble_scan_filter_adv_handle(struct ble_scan_filter_s *sf,
                                struct buffer_s *buffer, int16_t rssi)
{
  struct ble_scan_item_s *item;
  struct ble_addr_s adva;
  const uint8_t *ad = buffer->data + buffer->begin + 8;
  size_t ad_len = buffer->end - buffer->begin - 8;
  bool_t create = 0;
  enum ble_scanner_policy_e scan_policy;
  const struct ble_scan_filter_delegate_vtable_s *vtable
    = const_ble_scan_filter_delegate_vtable_s_from_base(sf->layer.delegate_vtable);

  if (ad_len > 31)
    return;

  switch (ble_advertise_packet_type_get(buffer)) {
  case BLE_ADV_IND:
  case BLE_ADV_NONCONN_IND:
  case BLE_ADV_DIRECT_IND:
    create = 1;
    break;

  case BLE_SCAN_RSP:
  case BLE_SCAN_REQ:
  case BLE_CONNECT_REQ:
  case BLE_ADV_SCAN_IND:
    break;
  }

  logk_trace("Got adv %p %P", item,
         buffer->data + buffer->begin,
         buffer->end - buffer->begin);

  ble_advertise_packet_txaddr_get(buffer, &adva);

  if (adva.type == BLE_ADDR_RANDOM
      && ble_addr_random_type(&adva) == BLE_ADDR_RANDOM_RESOLVABLE) {
    logk_trace("From random "BLE_ADDR_FMT"", BLE_ADDR_ARG(&adva));

    if (!ble_scan_filter_address_resolve(sf, &adva))
      return;
  }

  item = ble_scan_filter_device_get(sf, &adva, create);
  if (!item)
    return;

  item->device.last_seen = net_scheduler_time_get(sf->layer.scheduler);
  item->device.rssi = rssi;

  switch (ble_advertise_packet_type_get(buffer)) {
  case BLE_ADV_IND:
  case BLE_ADV_NONCONN_IND:
    if (item->device.connectable
        && ad_len == item->adv_ad_len
        && !memcmp(item->device.ad, ad, ad_len)
        && item->policy != BLE_SCAN_FILTER_MONITOR)
      return;

    item->device.connectable = ble_advertise_packet_type_get(buffer) == BLE_ADV_IND;

    if (ad_len != item->adv_ad_len && item->scan_ad_len)
      memmove(item->device.ad + ad_len, item->device.ad + item->adv_ad_len, item->scan_ad_len);
    memcpy(item->device.ad, ad, ad_len);
    item->adv_ad_len = ad_len;
    item->device.ad_len = ad_len + item->scan_ad_len;
    break;
    
  case BLE_ADV_DIRECT_IND:
    item->device.connectable = 1;
    item->device.scanned = 1;
    break;

  case BLE_SCAN_RSP:
    if (item->device.scanned
        && ad_len == item->scan_ad_len
        && !memcmp(item->device.ad + item->adv_ad_len, ad, ad_len)
        && item->policy != BLE_SCAN_FILTER_MONITOR) {
      item->rescan = 0;
      return;
    }

    item->device.connectable = 1;
    item->device.scanned = 1;
    item->rescan = 0;

    memcpy(item->device.ad + item->adv_ad_len, ad, ad_len);
    item->scan_ad_len = ad_len;
    item->device.ad_len = ad_len + item->adv_ad_len;
    break;

  case BLE_SCAN_REQ:
  case BLE_CONNECT_REQ:
  case BLE_ADV_SCAN_IND:
    return;
  }

  if (item->policy == BLE_SCAN_FILTER_IGNORE)
    return;

  item->policy = vtable->device_updated(sf->layer.delegate, &sf->layer, &item->device);

  if (!sf->layer.parent)
    return;

  switch (item->policy) {
  case BLE_SCAN_FILTER_IGNORE:
    scan_policy = BLE_SCANNER_IGNORE;
    break;
    
  case BLE_SCAN_FILTER_SCAN:
    if (item->device.scanned && !item->rescan)
      scan_policy = BLE_SCANNER_IGNORE;
    else
      scan_policy = BLE_SCANNER_SCAN;
    break;

  case BLE_SCAN_FILTER_CONNECT:
    scan_policy = BLE_SCANNER_CONNECT;
    break;

  default:
  case BLE_SCAN_FILTER_MONITOR:
    return;
  }

  for (size_t i = 0; i < sf->scan_params.target_count; ++i) {
    if (ble_addr_cmp(&adva, &sf->scan_params.target[i].addr))
      continue;

    if (scan_policy == sf->scan_params.default_policy) {
      if (i < sf->scan_params.target_count - 1)
        memmove(sf->scan_params.target + i, sf->scan_params.target + i + 1,
                sizeof(struct ble_scanner_target_s) * (sf->scan_params.target_count - i - 1));
      sf->scan_params.target_count--;
    } else {
      if (sf->scan_params.target[i].policy == scan_policy)
        return;

      sf->scan_params.target[i].policy = scan_policy;
    }
    goto params_update;
  }

  if (scan_policy != sf->scan_params.default_policy) {
    size_t moved_count = __MIN(BLE_SCANNER_TARGET_MAXCOUNT - 1, sf->scan_params.target_count);
    memmove(sf->scan_params.target + 1, sf->scan_params.target, sizeof(struct ble_scanner_target_s) * moved_count);
    sf->scan_params.target_count = moved_count + 1;
    sf->scan_params.target[0].addr = adva;
    sf->scan_params.target[0].policy = scan_policy;
  }

 params_update:
#if 0
  logk_trace("New policy table (default %d)", sf->scan_params.default_policy);
  for (size_t i = 0; i < sf->scan_params.target_count; ++i)
    logk_trace(" "BLE_ADDR_FMT": %d", BLE_ADDR_ARG(&sf->scan_params.target[i].addr), sf->scan_params.target[i].policy);
#endif

  ble_scanner_params_update(sf->layer.parent, &sf->scan_params);
}

static void ble_scan_filter_cleanup_task_destroy(void *task_)
{
  struct net_task_s *task = task_;
  task->destroy_func = NULL;
}

static void ble_scan_filter_cleanup_later(struct ble_scan_filter_s *sf)
{
  if (sf->cleanup_task.destroy_func)
    return;

  if (!sf->layer.parent)
    return;

  sf->cleanup_task.destroy_func = ble_scan_filter_cleanup_task_destroy;

  net_task_timeout_push(&sf->cleanup_task, &sf->layer,
                        net_scheduler_time_get(sf->layer.scheduler) + sf->lifetime_tk / 2, 0);
}

static void ble_scan_filter_cleanup(struct ble_scan_filter_s *sf)
{
  const struct ble_scan_filter_delegate_vtable_s *vtable
    = const_ble_scan_filter_delegate_vtable_s_from_base(sf->layer.delegate_vtable);

  net_task_destroy(&sf->cleanup_task);

  dev_timer_value_t last_seen_max = net_scheduler_time_get(sf->layer.scheduler) - sf->lifetime_tk;

  GCT_FOREACH(ble_scan_list, &sf->devices, item,
              if (item->device.last_seen < last_seen_max) {
                ble_scan_list_remove(&sf->devices, item);
                item->device.active = 0;
                vtable->device_updated(sf->layer.delegate, &sf->layer, &item->device);
                mem_free(item);
              } else {
                item->rescan = 1;
              });

  ble_scan_filter_cleanup_later(sf);
}

static
void ble_scan_filter_task_handle(struct net_layer_s *layer,
                                 struct net_task_s *task)
{
  struct ble_scan_filter_s *sf = ble_scan_filter_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_INBOUND:
    ble_scan_filter_adv_handle(sf, task->packet.buffer, task->packet.src_addr.rssi);
    break;

  case NET_TASK_TIMEOUT:
    if (task == &sf->cleanup_task) {
      ble_scan_filter_cleanup(sf);
      return;
    }
      
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static void ble_scan_filter_dandling(struct net_layer_s *layer)
{
  struct ble_scan_filter_s *sf = ble_scan_filter_s_from_layer(layer);

  logk_trace("Scan filter %p dandling", sf);

  net_scheduler_from_layer_cancel(sf->layer.scheduler, &sf->layer);

  GCT_FOREACH(ble_scan_list, &sf->devices, item,
              ble_scan_list_remove(&sf->devices, item);
              mem_free(item);
              );
}

static const struct net_layer_handler_s scan_filter_handler = {
  .destroyed = ble_scan_filter_destroyed,
  .task_handle = ble_scan_filter_task_handle,
  .dandling = ble_scan_filter_dandling,
};

error_t ble_scan_filter_create(struct net_scheduler_s *scheduler,
                               const void *params_,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable,
                               struct net_layer_s **layer)
{
  struct ble_scan_filter_s *sf = mem_alloc(sizeof(*sf), mem_scope_sys);
  const struct ble_scan_filter_param_s *params = params_;
  error_t err;

  if (!sf)
    return -ENOMEM;

  memset(sf, 0, sizeof(*sf));

  err = net_layer_init(&sf->layer, &scan_filter_handler, scheduler, delegate, delegate_vtable);
  if (err)
    goto err_free;

  ble_scan_list_init(&sf->devices);

  memcpy(&sf->scan_params, params, sizeof(sf->scan_params));
  sf->scan_params.default_policy = BLE_SCANNER_IGNORE;

  dev_timer_init_sec(&scheduler->timer, &sf->lifetime_tk, NULL, 10, 1);
  sf->peerdb = params->peerdb;
  sf->scan_params = params->scan_params;

  *layer = &sf->layer;

  return 0;

 err_free:

  mem_free(sf);

  return err;
}
