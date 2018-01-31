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

#include <ble/net/scan_filter.h>
#include <ble/net/generic.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/advertise.h>
#include <ble/security_db.h>
#include <ble/peer.h>

#include <gct_platform.h>
#include <gct/container_avl_p.h>
#include <gct/container_clist.h>

//#define dprintk printk
#define dprintk(...) do{}while(0)

#define GCT_CONTAINER_ALGO_dev_list_by_addr AVL_P
#define GCT_CONTAINER_ALGO_dev_list_by_time CLIST

struct ble_scan_item_s
{
  GCT_CONTAINER_ENTRY(dev_list_by_addr, by_addr);
  GCT_CONTAINER_ENTRY(dev_list_by_time, by_time);

  uint8_t adv_ad_len;
  uint8_t scan_ad_len;
  dev_timer_value_t last_scan;
  struct ble_scan_filter_device_s device;
};

GCT_CONTAINER_TYPES(dev_list_by_addr, struct ble_scan_item_s *, by_addr);
GCT_CONTAINER_TYPES(dev_list_by_time, struct ble_scan_item_s *, by_time);
GCT_CONTAINER_KEY_TYPES(dev_list_by_addr, CUSTOM, BLOB,
                        &dev_list_by_addr_item->device.addr, addr_key_fcn,
                        sizeof(struct ble_addr_s));
GCT_CONTAINER_KEY_FCNS(dev_list_by_addr, ASC, ALWAYS_INLINE,
                       dev_list_by_addr, addr_key_fcn,
                       init, destroy, remove, insert, lookup);
GCT_CONTAINER_FCNS(dev_list_by_time, ALWAYS_INLINE, dev_list_by_time,
                   init, destroy, remove, push);

/**
   BLE scanner layer
 */
struct ble_scan_filter_s
{
  struct net_layer_s layer;

  dev_list_by_addr_root_t devices_by_addr;
  dev_list_by_time_root_t devices_by_time;
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

  GCT_FOREACH(dev_list_by_addr, &sf->devices_by_addr, item,
              dev_list_by_addr_remove(&sf->devices_by_addr, item);
              dev_list_by_time_remove(&sf->devices_by_time, item);
              mem_free(item);
              );

  dev_list_by_addr_destroy(&sf->devices_by_addr);
  dev_list_by_time_destroy(&sf->devices_by_time);

  dprintk("Scan filter %p destroyed\n", sf);

  mem_free(sf);
}

static
struct ble_scan_item_s *ble_scan_filter_device_get(struct ble_scan_filter_s *sf,
                                                   const struct ble_addr_s *addr,
                                                   bool_t create)
{
  struct ble_scan_item_s *item;

  item = dev_list_by_addr_lookup(&sf->devices_by_addr, addr);
  if (item) {
    dev_list_by_time_remove(&sf->devices_by_time, item);
    item->device.active = 1;
    item->device.last_seen = net_scheduler_time_get(sf->layer.scheduler);
    dev_list_by_time_push(&sf->devices_by_time, item);

    return item;
  }

  if (!create)
    return NULL;

  item = mem_alloc(sizeof(*item), mem_scope_sys);
  if (!item)
    return item;

  memset(item, 0, sizeof(*item));

  item->device.id = sf->next_id++;
  item->device.addr = *addr;
  item->device.active = 1;
  item->device.first_seen = net_scheduler_time_get(sf->layer.scheduler);
  item->device.last_seen = net_scheduler_time_get(sf->layer.scheduler);
  item->last_scan = 0;
  dev_list_by_addr_insert(&sf->devices_by_addr, item);
  dev_list_by_time_push(&sf->devices_by_time, item);
  item->device.policy = BLE_SCANNER_MONITOR;
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

  dprintk("Resolved address "BLE_ADDR_FMT" to "BLE_ADDR_FMT"\n",
         BLE_ADDR_ARG(adva),
         BLE_ADDR_ARG(&peer.addr));

  *adva = peer.addr;
  return 1;
}

static void scan_filter_params_update(struct ble_scan_filter_s *sf)
{
  bool_t changed = 0;
  size_t target_count = 0;

  if (!sf->layer.parent)
    return;

  dprintk("%s\n", __func__);
  
  GCT_FOREACH(dev_list_by_time, &sf->devices_by_time, item, {
      if (item->device.policy == sf->scan_params.default_policy)
        GCT_FOREACH_CONTINUE;
      
      if (ble_addr_cmp(&item->device.addr, &sf->scan_params.target[target_count].addr)
          || sf->scan_params.target[target_count].policy != item->device.policy)
        changed = 1;

      sf->scan_params.target[target_count].addr = item->device.addr;
      sf->scan_params.target[target_count].policy = item->device.policy;

      dprintk(" - "BLE_ADDR_FMT", %d\n",
             BLE_ADDR_ARG(&sf->scan_params.target[target_count].addr),
             sf->scan_params.target[target_count].policy);

      target_count++;

      if (target_count >= BLE_SCANNER_TARGET_MAXCOUNT)
        GCT_FOREACH_BREAK;
    });

  if (sf->scan_params.target_count != target_count)
    changed = 1;

  if (!changed)
    return;

  sf->scan_params.target_count = target_count;

  ble_scanner_params_update(sf->layer.parent, &sf->scan_params);
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

  if (ad_len > 31) {
    dprintk("Bad AD len: %d\n", ad_len);
    return;
  }
  
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

  ble_advertise_packet_txaddr_get(buffer, &adva);

  if (adva.type == BLE_ADDR_RANDOM
      && ble_addr_random_type(&adva) == BLE_ADDR_RANDOM_RESOLVABLE) {
    //dprintk("From random "BLE_ADDR_FMT" %02x\n", BLE_ADDR_ARG(&adva),
    //        ble_advertise_packet_type_get(buffer));

    if (!ble_scan_filter_address_resolve(sf, &adva))
      return;
  }

  item = ble_scan_filter_device_get(sf, &adva, create);

  dprintk("Got adv %p %P\n", item,
         buffer->data + buffer->begin,
         buffer->end - buffer->begin);

  if (!item)
    return;

  item->device.rssi = rssi;

  dprintk(""BLE_ADDR_FMT" %d %02x %d\n", BLE_ADDR_ARG(&item->device.addr), item->device.rssi,
         ble_advertise_packet_type_get(buffer), item->device.policy);

  switch (ble_advertise_packet_type_get(buffer)) {
  case BLE_ADV_IND:
  case BLE_ADV_NONCONN_IND:
    if (item->device.connectable
        && ad_len == item->adv_ad_len
        && !memcmp(item->device.ad, ad, ad_len)
        && item->device.policy != BLE_SCANNER_MONITOR)
      return;

    item->device.connectable = ble_advertise_packet_type_get(buffer) == BLE_ADV_IND;

    memcpy(item->device.ad, ad, ad_len);
    item->adv_ad_len = ad_len;
    item->device.ad_len = ad_len;
    item->last_scan = 0;
    break;
    
  case BLE_ADV_DIRECT_IND:
    item->device.connectable = 1;
    item->device.scanned = 1;
    break;

  case BLE_SCAN_RSP:
    item->last_scan = net_scheduler_time_get(sf->layer.scheduler);

    if (item->device.scanned && !(item->device.policy & BLE_SCANNER_MONITOR))
      return;

    item->device.connectable = 1;
    item->device.scanned = 1;

    memcpy(item->device.ad + item->adv_ad_len, ad, ad_len);
    item->scan_ad_len = ad_len;
    item->device.ad_len = ad_len + item->adv_ad_len;
    break;

  case BLE_SCAN_REQ:
  case BLE_CONNECT_REQ:
  case BLE_ADV_SCAN_IND:
    return;
  }

  dprintk("Scan filter policy %d\n", item->device.policy);

  if (item->device.policy == BLE_SCANNER_IGNORE)
    return;

  scan_policy = vtable->device_updated(sf->layer.delegate, &sf->layer, &item->device);

  if (scan_policy == BLE_SCANNER_SCAN && item->device.scanned && item->last_scan)
    scan_policy = BLE_SCANNER_MONITOR;

  if (item->device.policy != scan_policy) {
    item->device.policy = scan_policy;
    scan_filter_params_update(sf);
  }
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
  bool_t changed = 0;

  net_task_destroy(&sf->cleanup_task);

  dprintk("%s\n", __func__);

  dev_timer_value_t now = net_scheduler_time_get(sf->layer.scheduler);
  dev_timer_value_t last_seen_min = now - sf->lifetime_tk;

  GCT_FOREACH(dev_list_by_time, &sf->devices_by_time, item,
              if (item->device.last_seen < last_seen_min) {
                dev_list_by_addr_remove(&sf->devices_by_addr, item);
                dev_list_by_time_remove(&sf->devices_by_time, item);
                item->device.active = 0;
                changed = item->device.policy != sf->scan_params.default_policy;
                vtable->device_updated(sf->layer.delegate, &sf->layer, &item->device);
                mem_free(item);
              } else if (item->last_scan < last_seen_min) {
                item->last_scan = 0;
              });

  if (changed)
    scan_filter_params_update(sf);
  
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

  dprintk("Scan filter %p dandling\n", sf);

  net_scheduler_from_layer_cancel(sf->layer.scheduler, &sf->layer);

  GCT_FOREACH(dev_list_by_time, &sf->devices_by_time, item,
              dev_list_by_addr_remove(&sf->devices_by_addr, item);
              dev_list_by_time_remove(&sf->devices_by_time, item);
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

  dev_list_by_addr_init(&sf->devices_by_addr);
  dev_list_by_time_init(&sf->devices_by_time);

  memcpy(&sf->scan_params, params, sizeof(sf->scan_params));
  sf->scan_params.default_policy = BLE_SCANNER_MONITOR;

  dev_timer_init_sec(&scheduler->timer, &sf->lifetime_tk, NULL, 2, 1);
  sf->peerdb = params->peerdb;
  sf->scan_params = params->scan_params;

  *layer = &sf->layer;

  return 0;

 err_free:

  mem_free(sf);

  return err;
}
