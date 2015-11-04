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

#include <ble/net/layer.h>
#include <ble/net/scanner.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/advertise.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#define dprintk printk

#define GCT_CONTAINER_ALGO_ble_peri_list CLIST

GCT_CONTAINER_KEY_TYPES(ble_peri_list, CUSTOM, SCALAR, item->first_seen, ble_peri_list);

GCT_CONTAINER_KEY_FCNS(ble_peri_list, ASC, inline, ble_peri_list, ble_peri_list,
                       init, destroy, pop, isempty, head, prev, next,remove, insert);
o
struct ble_peri_s
{
  GCT_CONTAINER_ENTRY(ble_peri_list, entry);

  struct ble_addr_s addr;
  uint8_t ad[80];
  uint8_t adv_ad_len;
  uint8_t scan_ad_len;
  dev_timer_value_t first_seen;
  dev_timer_value_t last_seen;
};

GCT_CONTAINER_TYPES(ble_peri_list, struct ble_peri_s *, entry);

/**
   BLE Advertising layer
 */
struct ble_scanner_s
{
  struct net_layer_s layer;

  struct device_ble_radio_s radio;
  struct device_timer_s timer;

  uint32_t interval_tk;
  uint32_t duration_tk;

  ble_peri_list_root_t adv_data;
};

struct ble_connection_parameters_s;

STRUCT_COMPOSE(ble_scanner_s, layer);


static const struct net_layer_handler_s scanner_handler;

static
error_t ble_scanner_init(
  struct ble_scanner_s *adv,
  struct net_scheduler_s *scheduler,
  const char *ble)
{
  error_t err;
  struct ble_radio_info_s ble_info;

  memset(adv, 0, sizeof(*adv));

  err = device_get_accessor_by_path(&adv->radio, NULL, ble, DRIVER_CLASS_BLE_RADIO);
  if (err)
    goto err_out;

  err = device_get_accessor_by_path(&adv->timer, NULL, ble, DRIVER_CLASS_TIMER);
  if (err)
    goto err_put_radio;

  err = net_layer_init(&adv->layer, &scanner_handler, scheduler);
  if (err)
    goto err_put_devices;

  DEVICE_OP(&adv->radio, get_info, &ble_info);

  scanner_request_schedule(adv);

  dprintk("scanner init done\n");

  return 0;

 err_put_devices:
  device_put_accessor(&adv->timer);

 err_put_radio:
  device_put_accessor(&adv->radio);

 err_out:
  return err;
}

static
void ble_scanner_destroyed(struct net_layer_s *layer)
{
  struct ble_scanner_s *adv = ble_scanner_s_from_layer(layer);

  dprintk("Scanner %p destroyed\n", adv);

  device_stop(&adv->radio);
  device_put_accessor(&adv->timer);
  device_put_accessor(&adv->radio);

  mem_free(adv);
}

static
void ble_scanner_task_handle(struct net_layer_s *layer,
                             struct net_task_s *task)
{
  struct ble_scanner_s *adv = ble_scanner_s_from_layer(layer);

  dprintk("%s in %p %p -> %p", __FUNCTION__, adv, task->source, task->target);

  switch (task->type) {

  default:
    dprintk(" other\n");
    break;
  }

  net_task_destroy(task);
}

static const struct net_layer_handler_s scanner_handler = {
  .destroyed = ble_scanner_destroyed,
  .task_handle = ble_scanner_task_handle,
  .type = BLE_NET_LAYER_ADV,
};

error_t ble_adv_create(struct net_scheduler_s *scheduler,
                       const char *radio_dev,
                       struct net_layer_s **layer)
{
  struct ble_scanner_s *adv = mem_alloc(sizeof(*adv), mem_scope_sys);

  if (!adv)
    return -ENOMEM;

  error_t err = ble_scanner_init(adv, scheduler, radio_dev);
  if (err)
    mem_free(adv);
  else
    *layer = &adv->layer;

  return err;
}
