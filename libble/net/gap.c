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

#define LOGK_MODULE_ID "bgap"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/gap.h>
#include <ble/net/generic.h>

#include <ble/gattdb/db.h>
#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>

struct ble_gatt_db_s;

/**
 BLE Generic Access Profile layer
 */
struct ble_gap_s
{
  struct net_layer_s layer;
  struct net_layer_s *sig;
  struct ble_gattdb_s *db;
  struct net_task_s *conn_update_task;
};

STRUCT_COMPOSE(ble_gap_s, layer);

static
void ble_gap_destroyed(struct net_layer_s *layer)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);

  net_layer_refdec(gap->sig);

  mem_free(gap);
}

static struct ble_gap_conn_params_update_s *gap_create_update(struct ble_gap_s *gap)
{
  const uint16_t *tmp;
  size_t size;

  if (ble_gattdb_std_char_read(gap->db,
                                BLE_GATT_SERVICE_GENERIC_ACCESS,
                                BLE_GATT_CHAR_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
                                (const void **)&tmp, &size) || size != 8)
    return NULL;

  struct ble_gap_conn_params_update_s *upd = malloc(sizeof(*upd));

  upd->interval_min = tmp[0];
  upd->interval_max = tmp[1];
  upd->slave_latency = tmp[2];
  upd->timeout = tmp[3];

  uint16_t ios_rules_timeout = (upd->slave_latency + 1) * upd->interval_max * 2 * 125 / 1000;

  upd->timeout = __MAX(tmp[3], ios_rules_timeout);

  upd->task.destroy_func = (void*)memory_allocator_push;

  return upd;
}

static void gap_update_conn_in(struct ble_gap_s *gap, uint32_t sec)
{
  if (gap->conn_update_task) {
    net_scheduler_task_cancel(gap->layer.scheduler,
                              gap->conn_update_task);
    gap->conn_update_task = NULL;
  }

  if (!gap->layer.parent)
    return;

  struct net_task_s *timeout = net_scheduler_task_alloc(gap->layer.scheduler);
  if (timeout) {
    dev_timer_delay_t ticks;

    dev_timer_init_sec(&gap->layer.scheduler->timer, &ticks, NULL, sec, 1);

    logk_trace("Asking for new connection parametters in %d ticks",
           (int)ticks);

    net_task_timeout_push(timeout, &gap->layer, net_scheduler_time_get(gap->layer.scheduler) + ticks, 0);
    gap->conn_update_task = timeout;
  }
}

static
void ble_gap_task_handle(struct net_layer_s *layer,
                         struct net_task_s *task)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);
  struct net_layer_s *target = NULL;

  if (!gap->layer.parent)
    goto out;

  switch (task->type) {
  default:
    break;

  case NET_TASK_TIMEOUT: {
    logk_trace("Asking for new connection parametters to LLCP");
    gap->conn_update_task = NULL;

    target = gap->layer.parent;

    goto ask;
  }

  case NET_TASK_RESPONSE:
    if (task->query.opcode != BLE_GAP_CONN_PARAMS_UPDATE)
      break;

    logk_trace("Conn params update response from %s: %d",
           task->source == layer->parent ? "LLCP" : "Sig",
           task->query.err);

    if (task->query.err && task->source == layer->parent) {
      logk_trace("LLCP does not support conn params update");

      target = gap->sig;
      goto ask;
    }

    logk_trace("Conn update retrying later...");
    gap_update_conn_in(gap, 30);
    break;
  }

 out:
  net_task_destroy(task);

  return;

 ask:
  if (target) {
    struct ble_gap_conn_params_update_s *upd = gap_create_update(gap);

    if (upd)
      net_task_query_push(&upd->task, target, &gap->layer, BLE_GAP_CONN_PARAMS_UPDATE);
  }

  goto out;
}

static void ble_gap_context_changed(struct net_layer_s *layer)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);

  if (layer->context.addr.encrypted)
    gap_update_conn_in(gap, 5);
}

static void ble_gap_dangling(struct net_layer_s *layer)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);

  if (gap->conn_update_task) {
    net_scheduler_task_cancel(gap->layer.scheduler,
                              gap->conn_update_task);
    gap->conn_update_task = NULL;
  }
}

static const struct net_layer_handler_s gap_handler = {
  .destroyed = ble_gap_destroyed,
  .task_handle = ble_gap_task_handle,
  .context_changed = ble_gap_context_changed,
  .dangling = ble_gap_dangling,
};

error_t ble_gap_create(
  struct net_scheduler_s *scheduler,
  const void *params_,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable,
  struct net_layer_s **layer)
{
  struct ble_gap_s *gap = mem_alloc(sizeof(*gap), mem_scope_sys);
  const struct ble_gap_params_s *params = params_;

  if (!gap)
    return -ENOMEM;

  memset(gap, 0, sizeof(*gap));

  error_t err = net_layer_init(&gap->layer, &gap_handler, scheduler,
                               delegate, delegate_vtable);
  if (err) {
    mem_free(gap);
    return err;
  }

  gap->db = params->db;
  gap->sig = net_layer_refinc(params->sig);
  gap->conn_update_task = NULL;

  *layer = &gap->layer;

  return err;
}
