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

#include <ble/net/gap.h>
#include <ble/net/signalling.h>
#include <ble/net/slave.h>
#include <ble/net/layer.h>

#include <ble/net/generic.h>

#include <ble/gattdb/db.h>
#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>

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
                                BLE_UUID_GENERIC_ACCESS_SERVICE,
                                BLE_UUID_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_CHAR,
                                (const void **)&tmp, &size) || size != 8)
    return NULL;

  struct ble_gap_conn_params_update_s *upd = malloc(sizeof(*upd));

  upd->interval_min = tmp[0];
  upd->interval_max = tmp[1];
  upd->slave_latency = tmp[2];
  upd->timeout = tmp[3];

  upd->task.destroy_func = (void*)memory_allocator_push;

  return upd;
}

static void gap_update_conn_in(struct ble_gap_s *gap, uint32_t sec)
{
  if (gap->conn_update_task) {
    net_scheduler_task_cancel(gap->layer.scheduler,
                              gap->conn_update_task);
    net_task_destroy(gap->conn_update_task);
    gap->conn_update_task = NULL;
  }

  if (!gap->layer.parent)
    return;

  struct net_task_s *timeout = net_scheduler_task_alloc(gap->layer.scheduler);
  if (timeout) {
    dev_timer_delay_t ticks;

    dev_timer_init_sec(&gap->layer.scheduler->timer, &ticks, NULL, sec, 1);

    printk("GAP asking for new connection parametters in %d ticks\n",
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

  if (!gap->layer.parent)
    goto out;

  switch (task->type) {
  default:
    break;

  case NET_TASK_TIMEOUT: {
    printk("GAP asking for new connection parametters to LLCP\n");
    struct ble_gap_conn_params_update_s *upd = gap_create_update(gap);

    if (!upd)
      break;

    net_task_query_push(&upd->task, gap->layer.parent, &gap->layer, BLE_GAP_CONN_PARAMS_UPDATE);

    gap->conn_update_task = NULL;
    break;
  }

  case NET_TASK_RESPONSE:
    if (task->query.opcode != BLE_GAP_CONN_PARAMS_UPDATE)
      break;

    printk("GAP conn params update response from %d: %d\n",
           &task->source->handler->type, task->query.err);

    if (task->source == layer->parent && task->query.err == -ENOTSUP) {
      struct ble_gap_conn_params_update_s *upd = gap_create_update(gap);

      printk("GAP conn params update forwarded to signalling\n");

      net_task_query_push(&upd->task, gap->sig, &gap->layer, BLE_GAP_CONN_PARAMS_UPDATE);
      break;
    }

    if (task->query.err == -EINVAL)
      gap_update_conn_in(gap, 30);

    break;
  }

 out:
  net_task_destroy(task);
}

static bool_t ble_gap_context_updated(
    struct net_layer_s *layer,
    const struct net_layer_context_s *parent_context)
{
  __attribute__((unused))
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);

  if (parent_context->addr.encrypted)
    gap_update_conn_in(gap, 5);

  return 1;
}

static void ble_gap_dandling(struct net_layer_s *layer)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);

  if (gap->conn_update_task) {
    net_scheduler_task_cancel(gap->layer.scheduler,
                              gap->conn_update_task);
    net_task_destroy(gap->conn_update_task);
    gap->conn_update_task = NULL;
  }
}

static const struct net_layer_handler_s gap_handler = {
  .destroyed = ble_gap_destroyed,
  .task_handle = ble_gap_task_handle,
  .context_updated = ble_gap_context_updated,
  .dandling = ble_gap_dandling,
  .type = BLE_NET_LAYER_GAP,
  .use_timer = 1,
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
