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

static
void ble_gap_destroyed(struct net_layer_s *layer)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);

  net_layer_refdec(gap->sig);

  gap->handler->destroyed(gap);
}

static void gap_cu_later(struct ble_gap_s *gap)
{
  struct net_task_s *task = net_scheduler_task_alloc(gap->layer.scheduler);

  net_task_timeout_push(task, &gap->layer,
                        net_scheduler_time_get(gap->layer.scheduler) + 32768 * 10,
                        0);
}

static
void ble_gap_task_handle(struct net_layer_s *layer,
                         struct net_task_header_s *header)
{
  struct ble_gap_s *gap = ble_gap_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  switch (header->type) {
  default:
    break;

  case NET_TASK_RESPONSE:
    switch (task->query.opcode) {
    case BLE_SIG_CONN_PARAMS_UPDATE:
      break;
    }
    break;

  case NET_TASK_TIMEOUT: {
    struct ble_signalling_conn_params_update_task_s *upd = malloc(sizeof(*upd));
    dev_timer_value_t now = net_scheduler_time_get(gap->layer.scheduler);

    upd->interval_min = (now & 0x1f) + 16;
    upd->interval_max = upd->interval_min + 16;
    upd->slave_latency = __MIN((now / 2) & 0x3f,
                               1600 / (upd->interval_max + 1));
    upd->timeout = 500;

    upd->task.header.destroy_func = (void*)memory_allocator_push;
    upd->task.header.allocator_data = NULL;

    net_task_query_push(&upd->task, gap->sig, &gap->layer, BLE_SIG_CONN_PARAMS_UPDATE);

    gap_cu_later(gap);
    break;
  }
  }

  net_task_cleanup(task);
}

static const struct net_layer_handler_s gap_handler = {
  .destroyed = ble_gap_destroyed,
  .task_handle = ble_gap_task_handle,
  .type = BLE_LAYER_TYPE_GAP,
  .use_timer = 1,
};

error_t ble_gap_init(
  struct ble_gap_s *gap,
  const struct ble_gap_handler_s *handler,
  struct net_scheduler_s *scheduler,
  struct net_layer_s *sig)
{
  error_t err = net_layer_init(&gap->layer, &gap_handler, scheduler);
  if (err)
    return err;

  gap->sig = net_layer_refinc(sig);
  gap->handler = handler;

  gap_cu_later(gap);

  return 0;
}
