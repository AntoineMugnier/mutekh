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

#define LOGK_MODULE_ID "nets"

#include <hexo/decls.h>
#include <hexo/types.h>
#include <hexo/lock.h>

#include <mutek/mem_alloc.h>
#include <mutek/scheduler.h>
#include <mutek/printk.h>

#include <net/layer.h>
#include <net/scheduler.h>
#include <net/task.h>

#include "network.h"

STRUCT_INHERIT(net_scheduler_s, dev_timer_rq_s, timer_rq);

GCT_CONTAINER_KEY_TYPES(net_timeout_queue, CUSTOM, SCALAR,
                        net_timeout_queue_item->timeout.deadline,
                        net_timeout_queue);
GCT_CONTAINER_KEY_FCNS(net_timeout_queue, ASC, static, net_timeout_queue, net_timeout_queue,
                       init, destroy, pop, head, insert);
GCT_CONTAINER_KEY_NOLOCK_FCNS(net_timeout_queue, ASC, static, net_timeout_queue_nolock, net_timeout_queue,
                              remove);

static void net_scheduler_timeout_schedule(struct net_scheduler_s *sched)
{
  struct net_task_s *task;

  task = net_timeout_queue_head(&sched->delayed_tasks);
  if (!task) {
    if (sched->timer_rq.rq.drvdata)
      DEVICE_OP(&sched->timer, cancel, &sched->timer_rq);
    return;
  }

  if (sched->timer_rq.rq.drvdata
      && sched->timer_rq.deadline == task->timeout.deadline)
    return;

  if (sched->timer_rq.rq.drvdata)
    DEVICE_OP(&sched->timer, cancel, &sched->timer_rq);

  sched->timer_rq.deadline = task->timeout.deadline;
  sched->timer_rq.delay = 0;

  if (DEVICE_OP(&sched->timer, request, &sched->timer_rq) != 0)
    kroutine_exec(&sched->timer_rq.rq.kr);
}

static bool_t net_scheduler_timeout_handle(struct net_scheduler_s *sched)
{
  struct net_task_s *task;
  dev_timer_value_t now;
  bool_t changed = 0;

  now = net_scheduler_time_get(sched);

  while ((task = net_timeout_queue_head(&sched->delayed_tasks))) {
    logk_trace("delayed task %p now %lld dl %lld", task, now, task->timeout.deadline);

    if (now < task->timeout.deadline - task->timeout.precision)
      break;

    changed = 1;

    task = net_timeout_queue_pop(&sched->delayed_tasks);
    net_layer_task_push(task->target, task);
  }

  return changed;
}

error_t net_scheduler_cleanup(struct net_scheduler_s *sched)
{
  struct net_task_s *task;

  if (!net_layer_sched_list_isempty(&sched->layers))
    return -EBUSY;

  logk_trace("%s", __func__);

  while ((task = net_timeout_queue_pop(&sched->delayed_tasks)))
    net_task_destroy(task);
  net_timeout_queue_destroy(&sched->delayed_tasks);

  slab_cleanup(&sched->task_pool);

  return 0;
}

static KROUTINE_EXEC(net_scheduler_timeout)
{
  struct net_scheduler_s *sched = KROUTINE_CONTAINER(kr, *sched, timer_rq.rq.kr);

  net_scheduler_timeout_handle(sched);
  net_scheduler_timeout_schedule(sched);
}

void net_scheduler_layer_created(struct net_scheduler_s *sched, struct net_layer_s *layer)
{
  net_layer_sched_list_pushback(&sched->layers, layer);
}

void net_scheduler_layer_destroyed(struct net_scheduler_s *sched, struct net_layer_s *layer)
{
  net_layer_sched_list_remove(&sched->layers, layer);
}

static SLAB_GROW(scheduler_pool_grow)
{
  return 20;
}

static void scheduler_task_free(
  struct net_task_s *task)
{
  struct net_scheduler_s *sched = task->source->scheduler;

  logk_trace("%s %p", __func__, task);

#if defined(CONFIG_COMPILE_DEBUG)
  memset(task, 0x55, sizeof(*task));
#endif

  slab_free(&sched->task_pool, task);
}

struct net_task_s *net_scheduler_task_alloc(
  struct net_scheduler_s *sched)
{
  struct net_task_s *task = slab_alloc(&sched->task_pool);

  logk_trace("%s %p", __func__, task);

  if (task) {
    task->destroy_func = (void*)scheduler_task_free;
    task->type = NET_TASK_INVALID;
    task->source = NULL;
    task->target = NULL;
  }

  return task;
}

error_t net_scheduler_init(
  struct net_scheduler_s *sched,
  struct buffer_pool_s *packet_pool,
  const char *timer_dev)
{
  error_t err;

  memset(sched, 0, sizeof(*sched));

  err = device_get_accessor_by_path(&sched->timer.base, NULL, timer_dev, DRIVER_CLASS_TIMER);
  if (err)
    return err;

  net_layer_sched_list_init(&sched->layers);
  net_timeout_queue_init(&sched->delayed_tasks);

  sched->packet_pool = packet_pool;

  slab_init(&sched->task_pool, sizeof(struct net_task_s),
            scheduler_pool_grow, mem_scope_sys);

  sched->timer_rq.rq.drvdata = NULL;
  kroutine_init_immediate(&sched->timer_rq.rq.kr, net_scheduler_timeout);

  device_start(&sched->timer.base);

  return 0;
}

void net_scheduler_task_push(
    struct net_scheduler_s *sched,
    struct net_task_s *task)
{
  logk_trace("scheduling task %p from %p to %p", task,
         task->source->handler,
         task->target->handler);

  if (task->type == NET_TASK_TIMEOUT) {
    net_timeout_queue_insert(&sched->delayed_tasks, task);
    net_scheduler_timeout_schedule(sched);
    return;
  }

  net_layer_task_push(task->target, task);
}

dev_timer_value_t net_scheduler_time_get(struct net_scheduler_s *sched)
{
  dev_timer_value_t now;

  DEVICE_OP(&sched->timer, get_value, &now, 0);

  return now;
}

void net_scheduler_from_layer_cancel(
  struct net_scheduler_s *sched,
  struct net_layer_s *source)
{
  GCT_FOREACH(net_task_queue, &sched->delayed_tasks, task,
              if (task->target == source || task->source == source) {
                net_timeout_queue_nolock_remove(&sched->delayed_tasks, task);
                net_task_destroy(task);
              }
              );
}

void net_scheduler_task_cancel(
  struct net_scheduler_s *sched,
  struct net_task_s *task)
{
  bool_t found = 0;

  if (task->type == NET_TASK_TIMEOUT) {
    GCT_FOREACH(net_timeout_queue, &sched->delayed_tasks, item,
                if (item != task)
                  GCT_FOREACH_CONTINUE;
                net_timeout_queue_nolock_remove(&sched->delayed_tasks, item);
                net_task_destroy(item);
                found = 1;
                GCT_FOREACH_BREAK;
                );

    if (found)
      return;
  }

  found = 0;

  GCT_FOREACH(net_layer_list, &sched->layers, target,
              GCT_FOREACH(net_task_queue, &target->pending, item,
                          if (item == task) {
                            net_task_queue_nolock_remove(&target->pending, item);
                            net_task_destroy(item);
                            found = 1;
                            GCT_FOREACH_BREAK;
                          }
                          );
              if (found)
                GCT_FOREACH_BREAK;
              );
}
