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

//#define dprintk printk
#define dprintk(...) do{}while(0)

STRUCT_INHERIT(net_scheduler_s, dev_timer_rq_s, timer_rq);

GCT_CONTAINER_KEY_TYPES(net_timeout_queue, CUSTOM, SCALAR,
                        net_timeout_queue_item->timeout.deadline,
                        net_timeout_queue);
GCT_CONTAINER_KEY_FCNS(net_timeout_queue, ASC, static, net_timeout_queue, net_timeout_queue,
                       init, destroy, pop, head, insert);
GCT_CONTAINER_KEY_NOLOCK_FCNS(net_timeout_queue, ASC, static, net_timeout_queue_nolock, net_timeout_queue,
                              remove);

static
void net_sched_wakeup(struct net_scheduler_s *sched)
{
  dprintk("Net scheduler wakeup\n");

  LOCK_SPIN_IRQ(&sched->lock);
  if (!sched->scheduled) {
    sched->scheduled = 1;
    sched_context_start(&sched->sched_context);
  }
  LOCK_RELEASE_IRQ(&sched->lock);
}

static KROUTINE_EXEC(net_scheduler_timeout)
{
  struct net_scheduler_s *sched = KROUTINE_CONTAINER(kr, *sched, timer_rq.base.kr);

  net_sched_wakeup(sched);
}

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

  DEVICE_OP(&sched->timer, request, &sched->timer_rq);
}

static bool_t net_scheduler_timeout_handle(struct net_scheduler_s *sched)
{
  struct net_task_s *task;
  dev_timer_value_t now;
  bool_t changed = 0;

  now = net_scheduler_time_get(sched);

  while ((task = net_timeout_queue_head(&sched->delayed_tasks))) {
    dprintk("Sched delayed task %p...\n", task);

    if (now < task->timeout.deadline - task->timeout.precision)
      break;

    changed = 1;

    task = net_timeout_queue_pop(&sched->delayed_tasks);
    net_task_queue_pushback(&sched->pending_tasks, task);
  }

  return changed;
}

static bool_t net_scheduler_tasks_handle(struct net_scheduler_s *sched)
{
  struct net_task_s *task;
  bool_t changed = 0;

  while ((task = net_task_queue_pop(&sched->pending_tasks))) {
    changed = 1;

    dprintk("Sched task %p handling in %p...\n", task, task->target->handler);

    task->target->handler->task_handle(task->target, task);
  }

  return changed;
}

error_t net_scheduler_cleanup(struct net_scheduler_s *sched)
{
  struct net_task_s *task;

  if (!net_layer_sched_list_isempty(&sched->layers))
    return -EBUSY;

  dprintk("%s\n", __FUNCTION__);

  sched->exiter = sched_get_current();
  net_sched_wakeup(sched);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&sched->lock);
  while (!sched->exited) {
    sched_stop_unlock(&sched->lock);
    lock_spin(&sched->lock);
  }
  lock_release(&sched->lock);
  CPU_INTERRUPT_RESTORESTATE;

  while ((task = net_task_queue_pop(&sched->pending_tasks)))
    net_task_destroy(task);

  while ((task = net_timeout_queue_pop(&sched->delayed_tasks)))
    net_task_destroy(task);

  net_task_queue_destroy(&sched->pending_tasks);
  net_timeout_queue_destroy(&sched->delayed_tasks);
  net_layer_sched_list_destroy(&sched->destroyed_layers);

  mem_free(context_destroy(&sched->context));

  slab_cleanup(&sched->task_pool);
  lock_destroy(&sched->lock);

  return 0;
}

static void sched_cleanup(void *param)
{
  struct net_scheduler_s *sched = param;

  dprintk("%s\n", __FUNCTION__);

  sched->exited = 1;

  sched_context_exit();
}

void net_scheduler_layer_created(struct net_scheduler_s *sched, struct net_layer_s *layer)
{
  net_layer_sched_list_pushback(&sched->layers, layer);
}

void net_scheduler_layer_destroyed(struct net_scheduler_s *sched, struct net_layer_s *layer)
{
  net_layer_sched_list_remove(&sched->layers, layer);
  net_layer_sched_list_pushback(&sched->destroyed_layers, layer);
  net_sched_wakeup(sched);
}

static CONTEXT_ENTRY(net_scheduler_worker)
{
  struct net_scheduler_s *sched = param;
  struct net_layer_s *layer;

  cpu_interrupt_enable();

  dprintk("Net scheduler started\n");

  while (!sched->exiter) {
    dprintk("Net scheduler iteration\n");

    net_scheduler_timeout_handle(sched);
    net_scheduler_timeout_schedule(sched);

    if (net_scheduler_tasks_handle(sched))
      continue;

    if (!net_layer_sched_list_isempty(&sched->destroyed_layers)) {
      while ((layer = net_layer_sched_list_pop(&sched->destroyed_layers)))
        net_layer_destroy_real(layer);

      /* printk("In scheduler: %d live layers left\n", */
      /*        net_layer_sched_list_count(&sched->layers)); */

      continue;
    }

    dprintk("   Nothing to do, waiting\n");

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&sched->lock);
    sched->scheduled = 0;
    sched_stop_unlock(&sched->lock);
    CPU_INTERRUPT_RESTORESTATE;
  }

  if (sched->timer_rq.rq.drvdata)
    DEVICE_OP(&sched->timer, cancel, &sched->timer_rq);

  cpu_interrupt_disable();

  cpu_context_stack_use(sched_tmp_context(), sched_cleanup, sched);
}

static SLAB_GROW(scheduler_pool_grow)
{
  return 20;
}

static void scheduler_task_free(
  struct net_task_s *task)
{
  struct net_scheduler_s *sched = task->source->scheduler;

  dprintk("%s %p\n", __FUNCTION__, task);

#if defined(CONFIG_COMPILE_DEBUG)
  memset(task, 0x55, sizeof(*task));
#endif

  slab_free(&sched->task_pool, task);
}

struct net_task_s *net_scheduler_task_alloc(
  struct net_scheduler_s *sched)
{
  struct net_task_s *task = slab_alloc(&sched->task_pool);

  dprintk("%s %p\n", __FUNCTION__, task);

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

  sched->exiter = NULL;
  sched->scheduled = 0;
  sched->exited = 0;
  lock_init(&sched->lock);

  net_layer_sched_list_init(&sched->layers);
  net_task_queue_init(&sched->pending_tasks);
  net_timeout_queue_init(&sched->delayed_tasks);
  net_layer_sched_list_init(&sched->destroyed_layers);

  sched->packet_pool = packet_pool;

  slab_init(&sched->task_pool, sizeof(struct net_task_s),
            scheduler_pool_grow, mem_scope_sys);

  sched->timer_rq.rq.drvdata = NULL;
  dev_timer_rq_init_immediate(&sched->timer_rq, net_scheduler_timeout);

  device_start(&sched->timer.base);

  const size_t stack_size = 1024;

  sched->stack = mem_alloc_align(stack_size, CONFIG_HEXO_STACK_ALIGN, mem_scope_sys);
  if (!sched->stack) {
    err = -ENOMEM;
    goto out_slab;
  }

  err = context_init(&sched->context, sched->stack, sched->stack + stack_size,
                     net_scheduler_worker, sched);
  if (err)
    goto out_stack;

  sched_context_init(&sched->sched_context, &sched->context);

  net_sched_wakeup(sched);

  return 0;

 out_stack:
  mem_free(sched->stack);
 out_slab:
  slab_cleanup(&sched->task_pool);
  lock_destroy(&sched->lock);

  return err;
}

void net_scheduler_task_push(
    struct net_scheduler_s *sched,
    struct net_task_s *task)
{
  dprintk("Sched task %p from %p to %p\n", task,
         task->source->handler,
         task->target->handler);

  if (task->type == NET_TASK_TIMEOUT)
    net_timeout_queue_insert(&sched->delayed_tasks, task);
  else
    net_task_queue_pushback(&sched->pending_tasks, task);

  net_sched_wakeup(sched);
}

dev_timer_value_t net_scheduler_time_get(struct net_scheduler_s *sched)
{
  dev_timer_value_t now;
  DEVICE_OP(&sched->timer, get_value, &now, 0);
  return now;
}

void net_scheduler_from_layer_cancel(
  struct net_scheduler_s *sched,
  struct net_layer_s *layer)
{
  GCT_FOREACH(net_timeout_queue, &sched->delayed_tasks, item,
              if (item->source != layer && item->target != layer)
                GCT_FOREACH_CONTINUE;
              net_timeout_queue_nolock_remove(&sched->delayed_tasks, item);
              net_task_destroy(item);
              );

  GCT_FOREACH(net_task_queue, &sched->pending_tasks, item,
              if (item->source != layer && item->target != layer)
                GCT_FOREACH_CONTINUE;
              net_task_queue_nolock_remove(&sched->pending_tasks, item);
              net_task_destroy(item);
              );
}

void net_scheduler_task_cancel(
  struct net_scheduler_s *sched,
  struct net_task_s *task)
{
  bool_t found = 0;

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

  GCT_FOREACH(net_task_queue, &sched->pending_tasks, item,
              if (item != task)
                GCT_FOREACH_CONTINUE;
              net_task_queue_nolock_remove(&sched->pending_tasks, item);
              net_task_destroy(item);
              GCT_FOREACH_BREAK;
              );
}

void net_scheduler_timer_use(struct net_scheduler_s *sched)
{
}

void net_scheduler_timer_release(struct net_scheduler_s *sched)
{
}

