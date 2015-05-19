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

//#define dprintk printk
#define dprintk(...) do{}while(0)

STRUCT_INHERIT(net_scheduler_s, dev_timer_rq_s, timer_rq);
GCT_CONTAINER_FCNS(net_task_queue, static, net_task_queue,
                   init, destroy, pushback, pop, remove);

GCT_CONTAINER_KEY_TYPES(net_timeout_queue, CUSTOM, SCALAR,
                        net_task_s_from_header(net_timeout_queue_item)->timeout.deadline,
                        net_timeout_queue);
GCT_CONTAINER_KEY_FCNS(net_timeout_queue, ASC, static, net_timeout_queue, net_timeout_queue,
                       init, destroy, pop, head, remove, insert);

static void net_sched_wakeup(struct net_scheduler_s *sched)
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
  struct net_scheduler_s *sched = KROUTINE_CONTAINER(kr, *sched, timer_rq.rq.kr);

  net_sched_wakeup(sched);
}

static void net_scheduler_timeout_schedule(struct net_scheduler_s *sched)
{
  struct net_task_s *task;

  task = net_task_s_from_header(net_timeout_queue_head(&sched->delayed_tasks));
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

  kroutine_init(&sched->timer_rq.rq.kr, net_scheduler_timeout, KROUTINE_IMMEDIATE);
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

  while ((task = net_task_s_from_header(net_timeout_queue_head(&sched->delayed_tasks)))) {
    dprintk("Sched delayed task %p...\n", task);

    if (now < task->timeout.deadline - task->timeout.precision)
      break;

    changed = 1;

    task = net_task_s_from_header(net_timeout_queue_pop(&sched->delayed_tasks));
    net_task_queue_pushback(&sched->pending_tasks, &task->header);
  }

  return changed;
}

static bool_t net_scheduler_tasks_handle(struct net_scheduler_s *sched)
{
  struct net_task_header_s *task;
  bool_t changed = 0;

  while ((task = net_task_queue_pop(&sched->pending_tasks))) {
    changed = 1;

    dprintk("Sched task %p handling in %S...\n", task, &task->target->type, 4);

    task->target->handler->task_handle(task->target, task);
  }

  return changed;
}

static void sched_cleanup(void *param)
{
  struct net_scheduler_s *sched = param;

  dprintk("Net scheduler cleanup\n");

  mem_free(context_destroy(&sched->context));
  sched->handler->destroyed(sched);

  sched_context_exit();
}

static CONTEXT_ENTRY(net_scheduler_worker)
{
  struct net_scheduler_s *sched = param;
  struct net_task_header_s *task;

  dprintk("Net scheduler started\n");

  while (sched->running) {
    dprintk("Net scheduler iteration\n");

    net_scheduler_timeout_handle(sched);
    net_scheduler_timeout_schedule(sched);

    if (net_scheduler_tasks_handle(sched))
      continue;

    dprintk("   Nothing to do, waiting\n");
    
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&sched->lock);
    sched->scheduled = 0;
    sched_stop_unlock(&sched->lock);
    CPU_INTERRUPT_RESTORESTATE;
  }

  while ((task = net_task_queue_pop(&sched->pending_tasks)))
    net_task_cleanup(net_task_s_from_header(task));

  while ((task = net_timeout_queue_pop(&sched->delayed_tasks)))
    net_task_cleanup(net_task_s_from_header(task));

  dprintk("Net scheduler done\n");

  if (sched->timer_rq.rq.drvdata)
    DEVICE_OP(&sched->timer, cancel, &sched->timer_rq);

  net_task_queue_destroy(&sched->pending_tasks);
  net_timeout_queue_destroy(&sched->delayed_tasks);
  device_stop(&sched->timer);
  slab_cleanup(&sched->task_pool);
  lock_destroy(&sched->lock);

  cpu_context_stack_use(sched_tmp_context(), sched_cleanup, sched);
}

static SLAB_GROW(scheduler_pool_grow)
{
  return 20;
}

static void scheduler_task_free(
  struct net_task_s *task)
{
  struct net_scheduler_s *sched = task->header.allocator_data;

  slab_free(&sched->task_pool, task);

  net_scheduler_refdec(sched);
}

struct net_task_s *net_scheduler_task_alloc(
  struct net_scheduler_s *sched)
{
  struct net_task_s *task = slab_alloc(&sched->task_pool);

  task->header.destroy_func = scheduler_task_free;
  task->header.allocator_data = net_scheduler_refinc(sched);

  return task;
}

error_t net_scheduler_init(
  struct net_scheduler_s *sched,
  const struct net_scheduler_handler_s *handler,
  struct buffer_pool_s *packet_pool,
  const char *timer_dev)
{
  error_t err;

  memset(sched, 0, sizeof(*sched));

  net_scheduler_refinit(sched);

  err = device_get_accessor_by_path(&sched->timer, NULL, timer_dev, DRIVER_CLASS_TIMER);
  if (err)
    return err;

  sched->handler = handler;
  sched->running = 1;
  sched->scheduled = 0;
  lock_init(&sched->lock);

  net_task_queue_init(&sched->pending_tasks);
  net_timeout_queue_init(&sched->delayed_tasks);

  sched->packet_pool = packet_pool;

  slab_init(&sched->task_pool, sizeof(struct net_task_s),
            scheduler_pool_grow, mem_scope_sys);

  sched->timer_rq.rq.drvdata = NULL;

  device_start(&sched->timer);

  const size_t stack_size = 512;

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

void net_scheduler_destroy(struct net_scheduler_s *sched)
{
  sched->running = 0;
  net_sched_wakeup(sched);
}

void net_scheduler_task_push(
    struct net_scheduler_s *sched,
    struct net_task_header_s *task)
{
  dprintk("Sched task %p pushed %d to %S\n", task, task->type,
          &task->target->type, 4);

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
              if (item->target != layer)
                GCT_FOREACH_CONTINUE;
              net_timeout_queue_remove(&sched->delayed_tasks, item);
              );

  GCT_FOREACH(net_task_queue, &sched->pending_tasks, item,
              if (item->target != layer)
                GCT_FOREACH_CONTINUE;
              net_task_queue_remove(&sched->pending_tasks, item);
              );
}
