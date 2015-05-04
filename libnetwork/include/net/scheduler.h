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

/**
   @file
   @module{Network stack library}
   @short Network scheduler
  
   @section {Description}
  
   Network scheduler maintains a queue of tasks.  Network scheduler is a
   Mutek's scheduler context with a FIFO a tasks to run.  This design
   ensures at most one network task is running at the same time for a
   given network context.  This avoids usage for locking.
  
   All tasks share execution in the same CPU stack, reducing memory
   usage.
  
   Tasks may have a timeout. Timeout is expressed relative to
   scheduler's reference timer.
  
   A scheduler is reference-counted.  It gets destroyed with its last
   layer.
  
   @end section
 */

#ifndef NET_SCHEDULER_H
#define NET_SCHEDULER_H

#include <hexo/types.h>
#include <mutek/scheduler.h>
#include <device/class/timer.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

#include "task.h"

#define GCT_CONTAINER_ALGO_net_timeout_queue CLIST
#define GCT_CONTAINER_LOCK_net_timeout_queue HEXO_LOCK_IRQ

GCT_CONTAINER_TYPES(net_timeout_queue, struct net_task_header_s *, queue_entry);

struct net_scheduler_s;

/**
   @this is a vtable for a scheduler.
 */
struct net_scheduler_handler_s
{
  /**
     @this gets called after the last layer got destroyed and dropped
     the last reference on the scheduler.

     Delegate should free up the scheduler's context memory.
   */
  void (*destroyed)(struct net_scheduler_s *sched);
};

/**
   @this is a network scheduler context.  No field should be accessed
   directly.
 */
struct net_scheduler_s
{
  lock_t lock;

  GCT_REFCOUNT_ENTRY(obj_entry);

  const struct net_scheduler_handler_s *handler;

  struct context_s context;
  struct sched_context_s sched_context;

  net_task_queue_root_t pending_tasks;
  net_timeout_queue_root_t delayed_tasks;

  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;

  struct slab_s task_pool;
  struct buffer_pool_s *packet_pool;

  bool_t running;
  bool_t scheduled;

  void *stack;
};

GCT_REFCOUNT(net_scheduler, struct net_scheduler_s *, obj_entry);

/**
   @this initializes a network scheduler context.

   @param sched Scheduler context to initialize
   @param handler Vtable for scheduler
   @param packet_pool Buffer pool to allocate packets in
   @param timer_dev Timer device path. It will be the reference timer
          for the scheduler.
 */
error_t net_scheduler_init(
  struct net_scheduler_s *sched,
  const struct net_scheduler_handler_s *handler,
  struct buffer_pool_s *packet_pool,
  const char *timer_dev);

/**
   @this allocates a task in the scheduler's task internal slab.  Task
   should only be used inside layers bound to this scheduler.
 */
struct net_task_s *net_scheduler_task_alloc(
  struct net_scheduler_s *sched);

/**
   @this frees a task allocated from @ref {net_scheduler_task_alloc}.
 */
void net_scheduler_task_free(
  struct net_scheduler_s *sched,
  struct net_task_s *task);

/**
   @this allocates a packet from scheduler's packet pool.
 */
ALWAYS_INLINE
struct buffer_s *net_scheduler_packet_alloc(struct net_scheduler_s *sched)
{
  return buffer_pool_alloc(sched->packet_pool);
}

/**
   @this retrieves current time from scheduler's reference timer.
 */
dev_timer_value_t net_scheduler_time_get(
  struct net_scheduler_s *sched);

/**
   @this pushes a task to scheduling. @internal
 */
void net_scheduler_task_push(
  struct net_scheduler_s *sched,
  struct net_task_header_s *task);

/**
   @this cancels a scheduled task. @internal
 */
void net_scheduler_task_cancel(
  struct net_scheduler_s *sched,
  struct net_task_header_s *task);

/**
   @this cancels all tasks involving a given layer. @internal
 */
void net_scheduler_from_layer_cancel(
  struct net_scheduler_s *sched,
  struct net_layer_s *layer);

/**
   @this retrieves the maximum packet size from scheduler's packet
   pool buffers.
 */
ALWAYS_INLINE size_t net_scheduler_packet_mtu(
  struct net_scheduler_s *sched)
{
  return buffer_pool_unit_size(sched->packet_pool);
}

#endif
