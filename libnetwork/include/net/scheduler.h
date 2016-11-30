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

#ifndef NET_SCHEDULER_H
#define NET_SCHEDULER_H

/**
   @file
   @module {Libraries::Abstract network stack}
   @short Network scheduler

   @this contains all relevant declarations about @ref
   {net_scheduler_s} {Network stack scheduler}.
 */

#include <hexo/types.h>
#include <mutek/scheduler.h>
#include <device/class/timer.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

#include <net/task.h>
#include <net/layer.h>

#define GCT_CONTAINER_ALGO_net_timeout_queue CLIST
#define GCT_CONTAINER_LOCK_net_timeout_queue HEXO_LOCK_IRQ

GCT_CONTAINER_TYPES(net_timeout_queue, struct net_task_s *, queue_entry);

struct net_scheduler_s;

struct net_scheduler_vtable_s
{
  /**
     @returns false if scheduler processing should stop definitly.
   */
  bool_t (*idle)(struct net_scheduler_s *scheduler);
};

/**
   @this is a network scheduler context.  No field should be accessed
   directly.
 */
struct net_scheduler_s
{
  net_timeout_queue_root_t delayed_tasks;
  net_layer_sched_list_root_t layers;

  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;

  struct slab_s task_pool;
  struct buffer_pool_s *chunk_pool;
};

/**
   @this initializes a network scheduler context.

   @param sched Scheduler context to initialize
   @param chunk_pool Buffer pool to allocate chunks in
   @param timer_dev Timer device path. It will be the reference timer
          for the scheduler.
 */
error_t net_scheduler_init(
  struct net_scheduler_s *sched,
  struct buffer_pool_s *chunk_pool,
  const char *timer_dev);

/**
   @this releases all data associated to a scheduler.
 */
error_t net_scheduler_cleanup(struct net_scheduler_s *sched);

/**
   @this allocates a task in the scheduler's task internal slab.  Task
   must only be used by layers associated to this scheduler.
 */
struct net_task_s *net_scheduler_task_alloc(struct net_scheduler_s *sched);

/**
   @this allocates a chunk from scheduler's chunk pool.
 */
ALWAYS_INLINE
struct buffer_s *net_scheduler_chunk_alloc(struct net_scheduler_s *sched)
{
  return buffer_pool_alloc(sched->chunk_pool);
}

/**
   @this allocates a chunk for layer.

   @param layer Layer to allocate chunk for
   @param begin reserved header size for layers headers
          (should be at least @tt{layer->context.prefix_size})
   @param size minimal data size to allocate chunk for
          (should be no more than @tt{layer->context.mtu})
 */
ALWAYS_INLINE
struct buffer_s *net_layer_chunk_alloc(
  struct net_layer_s *layer,
  size_t begin,
  size_t size)
{
  struct buffer_s *pkt = net_scheduler_chunk_alloc(layer->scheduler);

  if (pkt) {
    pkt->begin = begin;
    pkt->end = begin + size;
  }

  return pkt;
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
  struct net_task_s *task);

/**
   @this cancels a scheduled task. @internal
 */
void net_scheduler_task_cancel(
  struct net_scheduler_s *sched,
  struct net_task_s *task);

/**
   @this cancels all tasks involving a given layer. @internal
 */
void net_scheduler_from_layer_cancel(
  struct net_scheduler_s *sched,
  struct net_layer_s *layer);

#endif
