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

struct net_scheduler_handler_s
{
  void (*destroyed)(struct net_scheduler_s *sched);
};

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

#if 0
#define net_scheduler_refinc(x) ({ printk("Scheduler refinc %d\n", (x)->obj_entry._count.value); net_scheduler_refinc(x); })
#define net_scheduler_refdec(x) do{ printk("Scheduler refdec %d\n", (x)->obj_entry._count.value); net_scheduler_refdec(x); }while(0)
#endif

error_t net_scheduler_init(
  struct net_scheduler_s *sched,
  const struct net_scheduler_handler_s *handler,
  struct buffer_pool_s *packet_pool,
  const char *timer_dev);

struct net_task_s *net_scheduler_task_alloc(
  struct net_scheduler_s *sched);

void net_scheduler_stop(
  struct net_scheduler_s *sched);

void net_scheduler_task_free(
  struct net_scheduler_s *sched,
  struct net_task_s *task);

struct buffer_s *net_scheduler_packet_alloc(
  struct net_scheduler_s *sched);

void net_scheduler_cleanup(
  struct net_scheduler_s *sched);

dev_timer_value_t net_scheduler_time_get(
  struct net_scheduler_s *sched);

void net_scheduler_task_push(
  struct net_scheduler_s *sched,
  struct net_task_header_s *task);

void net_scheduler_task_cancel(
  struct net_scheduler_s *sched,
  struct net_task_header_s *task);

void net_scheduler_from_layer_cancel(
  struct net_scheduler_s *sched,
  struct net_layer_s *layer);

ALWAYS_INLINE size_t net_scheduler_packet_mtu(
  struct net_scheduler_s *sched)
{
  return buffer_pool_unit_size(sched->packet_pool);
}

#endif
