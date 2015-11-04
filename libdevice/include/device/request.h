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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

/**
 * @file
 * @module{Devices support library}
 * @short Generic device request
 */

#ifndef __DEVICE_REQUEST_H__
#define __DEVICE_REQUEST_H__

#include <hexo/types.h>

#include <gct_platform.h>
#include <gct/container_clist.h>
#include <gct/container_avl_p.h>
#include <device/device.h>
#include <device/driver.h>

#include <mutek/kroutine.h>
#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
# include <hexo/lock.h>
# include <hexo/interrupt.h>
#endif

#include <assert.h>

/* Container algorithm used for of queue device requests */
#define GCT_CONTAINER_ALGO_dev_request_queue CLIST

/* Container algorithm used for priority queue of device requests */
// For now, AVL_P has no support for collisions
//#define GCT_CONTAINER_ALGO_dev_request_pqueue AVL_P
#define GCT_CONTAINER_ALGO_dev_request_pqueue CLIST

struct dev_request_s
{
  struct kroutine_s                     kr;

  union {
    GCT_CONTAINER_ENTRY(dev_request_queue, queue_entry);
    GCT_CONTAINER_ENTRY(dev_request_pqueue, pqueue_entry);
  };

  /** Caller private data */
  void                                  *pvdata;

  /** Driver private data */
  union {
    void                                *drvdata;
    uintptr_t                           drvuint;
  };
};

STRUCT_COMPOSE(dev_request_s, kr);

GCT_CONTAINER_TYPES(dev_request_queue, struct dev_request_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_request_queue, inline, dev_request_queue,
                   init, destroy, push, pushback, pop, remove, isempty, head, tail, next);

GCT_CONTAINER_TYPES(dev_request_pqueue, struct dev_request_s *, pqueue_entry);
GCT_CONTAINER_FCNS(dev_request_pqueue, inline, dev_request_pqueue,
                   init, destroy, pop, isempty, head, prev, next, remove);

struct dev_request_status_s
{
# ifdef CONFIG_MUTEK_SCHEDULER
  lock_t lock;
  struct sched_context_s *ctx;
# endif
  bool_t done;
};

inline KROUTINE_EXEC(dev_request_spin_done)
{
  struct dev_request_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_request_status_s *status = rq->pvdata;

  status->done = 1;
}

inline void
dev_request_spin_init(struct dev_request_s *rq,
                      struct dev_request_status_s *status)
{
  status->done = 0;
  rq->pvdata = status;
  kroutine_init(&rq->kr, &dev_request_spin_done, KROUTINE_IMMEDIATE);
}

inline void
dev_request_spin_wait(struct dev_request_status_s *status)
{
# ifdef CONFIG_DEVICE_IRQ
  assert(cpu_is_interruptible());
# endif

  do {
    order_compiler_mem();
  } while (!status->done);
}

# ifdef CONFIG_MUTEK_SCHEDULER
inline KROUTINE_EXEC(dev_request_sched_done)
{
  struct dev_request_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_request_status_s *status = rq->pvdata;

  LOCK_SPIN_IRQ(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  LOCK_RELEASE_IRQ(&status->lock);
}

inline void
dev_request_sched_init(struct dev_request_s *rq,
                       struct dev_request_status_s *status)
{
  status->done = 0;
  lock_init(&status->lock);
  status->ctx = NULL;
  rq->pvdata = status;
  kroutine_init(&rq->kr, &dev_request_sched_done, KROUTINE_IMMEDIATE);
}

inline void
dev_request_sched_wait(struct dev_request_status_s *status)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status->lock);

  if (!status->done)
    {
      status->ctx = sched_get_current();
      sched_stop_unlock(&status->lock);
    }
  else
    lock_release(&status->lock);

  CPU_INTERRUPT_RESTORESTATE;

  lock_destroy(&status->lock);
}
# endif

/** @see dev_request_delay_func_t */
#define DEV_REQUEST_DELAYED_FUNC(n) void (n)(struct device_accessor_s *accessor, \
                                             struct dev_request_s *rq_)

/** @This is the device request processing function which is called
    with interrupts enabled. @see dev_request_dlqueue_s */
typedef DEV_REQUEST_DELAYED_FUNC(dev_request_delayed_func_t);

/** Delayed device request queue. May be used in device drivers where
    requests involve a large amount of computation which can be
    executed with interrupts enabled.

    When the @ref #CONFIG_DEVICE_DELAYED_REQUEST configuration token
    is defined, the requests which are queued when interrupts are
    disabled will be processed later in an other context.

    In the other case, the execution of the function takes place
    immediately. If this is not on option, the driver must enforce
    definition of the configuration token.
*/
struct dev_request_dlqueue_s
{
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  struct kroutine_s          kr;
  dev_request_queue_root_t   queue;
#endif
  dev_request_delayed_func_t *func;
};

#ifdef CONFIG_DEVICE_DELAYED_REQUEST
/** @internal */
extern KROUTINE_EXEC(dev_request_delayed_kr);
#endif

/** @This initializes a delayed device request queue. */
ALWAYS_INLINE void
dev_request_delayed_init(struct dev_request_dlqueue_s *q,
                         dev_request_delayed_func_t *f)
{
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  dev_request_queue_init(&q->queue);
  kroutine_init(&q->kr, dev_request_delayed_kr, KROUTINE_SCHED_SWITCH);
#endif
  q->func = f;
}

/** @This cleanups a delayed device request queue. */
ALWAYS_INLINE void
dev_request_delayed_cleanup(struct dev_request_dlqueue_s *q)
{
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  dev_request_queue_destroy(&q->queue);
#endif
}

/** @This removes the request from the queue, schedules execution of
    the next request and executes the completion kroutine of the
    request. This must be called from the @ref
    dev_request_delayed_func_t processing function of the driver. */
inline void
dev_request_delayed_end(struct dev_request_dlqueue_s *q,
                        struct dev_request_s *rq)
{
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  struct device_accessor_s *accessor = rq->drvdata;

  if (accessor != NULL)
    {
      struct device_s *dev = accessor->dev;
      LOCK_SPIN_IRQ(&dev->lock);
      dev_request_queue_remove(&q->queue, rq);
      if (!dev_request_queue_isempty(&q->queue))
        kroutine_exec(&q->kr); /* delayed exec next rq */
      LOCK_RELEASE_IRQ(&dev->lock);
    }
  kroutine_exec(&rq->kr);
#endif
}

/** @This tests if a delayed device request queue is idle. */
ALWAYS_INLINE bool_t
dev_request_delayed_isidle(struct dev_request_dlqueue_s *q)
{
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  return dev_request_queue_isempty(&q->queue);
#else
  return 1;
#endif
}

/** @This pushes the device request in a @ref dev_request_dlqueue_s
    delayed execution queue. if interrupts are enabled and either the
    @tt critical parameter is not set or the queue is empty, the
    processing function is called immediately. In other cases, the
    call is delayed for execution from an interruptible context.

    When the @tt critical parameter is set, requests are not handled
    in parallel. This means that the next call to the @ref
    dev_request_delayed_func_t processing function will not occur
    before the call to the @ref dev_request_delayed_end function for
    the current request.

    When the @ref #CONFIG_DEVICE_DELAYED_REQUEST token is not defined,
    no queue is used and critical calls will make processing run with
    interrupts disabled.
*/
inline void
dev_request_delayed_push(struct device_accessor_s *accessor,
                         struct dev_request_dlqueue_s *q,
                         struct dev_request_s *rq, bool_t critical)
{
  struct device_s *dev = accessor->dev;
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  bool_t interruptible = cpu_is_interruptible();
  bool_t empty = 1;
  rq->drvdata = NULL;
  if (critical || !interruptible)
    {
      LOCK_SPIN_IRQ(&dev->lock);
      empty = dev_request_queue_isempty(&q->queue);
      rq->drvdata = accessor;
      dev_request_queue_pushback(&q->queue, rq);
      if (empty && !interruptible)
        kroutine_exec(&q->kr); /* delayed exec */
      LOCK_RELEASE_IRQ(&dev->lock);
    }
  if (empty && interruptible)
    q->func(accessor, rq);
#else
  reg_t irq_state;
  if (critical)
    lock_spin_irq2(&dev->lock, &irq_state);
  q->func(accessor, rq);
  if (critical)
    lock_release_irq2(&dev->lock, &irq_state);
  kroutine_exec(&rq->kr);    /* request end */
#endif
}

#endif

