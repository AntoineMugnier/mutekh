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
 * @module {Core::Devices support library}
 * @short Generic device request
 */

#ifndef __DEVICE_REQUEST_H__
#define __DEVICE_REQUEST_H__

#include <assert.h>
#include <hexo/types.h>
#include <hexo/decls.h>

#include <gct_platform.h>
#include <gct/container_clist.h>
#include <gct/container_avl_p.h>
#include <device/device.h>
#include <device/driver.h>

#include <mutek/kroutine.h>
#ifdef CONFIG_MUTEK_CONTEXT_SCHED
# include <mutek/scheduler.h>
# include <hexo/lock.h>
# include <hexo/interrupt.h>
#endif

#include <hexo/decls.h>
#include <assert.h>

/* Container algorithm used for of queue device requests */
#define GCT_CONTAINER_ALGO_dev_request_queue CLIST

/* Container algorithm used for priority queue of device requests */
// For now, AVL_P has no support for collisions
//#define GCT_CONTAINER_ALGO_dev_request_pqueue AVL_P
#define GCT_CONTAINER_ALGO_dev_request_pqueue CLIST

/** Device request base */
struct dev_request_s
{
  union {
    struct kroutine_s                     kr;
    FIELD_ALIAS(struct kroutine_s,        pv,
                error_t,                  error);
  };

  union {
    GCT_CONTAINER_ENTRY(dev_request_queue, queue_entry);
    GCT_CONTAINER_ENTRY(dev_request_pqueue, pqueue_entry);
    uintptr_t pushed;
  };

  /** Caller private data */
  union {
    void                                *pvdata;
    uintptr_t                           pvuint;
  };

  /** Driver private data */
  union {
    void                                *drvdata;
    uintptr_t                           drvuint;
  };
};

STRUCT_COMPOSE(dev_request_s, kr);

GCT_CONTAINER_TYPES(dev_request_queue, struct dev_request_s *, queue_entry);

GCT_CONTAINER_FCNS(dev_request_queue, inline, dev_rq_queue,
                   init, destroy, isempty);

GCT_CONTAINER_FCNS(dev_request_queue, inline, __dev_rq_queue,
                   pushback, pop, remove, head);

#define DEV_REQUEST_QUEUE_OPS(class_)                                   \
                                                                        \
/** @This returns the oldest class_ request in the given queue.         \
    A @tt NULL pointer is returned if the queue is empty.               \
    For use in device drivers. */                                       \
ALWAYS_INLINE struct dev_##class_##_rq_s *                              \
dev_##class_##_rq_head(dev_request_queue_root_t *q)                     \
{                                                                       \
  return dev_##class_##_rq_s##_cast(__dev_rq_queue_head(q));            \
}                                                                       \
                                                                        \
/** @This insert a new class_ request at the end of the given queue.    \
    For use in device drivers. */                                       \
ALWAYS_INLINE void                                                      \
dev_##class_##_rq_pushback(dev_request_queue_root_t *q,                 \
                           struct dev_##class_##_rq_s *rq)              \
{                                                                       \
  assert(rq->base.pushed == 0xdead);                                    \
  __dev_rq_queue_pushback(q, &rq->base);                                \
}                                                                       \
                                                                        \
/** @This removes and return the oldest class_ request in the queue.    \
    A @tt NULL pointer is returned if the queue is empty.               \
    For use in device drivers. */                                       \
ALWAYS_INLINE struct dev_##class_##_rq_s *                              \
dev_##class_##_rq_pop(dev_request_queue_root_t *q)                      \
{                                                                       \
  struct dev_##class_##_rq_s *rq;                                       \
  rq = dev_##class_##_rq_s##_cast(__dev_rq_queue_pop(q));               \
  IFASSERT(if (rq) rq->base.pushed = 0xdead);                           \
  return rq;                                                            \
}                                                                       \
                                                                        \
/** @This removes the specified class_ request from the queue.          \
    For use in device drivers. */                                       \
ALWAYS_INLINE void                                                      \
dev_##class_##_rq_remove(dev_request_queue_root_t *q, struct dev_##class_##_rq_s *rq) \
{                                                                       \
  assert(rq->base.pushed != 0xdead);                                    \
  __dev_rq_queue_remove(q, &rq->base);                                  \
  IFASSERT(rq->base.pushed = 0xdead);                                   \
}

GCT_CONTAINER_TYPES(dev_request_pqueue, struct dev_request_s *, pqueue_entry);

GCT_CONTAINER_FCNS(dev_request_pqueue, inline, dev_rq_pqueue,
                   init, destroy, isempty);

GCT_CONTAINER_FCNS(dev_request_pqueue, inline, __dev_rq_pqueue,
                   head, prev, next);

#define DEV_REQUEST_PQUEUE_OPS(class_)                                  \
                                                                        \
/** @This returns the first class_ request in the given priority queue. \
    A @tt NULL pointer is returned if the queue is empty.               \
    For use in device drivers. */                                       \
ALWAYS_INLINE struct dev_##class_##_rq_s *                              \
dev_##class_##_rq_head(dev_request_pqueue_root_t *q)                    \
{                                                                       \
  return dev_##class_##_rq_s##_cast(__dev_rq_pqueue_head(q));           \
}                                                                       \
                                                                        \
/** @This returns the class_ request after the given request in the     \
    priority queue. A @tt NULL pointer is returned if request is the    \
    last one. For use in device drivers. */                             \
ALWAYS_INLINE struct dev_##class_##_rq_s *                              \
dev_##class_##_rq_next(dev_request_pqueue_root_t *q,                    \
                       struct dev_##class_##_rq_s *rq)                  \
{                                                                       \
  return dev_##class_##_rq_s##_cast(__dev_rq_pqueue_next(q, &rq->base));    \
}                                                                       \
                                                                        \
/** @This returns the class_ request before the given request in the    \
    priority queue. A @tt NULL pointer is returned if request is the    \
    first one. For use in device drivers. */                            \
ALWAYS_INLINE struct dev_##class_##_rq_s *                              \
dev_##class_##_rq_prev(dev_request_pqueue_root_t *q,                    \
                       struct dev_##class_##_rq_s *rq)                  \
{                                                                       \
  return dev_##class_##_rq_s##_cast(__dev_rq_pqueue_prev(q, &rq->base));    \
}                                                                       \
                                                                        \
/** @This insert a new class_ request at the right location             \
    in the given priority queue. For use in device drivers. */          \
ALWAYS_INLINE void                                                      \
dev_##class_##_rq_insert(dev_request_pqueue_root_t *q,                  \
                         struct dev_##class_##_rq_s *rq)                \
{                                                                       \
  assert(rq->base.pushed == 0xdead);                                    \
  __dev_##class_##_pqueue_insert(q, &rq->base);                         \
}                                                                       \
                                                                        \
/** @This removes the specified class_ request from the priority queue. \
    For use in device drivers.  */                                      \
ALWAYS_INLINE void                                                      \
dev_##class_##_rq_remove(dev_request_pqueue_root_t *q,                  \
                         struct dev_##class_##_rq_s *rq)                \
{                                                                       \
  assert(rq->base.pushed != 0xdead);                                    \
  __dev_##class_##_pqueue_remove(q, &rq->base);                         \
  IFASSERT(rq->base.pushed = 0xdead);                                   \
}

#define DEV_REQUEST_INHERIT(class_)                                     \
STRUCT_INHERIT(dev_##class_##_rq_s, dev_request_s, base);               \
                                                                        \
/** @This initializes the given class_ request callback.                \
    This must be used before submitting the request to a driver. */     \
ALWAYS_INLINE void                                                      \
dev_##class_##_rq_init_immediate(struct dev_##class_##_rq_s *rq,        \
                                 kroutine_exec_t *exec)                 \
{                                                                       \
  kroutine_init_immediate(&rq->base.kr, exec);                          \
  IFASSERT(rq->base.pushed = 0xdead);                                   \
}                                                                       \
                                                                        \
/** @This initializes the given class_ request callback.                \
    This must be used before submitting the request to a driver. */     \
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,                 \
void dev_##class_##_rq_init(struct dev_##class_##_rq_s *rq,             \
                            kroutine_exec_t *exec),                     \
{                                                                       \
  kroutine_init_deferred(&rq->base.kr, exec);                           \
  IFASSERT(rq->base.pushed = 0xdead);                                   \
});                                                                     \
                                                                        \
/** @This initializes the given class_ request callback.                \
    This must be used before submitting the request to a driver. */     \
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,                 \
void dev_##class_##_rq_init_seq(struct dev_##class_##_rq_s *rq,         \
                                kroutine_exec_t *exec,                  \
                                struct kroutine_sequence_s *seq),       \
{                                                                       \
  kroutine_init_deferred_seq(&rq->base.kr, exec, seq);                  \
  IFASSERT(rq->base.pushed = 0xdead);                                   \
});                                                                     \
                                                                        \
/** @This initializes the given class_ request callback.                \
    This must be used before submitting the request to a driver. */     \
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_QUEUE,                 \
void dev_##class_##_rq_init_queue(struct dev_##class_##_rq_s *rq,       \
                                  kroutine_exec_t *exec,                \
                                  struct kroutine_queue_s *queue),      \
{                                                                       \
  kroutine_init_queue(&rq->base.kr, exec, queue);                       \
  IFASSERT(rq->base.pushed = 0xdead);                                   \
})                                                                      \
                                                                        \
/** @This invokes or schedules execution of the class_ request          \
    callback associated to the request. For use in device drivers. */   \
ALWAYS_INLINE void                                                      \
dev_##class_##_rq_done(struct dev_##class_##_rq_s *rq)                  \
{                                                                       \
  assert(rq->base.pushed == 0xdead);                                    \
  kroutine_exec(&rq->base.kr);                                          \
}                                                                       \
                                                                        \
/** @This retrieves a pointer to the request when in the request        \
    completion callback routine. */                                     \
ALWAYS_INLINE struct dev_##class_##_rq_s *                              \
dev_##class_##_rq_from_kr(struct kroutine_s *kr)                        \
{                                                                       \
  return dev_##class_##_rq_s##_cast(dev_request_s_from_kr(kr));         \
}

struct dev_request_status_s
{
# ifdef CONFIG_MUTEK_CONTEXT_SCHED
  lock_t lock;
  struct sched_context_s *ctx;
# endif
  bool_t done;
};

# ifdef CONFIG_MUTEK_CONTEXT_SCHED
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
  IFASSERT(rq->pushed = 0xdead);
  kroutine_init_immediate(&rq->kr, &dev_request_sched_done);
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

#define DEV_REQUEST_WAIT_FUNC(class_)                                     \
/** This function use the scheduler api to put current context in       \
    wait state during the request. This can only be called from a       \
    scheduler context. */                                               \
config_depend_inline(CONFIG_MUTEK_CONTEXT_SCHED,                        \
error_t dev_##class_##_wait_rq(const struct device_##class_##_s *acc,   \
                               struct dev_##class_##_rq_s *rq),         \
{                                                                       \
    struct dev_request_status_s status;                                 \
    dev_request_sched_init(&rq->base, &status);                         \
    DEVICE_OP((struct device_##class_##_s *)acc, request, rq);          \
    dev_request_sched_wait(&status);                                    \
    return rq->error;                                                   \
})

#ifdef CONFIG_MUTEK_SEMAPHORE

struct semaphore_poll_s;

/** @internal */
KROUTINE_EXEC(dev_request_sem_done);

/** @This setups a device request for use in a semaphore poll construct.
    The @tt kr and @tt pvdata fields of the request are initialized.
    @see semaphore_poll_init */
config_depend_inline(CONFIG_MUTEK_SEMAPHORE,
void dev_request_poll_init(struct dev_request_s *rq,
                           const struct semaphore_poll_s *give),
{
  IFASSERT(rq->pushed = 0xdead);
  kroutine_init_immediate(&rq->kr, &dev_request_sem_done);
  rq->pvdata = (void*)give;
})

# endif

/** @see dev_request_delay_func_t */
#define DEV_REQUEST_DELAYED_FUNC(n) void (n)(const struct device_accessor_s *accessor, \
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
  dev_rq_queue_init(&q->queue);
  kroutine_init_sched_switch(&q->kr, dev_request_delayed_kr);
#endif
  q->func = f;
}

/** @This cleanups a delayed device request queue. */
ALWAYS_INLINE void
dev_request_delayed_cleanup(struct dev_request_dlqueue_s *q)
{
#ifdef CONFIG_DEVICE_DELAYED_REQUEST
  dev_rq_queue_destroy(&q->queue);
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
  const struct device_accessor_s *accessor = rq->drvdata;

  if (accessor != NULL)
    {
      struct device_s *dev = accessor->dev;
      LOCK_SPIN_IRQ(&dev->lock);
      __dev_rq_queue_remove(&q->queue, rq);
      if (!dev_rq_queue_isempty(&q->queue))
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
  return dev_rq_queue_isempty(&q->queue);
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
dev_request_delayed_push(const struct device_accessor_s *accessor,
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
      empty = dev_rq_queue_isempty(&q->queue);
      rq->drvdata = (void*)accessor;
      __dev_rq_queue_pushback(&q->queue, rq);
      if (empty && !interruptible)
        kroutine_exec(&q->kr); /* delayed exec */
      LOCK_RELEASE_IRQ(&dev->lock);
    }
  if (empty && interruptible)
    q->func(accessor, rq);
#else
  cpu_irq_state_t irq_state;
  if (critical)
    lock_spin_irq2(&dev->lock, &irq_state);
  q->func(accessor, rq);
  if (critical)
    lock_release_irq2(&dev->lock, &irq_state);
  kroutine_exec(&rq->kr);    /* request end */
#endif
}

#endif

