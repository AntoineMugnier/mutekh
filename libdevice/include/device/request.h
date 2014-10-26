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
#include <gct/container_avl.h>

#include <mutek/kroutine.h>
#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
# include <hexo/lock.h>
#endif

/* Container algorithm used for of queue device requests */
#define GCT_CONTAINER_ALGO_dev_request_queue CLIST

/* Container algorithm used for priority queue of device requests */
#define GCT_CONTAINER_ALGO_dev_request_pqueue AVL

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
  void                                  *drvdata;
};

STRUCT_COMPOSE(dev_request_s, kr);

GCT_CONTAINER_TYPES(dev_request_queue, struct dev_request_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_request_queue, inline, dev_request_queue,
                   init, destroy, pushback, pop, isempty, head);

GCT_CONTAINER_TYPES(dev_request_pqueue, struct dev_request_s *, pqueue_entry);
GCT_CONTAINER_FCNS(dev_request_pqueue, inline, dev_request_pqueue,
                   init, destroy, pop, isempty, head);

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

  while (!status->done)
    order_compiler_mem();
}

# ifdef CONFIG_MUTEK_SCHEDULER
inline KROUTINE_EXEC(dev_request_sched_done)
{
  struct dev_request_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_request_status_s *status = rq->pvdata;

  lock_spin(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  lock_release(&status->lock);
}

inline void
dev_request_sched_init(struct dev_request_s *rq,
                       struct dev_request_status_s *status)
{
  status->done = 0;
  lock_init(&status->lock);
  status->ctx = NULL;
  rq->pvdata = status;
  kroutine_init(&rq->kr, &dev_request_spin_done, KROUTINE_IMMEDIATE);
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

#endif

