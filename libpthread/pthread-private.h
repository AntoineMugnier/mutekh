/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef PTHREAD_PRIVATE_H_
#define PTHREAD_PRIVATE_H_

#include <pthread.h>
#include <semaphore.h>

#include <mutek/task.h>
#include <mutek/lock.h>

#include <mutek/template/cont_clist.h>

CONTAINER_FUNC(static inline, pthread, CLIST, __pthread_list, NOLOCK, queue);

/** runnable thread list */
extern struct pthread_pool_s __pthread_runnable;



static inline void
__pthread_init(struct pthread_s *thread)
{
  thread->joinable = 0;
  thread->detached = 0;
  thread->joined = 0;
#ifdef CONFIG_PTHREAD_CANCEL
  thread->canceled = 0;
  thread->cancelstate = PTHREAD_CANCEL_ENABLE;
  thread->cancelasync = PTHREAD_CANCEL_DEFERRED;
#endif
}

/** thread pool init */
static inline void
__pthread_pool_init(struct pthread_pool_s *pool)
{
  lock_init(&pool->lock);
  __pthread_list_init(&pool->list);
}

/** add thread to threads pool */
static inline void
__pthread_pool_add(struct pthread_pool_s *pool,
		 struct pthread_s *thread)
{
  lock_spin_irq(&pool->lock);
  __pthread_list_pushback(&pool->list, thread);
  lock_release_irq(&pool->lock);
}

/** put thread in wait queue,
    irqs must be already disabled */
static inline void
__pthread_wait(pthread_cont_t *queue)
{
  struct pthread_s *thread;

  lock_spin(&__pthread_runnable.lock);
  thread = __pthread_list_pop(&__pthread_runnable.list);
  lock_release(&__pthread_runnable.lock);

  __pthread_list_pushback(queue, thread);
}


/** push thread back in its run queue,
    irqs must be already disabled */
static inline void
__pthread_wake(struct pthread_s *thread)
{
  lock_spin(&__pthread_runnable.lock);
  __pthread_list_pushback(&__pthread_runnable.list, thread);  
  lock_release(&__pthread_runnable.lock);
}


#endif

