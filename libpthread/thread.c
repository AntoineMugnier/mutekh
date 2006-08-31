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

#include <stdio.h>

#include <hexo/alloc.h>
#include <hexo/types.h>

#include "pthread-private.h"

/** runnable thread list */
struct pthread_pool_s __pthread_runnable;

/** initial thread */
static struct pthread_s init_thread;

/** idle thread */
static struct pthread_s idle_thread;

/** pointer to current thread */
CONTEXT_LOCAL pthread_t __pthread_current;

/** idle thread function */
static CONTEXT_ENTRY(pthread_context_idle)
{
  /* release lock acquired in previous __pthread_switch() call */
  lock_release(&__pthread_runnable.lock);

  while (1)
    {
      printf("I am idle\n");
      __pthread_switch();
    }
}



/** get next thread in pool */
static inline struct pthread_s *
__pthread_run_candidate(struct pthread_pool_s *pool)
{
  struct pthread_s	*thread;

  if ((thread = __pthread_list_pop(&pool->list)))
    {
      __pthread_list_pushback(&pool->list, thread);
      return __pthread_list_head(&pool->list);
    }

  return &idle_thread;
}



/** init pthread sub system and bootstrap initial thread */
void
__pthread_bootstrap(void)
{
  /* initialize runnable pthread pool */
  __pthread_pool_init(&__pthread_runnable);

  /* setup current thread */
  __pthread_init(&init_thread);
  context_bootstrap(&init_thread.context);

  CONTEXT_LOCAL_SET(__pthread_current, &init_thread);

  /* add thread to running threads list */
  __pthread_pool_add(&__pthread_runnable, &init_thread);

  /* setup idle thread */
  __pthread_init(&idle_thread);
  context_init(&idle_thread.context, CONFIG_PTHREAD_STACK_SIZE, pthread_context_idle, 0);
}

void
__pthread_dump_runqueue(void)
{
  pthread_item_t	t;

  if (lock_state(&__pthread_runnable.lock))
    printf("__pthread_dump_runqueue(): __pthread_runnable.lock is held\n");

  lock_spin_irq(&__pthread_runnable.lock);

  t = __pthread_list_head(&__pthread_runnable.list);

  printf("Current pthread: %p (self) %p (head)\n", pthread_self(), t);

  if (t)
    do {
      printf("  [%p] next %p prev %p\n", t,
	     __pthread_list_next(&__pthread_runnable.list, t),
	     __pthread_list_prev(&__pthread_runnable.list, t));
    
      t = __pthread_list_next(&__pthread_runnable.list, t);

    } while (t != __pthread_list_head(&__pthread_runnable.list));

  lock_release_irq(&__pthread_runnable.lock);
}

/** switch to next __pthread_runnable thread */
void
__pthread_switch(void)
{
  struct pthread_s	*candidate;
  __reg_t		irq_state;

#ifdef CONFIG_PTHREAD_CANCEL
  if (pthread_self()->cancelasync)
    pthread_testcancel();
#endif

  cpu_interrupt_savestate_disable(&irq_state);
  lock_spin(&__pthread_runnable.lock);

  /* get next candidate for execution */
  candidate = __pthread_run_candidate(&__pthread_runnable);

  context_switch_to(&candidate->context);

  lock_release(&__pthread_runnable.lock);
  cpu_interrupt_restorestate(&irq_state);
}



static void
__pthread_cleanup(void)
{
  struct pthread_s *thread = pthread_self();
  struct pthread_s *candidate;

  /* cleanup current context */
  context_destroy(&thread->context);

  /* free thread structure */
  mem_free(thread);

  /* jump to cadidate thread */
  candidate = __pthread_run_candidate(&__pthread_runnable);
  context_jump_to(&candidate->context);
}

#ifdef CONFIG_PTHREAD_CANCEL

/** cancelation context linked list head */
CONTEXT_LOCAL struct __pthread_cleanup_s *__pthread_cleanup_list = 0;

void __pthread_cancel_self(void)
{
  struct __pthread_cleanup_s		*c;

  cpu_interrupt_disable();

  /* call thread cleanup handlers */
  for (c = CONTEXT_LOCAL_GET(__pthread_cleanup_list); c; c = c->prev)
    c->fcn(c->arg);

  pthread_exit(PTHREAD_CANCELED);
}

#endif /* CONFIG_PTHREAD_CANCEL */

/** end pthread execution */
void
pthread_exit(void *retval)
{
  struct pthread_s	*thread = pthread_self();
  static __reg_t	tmp_stack[64];

  /* remove thread from runnable list */
  lock_spin_irq(&__pthread_runnable.lock);
  __pthread_list_pop(&__pthread_runnable.list);

#ifdef CONFIG_PTHREAD_JOIN
  if (!thread->detached)
    {
      if (!thread->joined)
	/* thread not joined yet */
	{
	  /* mark thread as joinable */
	  thread->joinable = 1;

	  thread->joined_retval = retval;

	  /* do not release irq yet, thread must not be interrupted before its end */
	  lock_release(&__pthread_runnable.lock);

	  /* stop thread, waiting for pthread_join or pthread_detach */
	  __pthread_switch();

	  /* remove from run queue again */
	  lock_spin(&__pthread_runnable.lock);

	  __pthread_list_pop(&__pthread_runnable.list);
	}
      else
	/* thread already joined */
	{
	  thread->joined->joined_retval = retval;

	  /* wake up joined thread */
	  __pthread_list_pushback(&__pthread_runnable.list, thread->joined);
	}
    }
#endif /* CONFIG_PTHREAD_JOIN */

  /* __pthread_runnable lock must still be held here to protect
     temporary stack, it will be released before __pthread_switch()
     end */

  /* setup temp stack memory and jump to __pthread_cleanup() */
  cpu_context_set_stack((uintptr_t)(tmp_stack - 1) + sizeof(tmp_stack),
		     __pthread_cleanup);
}



#ifdef CONFIG_PTHREAD_JOIN

/** wait for thread termination */
error_t
pthread_join(pthread_t thread, void **value_ptr)
{
  lock_spin_irq(&__pthread_runnable.lock);

# ifdef CONFIG_PTHREAD_CHECK
  if (thread->detached)
    {
      lock_release_irq(&__pthread_runnable.lock);
      return EINVAL;
    }
# endif

  if (thread->joinable)
    {
      *value_ptr = thread->joined_retval;

      /* wake up terminated thread waiting for join */
      __pthread_list_pushback(&__pthread_runnable.list, thread);

      lock_release_irq(&__pthread_runnable.lock);
    }
  else
    {
      /* wait for thread termination */
      thread->joined = __pthread_list_pop(&__pthread_runnable.list);

      lock_release_irq(&__pthread_runnable.lock);

      __pthread_switch();

      *value_ptr = pthread_self()->joined_retval;
    }

  return 0;
}



/** detach pthread */
error_t
pthread_detach(pthread_t thread)
{
  lock_spin_irq(&__pthread_runnable.lock);

# ifdef CONFIG_PTHREAD_CHECK
  if (thread->detached)
    {
      lock_release_irq(&__pthread_runnable.lock);
      return EINVAL;
    }
# endif

  thread->detached = 1;

  if (thread->joinable)
    __pthread_list_pushback(&__pthread_runnable.list, thread);

  lock_release_irq(&__pthread_runnable.lock);

  return 0;
}

#endif /* CONFIG_PTHREAD_JOIN */

/** thread context entry point */

static CONTEXT_ENTRY(pthread_context_entry)
{
  struct pthread_s	*thread = param;

  CONTEXT_LOCAL_SET(__pthread_current, thread);

  /* release lock acquired in previous __pthread_switch() call */
  lock_release(&__pthread_runnable.lock);

  /* enable interrupts for current thread */
  cpu_interrupt_enable();	/* FIXME should reflect state at thread creation time ? */

  /* call pthread_exit with return value if thread main functions
     returns */
  pthread_exit(thread->start_routine(thread->arg));
}



/** create a new pthread  */

error_t
pthread_create(pthread_t *thread_, const pthread_attr_t *attr,
	       pthread_start_routine_t *start_routine, void *arg)
{
  struct pthread_s	*thread;
  error_t		res;

  thread = mem_alloc(sizeof (struct pthread_s), MEM_SCOPE_SYS);

  if (!thread)
    return ENOMEM;

  /* setup context context for new thread */
  res = context_init(&thread->context, CONFIG_PTHREAD_STACK_SIZE, pthread_context_entry, thread);

  if (res)
    {
      mem_free(thread);
      return res;
    }

  thread->start_routine = start_routine;
  thread->arg = arg;
  __pthread_init(thread);

  /* add new thread runnable threads list */
  __pthread_pool_add(&__pthread_runnable, thread);

  *thread_ = thread;

  return 0;
}

