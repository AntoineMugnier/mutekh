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
#include <pthread.h>

#include <hexo/error.h>
#include <hexo/alloc.h>
#include <hexo/local.h>
#include <hexo/types.h>
#include <hexo/scheduler.h>
#include <hexo/segment.h>

/** pointer to current thread */
CONTEXT_LOCAL pthread_t __pthread_current;

/** switch to next thread */
void
__pthread_switch(void)
{
  assert(cpu_interrupt_getstate());

#ifdef CONFIG_PTHREAD_CANCEL
  if (pthread_self()->cancelasync)
    pthread_testcancel();
#endif

  cpu_interrupt_disable();
  sched_context_switch();
  cpu_interrupt_enable();
}


static void
__pthread_cleanup(void)
{
  struct pthread_s *thread = pthread_self();

  /* cleanup current context */
  arch_contextstack_free(context_destroy(&thread->sched_ctx.context));

  sched_queue_destroy(&thread->joined);

  /* free thread structure */
  mem_free(thread);

  /* schduler context switch without saving */
  sched_context_exit();
}

#ifdef CONFIG_PTHREAD_CANCEL

/** cancelation context linked list head */
CONTEXT_LOCAL struct __pthread_cleanup_s *__pthread_cleanup_list = NULL;

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

static reg_t	tmp_stack[64];

/** end pthread execution */
void
pthread_exit(void *retval)
{
  struct pthread_s	*thread = pthread_self();

  /* remove thread from runnable list */
  cpu_interrupt_disable();
  sched_queue_wrlock(&thread->joined);

#ifdef CONFIG_PTHREAD_JOIN
  if (!thread->detached)
    {
      if (sched_queue_nolock_isempty(&thread->joined))
	/* thread not joined yet */
	{
	  /* mark thread as joinable */
	  thread->joinable = 1;

	  thread->joined_retval = retval;

	  /* do not release irq yet, thread must not be interrupted before its end */
	  sched_queue_unlock(&thread->joined);

	  /* stop thread, waiting for pthread_join or pthread_detach */
	  sched_context_stop();
	}
      else
	/* thread already joined */
	{
	  struct sched_context_s	*ctx;

	  /* wake up joined threads */
	  while ((ctx = sched_wake(&thread->joined)))
	    {
	      struct pthread_s	*thread = ctx->private;

	      thread->joined_retval = retval;
	    }
	}
    }
#endif /* CONFIG_PTHREAD_JOIN */

  sched_lock();

  /* setup temp stack memory and jump to __pthread_cleanup() */
  cpu_context_set((uintptr_t)(tmp_stack - 1) + sizeof(tmp_stack),
		     __pthread_cleanup);
}



#ifdef CONFIG_PTHREAD_JOIN

/** wait for thread termination */
error_t
pthread_join(pthread_t thread, void **value_ptr)
{
  error_t	res = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&thread->joined);

# ifdef CONFIG_PTHREAD_CHECK
  if (thread->detached)
    {
      sched_queue_unlock(&thread->joined);
      res = EINVAL;
    }
  else
# endif
  {
    if (thread->joinable)
      {
	*value_ptr = thread->joined_retval;

	/* wake up terminated thread waiting for join */
	sched_context_start(&thread->sched_ctx);

	sched_queue_unlock(&thread->joined);
      }
    else
      {
	/* wait for thread termination */
	sched_wait_unlock(&thread->joined);

	*value_ptr = pthread_self()->joined_retval;
      }
  }

  CPU_INTERRUPT_RESTORESTATE;

  return res;
}



/** detach pthread */
error_t
pthread_detach(pthread_t thread)
{
  error_t	res = 0;

  sched_queue_wrlock(&thread->joined);

# ifdef CONFIG_PTHREAD_CHECK
  if (thread->detached)
    {
      sched_queue_unlock(&thread->joined);
      res = EINVAL;
    }
  else
# endif
    {
      thread->detached = 1;

      if (thread->joinable)
	sched_context_start(&thread->sched_ctx);

      sched_queue_unlock(&thread->joined);
    }

  return res;
}

#endif /* CONFIG_PTHREAD_JOIN */


/** thread context entry point */

static CONTEXT_ENTRY(pthread_context_entry)
{
  struct pthread_s	*thread = param;

  CONTEXT_LOCAL_SET(__pthread_current, thread);

  /* release lock acquired in previous sched_context_switch() call */
  sched_unlock();

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
  reg_t			*stack;

  thread = mem_alloc(sizeof (struct pthread_s), MEM_SCOPE_SYS);

  if (!thread)
    return ENOMEM;

  stack = arch_contextstack_alloc(CONFIG_HEXO_SCHED_IDLE_STACK_SIZE * sizeof(reg_t));

  if (stack == NULL)
    {
      mem_free(thread);
      return ENOMEM;
    }

  /* setup context for new thread */
  res = context_init(&thread->sched_ctx.context, stack, CONFIG_PTHREAD_STACK_SIZE, pthread_context_entry, thread);

  if (res)
    {
      mem_free(thread);
      arch_contextstack_free(stack);
      return res;
    }

  sched_context_init(&thread->sched_ctx);
  thread->sched_ctx.private = thread;

  sched_queue_init(&thread->joined);
  thread->start_routine = start_routine;
  thread->arg = arg;
  thread->joinable = 0;
  thread->detached = 0;
#ifdef CONFIG_PTHREAD_CANCEL
  thread->canceled = 0;
  thread->cancelstate = PTHREAD_CANCEL_ENABLE;
  thread->cancelasync = PTHREAD_CANCEL_DEFERRED;
#endif

  *thread_ = thread;

  /* add new thread runnable threads list */
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_context_start(&thread->sched_ctx);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

