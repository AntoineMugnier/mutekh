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

#include <mutek/scheduler.h>
#include <hexo/init.h>
#include <hexo/local.h>
#include <hexo/segment.h>
#include <hexo/ipi.h>

/* processor current scheduler context */
CONTEXT_LOCAL struct sched_context_s *sched_cur = NULL;

/* processor idle context */
CPU_LOCAL struct sched_context_s sched_idle;

/* return next scheduler candidate */
static struct sched_context_s *
__sched_candidate_noidle(sched_queue_root_t *root)
{
#ifdef CONFIG_MUTEK_SCHEDULER_CANDIDATE_FCN
  struct sched_context_s *c = NULL;

  CONTAINER_FOREACH_NOLOCK(sched_queue, DLIST, root, {
    if (item->is_candidate == NULL || item->is_candidate(item))
      {
	sched_queue_nolock_remove(root, item);
	c = item;
	CONTAINER_FOREACH_BREAK;
      }
  });

  return c;
#else
  return sched_queue_nolock_pop(root);
#endif
}

/* return next scheduler candidate */
static inline struct sched_context_s *
__sched_candidate(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  if ((next = __sched_candidate_noidle(root)) == NULL)
    next = CPU_LOCAL_ADDR(sched_idle);

  return next;
}

/************************************************************************/

#if defined (CONFIG_MUTEK_SCHEDULER_MIGRATION)

/* scheduler root */
static sched_queue_root_t	sched_root;

# if defined(CONFIG_HEXO_IPI)
/* sleeping cpu list */

CONTAINER_TYPE(sched_cls_queue, CLIST, struct sched_cls_item_s
{
  CONTAINER_ENTRY_TYPE(CLIST)	list_entry;
}, list_entry);

CONTAINER_FUNC(sched_cls_queue, CLIST, static inline, sched_cls_queue, list_entry);

static sched_cls_queue_root_t cls_queue;

static CPU_LOCAL struct sched_cls_item_s sched_cls_item;

#  define GET_CLS_FROM_ITEM(item) ((void*)((uintptr_t)(item) - (uintptr_t)&sched_cls_item))
# endif /* IPI */

/* return scheduler root */
static inline sched_queue_root_t *
__sched_root(void)
{
  return &sched_root;
}

static inline
void __sched_context_push(struct sched_context_s *sched_ctx)
{
	sched_queue_pushback(&sched_root, sched_ctx);
#if defined(CONFIG_HEXO_IPI)
	struct sched_cls_item_s *idle = sched_cls_queue_pop(&cls_queue);
	if ( idle ) {
		ipi_post(GET_CLS_FROM_ITEM(idle));
		sched_cls_queue_pushback(&cls_queue, idle);
	}
#endif /* IPI */
}

/************************************************************************/

#elif defined (CONFIG_MUTEK_SCHEDULER_STATIC)

/* scheduler root */
static CPU_LOCAL sched_queue_root_t	sched_root;

/* return scheduler root */
static inline sched_queue_root_t *
__sched_root(void)
{
  return CPU_LOCAL_ADDR(sched_root);
}

static inline
void __sched_context_push(struct sched_context_s *sched_ctx)
{
	sched_queue_pushback(
		CPU_LOCAL_CLS_ADDR(sched_ctx->cpu_cls, sched_root),
		sched_ctx);
#if defined(CONFIG_HEXO_IPI)
	ipi_post(sched_ctx->cpu_cls);
#endif /* IPI */
}

#endif

/************************************************************************/

/* idle context runtime */
static CONTEXT_ENTRY(sched_context_idle)
{
  sched_queue_root_t *root = __sched_root();

#if defined(CONFIG_MUTEK_SCHEDULER_MIGRATION) && defined(CONFIG_HEXO_IPI)
  sched_cls_queue_push(&cls_queue, CPU_LOCAL_ADDR(sched_cls_item));
#endif

  /* release lock acquired in previous sched_context_switch() call */
  sched_unlock();
  cpu_interrupt_disable();

  while (1)
    {
      struct sched_context_s	*next;

      /* do not wait if several cpus are running because context may
	 be put in running queue by an other cpu with no interrupt */
#if !defined(CONFIG_SMP)
      /* CPU sleep waiting for interrupts */
      cpu_interrupt_wait();
#elif defined(CONFIG_HEXO_IPI)
      if (CPU_LOCAL_GET(ipi_icu_dev))
		  cpu_interrupt_wait();
#endif

      /* Let enough time for pending interrupts to execute and assume
	 memory is clobbered to force scheduler root queue
	 reloading after interrupts execution. */
      cpu_interrupt_process();

      sched_queue_wrlock(root);

      if ((next = __sched_candidate_noidle(root)) != NULL)
	{
#if defined(CONFIG_MUTEK_SCHEDULER_MIGRATION) && defined(CONFIG_HEXO_IPI)
	  sched_cls_queue_remove(&cls_queue, CPU_LOCAL_ADDR(sched_cls_item));
#endif
	  context_switch_to(&next->context);
#if defined(CONFIG_MUTEK_SCHEDULER_MIGRATION) && defined(CONFIG_HEXO_IPI)
	  sched_cls_queue_push(&cls_queue, CPU_LOCAL_ADDR(sched_cls_item));
#endif
	  //	  printk("(c%i idle)", cpu_id());
	}

      sched_queue_unlock(root);
    }
}

/************************************************************************/

/* Switch to next context available in the root queue. This function
   returns if no other context is available, controle is passed
   back to current context rather than Idle context. Must be called
   with interrupts disabled */
void sched_context_switch(void)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_is_interruptible());

  sched_queue_wrlock(root);

  if ((next = __sched_candidate_noidle(root)))
    {
      /* push context back in running queue */
      sched_queue_nolock_pushback(root, CONTEXT_LOCAL_GET(sched_cur));
      context_switch_to(&next->context);
    }

  sched_queue_unlock(root);
}

/* Must be called with interrupts disabled and sched locked */
void sched_context_exit(void)
{
  assert(!cpu_is_interruptible());

  struct sched_context_s	*next;

  /* get next running context */
  next = __sched_candidate(__sched_root());
  context_jump_to(&next->context);
}

void sched_lock(void)
{
  assert(!cpu_is_interruptible());

  sched_queue_wrlock(__sched_root());
}

void sched_unlock(void)
{
  assert(!cpu_is_interruptible());

  sched_queue_unlock(__sched_root());
}

void sched_context_init(struct sched_context_s *sched_ctx)
{
  /* set sched_cur context local variable */
  CONTEXT_LOCAL_TLS_SET(sched_ctx->context.tls,
			sched_cur, sched_ctx);

  sched_ctx->private = NULL;

#if defined (CONFIG_MUTEK_SCHEDULER_STATIC)
  sched_ctx->cpu_cls = (void*)CPU_GET_CLS();
#endif

#ifdef CONFIG_MUTEK_SCHEDULER_CANDIDATE_FCN
  sched_ctx->is_candidate = NULL;
#endif

}

/* Must be called with interrupts disabled */
void sched_context_start(struct sched_context_s *sched_ctx)
{
  assert(!cpu_is_interruptible());

  __sched_context_push(sched_ctx);
}

void sched_wait_callback(sched_queue_root_t *queue,
		         void (*callback)(void *ctx), void *ctx)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_is_interruptible());

  /* add current context to queue, assume dont need lock */
  sched_queue_nolock_pushback(queue, CONTEXT_LOCAL_GET(sched_cur));
  callback(ctx);

  /* get next running context */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* push current context in the 'queue', unlock it and switch to next
   context available in the 'root' queue. Must be called with
   interrupts disabled */
void sched_wait_unlock(sched_queue_root_t *queue)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_is_interruptible());

  /* add current context to queue, assume queue is already locked */
  sched_queue_nolock_pushback(queue, CONTEXT_LOCAL_GET(sched_cur));
  sched_queue_unlock(queue);

  /* get next running context */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* Switch to next context available in the 'root' queue, do not put
   current context in any queue. Idle context may be selected if no
   other contexts are available. Must be called with interrupts
   disabled */
void sched_context_stop(void)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_is_interruptible());

  /* get next running context */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* Same as sched_context_stop but unlock given spinlock before switching */
void sched_context_stop_unlock(lock_t *lock)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_is_interruptible());

  /* get next running context */
  sched_queue_wrlock(root);
  lock_release(lock);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* Must be called with interrupts disabled and queue locked */
struct sched_context_s *sched_wake(sched_queue_root_t *queue)
{
  struct sched_context_s	*sched_ctx;

  assert(!cpu_is_interruptible());

  if ((sched_ctx = sched_queue_nolock_pop(queue)))
	  __sched_context_push(sched_ctx);

  return sched_ctx;
}

void sched_global_init(void)
{
#if defined (CONFIG_MUTEK_SCHEDULER_MIGRATION)
# if defined(CONFIG_HEXO_IPI)
  sched_cls_queue_init(&cls_queue);
# endif
  sched_queue_init(__sched_root());
#endif
}

void sched_cpu_init(void)
{
  struct sched_context_s *idle = CPU_LOCAL_ADDR(sched_idle);
  uint8_t *stack;
  error_t err;

  stack = arch_contextstack_alloc(CONFIG_MUTEK_SCHEDULER_IDLE_STACK_SIZE);

  assert(stack != NULL);

  err = context_init(&idle->context, stack,
		     stack + CONFIG_MUTEK_SCHEDULER_IDLE_STACK_SIZE,
		     sched_context_idle, 0);

  assert(err == 0);

#if defined (CONFIG_MUTEK_SCHEDULER_STATIC)
  sched_queue_init(__sched_root());
#endif
}

#ifdef CONFIG_MUTEK_SCHEDULER_MIGRATION

void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
# ifndef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
# endif
}

void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
# ifndef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
# endif
}

void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
# ifndef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
# endif
}

void sched_affinity_all(struct sched_context_s *sched_ctx)
{
# ifndef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
# endif
}

void sched_affinity_clear(struct sched_context_s *sched_ctx)
{
# ifndef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
# endif
}

#endif

#ifdef CONFIG_MUTEK_SCHEDULER_STATIC

void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
	sched_ctx->cpu_cls = CPU_GET_CLS_ID(cpu);
}

void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
  sched_affinity_add(sched_ctx, cpu);
}

void sched_affinity_all(struct sched_context_s *sched_ctx)
{
}

void sched_affinity_clear(struct sched_context_s *sched_ctx)
{
}

#endif

void sched_context_candidate_fcn(struct sched_context_s *sched_ctx,
				 sched_candidate_fcn_t *fcn)
{
#ifdef CONFIG_MUTEK_SCHEDULER_CANDIDATE_FCN
  sched_ctx->is_candidate = fcn;
#else
  abort();
#endif
}

