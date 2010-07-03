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

#ifndef MUTEK_SCHEDULER_H_
#define MUTEK_SCHEDULER_H_

/**
 * @file
 * @module{Mutek}
 * @short Kernel execution context scheduler
 */

#ifndef CONFIG_MUTEK_SCHEDULER
# warning hexo scheduler is not enabled in configuration file
#else

#include <hexo/context.h>
#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_dlist.h>

struct sched_context_s;

/** scheduler context candidate checking function */
#define SCHED_CANDIDATE_FCN(n) bool_t (n)(struct sched_context_s *sched_ctx)

/** scheduler context candidate checking function type */
typedef SCHED_CANDIDATE_FCN(sched_candidate_fcn_t);



#define CONTAINER_LOCK_sched_queue HEXO_SPIN

struct sched_context_s
{
  CONTAINER_ENTRY_TYPE(DLIST) list_entry;
  struct scheduler_s *scheduler;		//< keep track of associated scheduler queue
  struct context_s	context;	//< execution context

  void			*priv;

#ifdef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
  cpu_bitmap_t		cpu_map;
#endif

#ifdef CONFIG_MUTEK_SCHEDULER_CANDIDATE_FCN
  sched_candidate_fcn_t	*is_candidate;
#endif

};

CONTAINER_TYPE       (sched_queue, DLIST, struct sched_context_s, list_entry);

CONTAINER_FUNC       (sched_queue, DLIST, static inline, sched_queue, list_entry);
CONTAINER_FUNC_NOLOCK(sched_queue, DLIST, static inline, sched_queue_nolock, list_entry);

#define SCHED_QUEUE_INITIALIZER CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST)


/** @internal */
extern CONTEXT_LOCAL struct sched_context_s *sched_cur;
/** @internal */
extern CPU_LOCAL struct sched_context_s sched_idle;


/** @this return current scheduler context */
static inline struct sched_context_s *
sched_get_current(void)
{
  return CONTEXT_LOCAL_GET(sched_cur);
}

/** @this return a cpu local context for temporary stack use with @ref
    cpu_context_stack_use. This context must be used with interupts
    disabled. This is useful during context exit/destroy. */
static inline struct context_s * sched_tmp_context(void)
{
  return &CPU_LOCAL_ADDR(sched_idle)->context;
}

/** scheduler context preemption handler.
    Push current context back in running queue and
    return next scheduler candidate for preemption.
    @see context_set_preempt @see #CONTEXT_PREEMPT */
CONTEXT_PREEMPT(sched_preempt_switch);

/** scheduler context preemption handler.
    Return next scheduler candidate for preemption.
    @see context_set_preempt @see #CONTEXT_PREEMPT */
CONTEXT_PREEMPT(sched_preempt_stop);

/** scheduler context preemption handler.
    Return next scheduler candidate for preemption.
    @see context_set_preempt @see #CONTEXT_PREEMPT */
CONTEXT_PREEMPT(sched_preempt_wait_unlock);


/** initialize scheduler context. context_init(&sched_ctx->context)
    must be called before */
void sched_context_init(struct sched_context_s *sched_ctx);

/** switch to next context */
/* Must be called with interrupts disabled */
static inline void sched_context_switch(void)
{
  struct context_s *next = sched_preempt_switch(NULL);

  if (next)
    context_switch_to(next);
}

/** jump to next context without saving current context. current
    context will be lost. Must be called with interrupts disabled and
    main sched queue locked */
/* Must be called with interrupts disabled */
static inline void sched_context_exit(void)
{
  context_jump_to(sched_preempt_stop(NULL));
}

/** push current context in the 'queue', unlock it and switch to next
   context available in the 'root' queue. Must be called with
   interrupts disabled */
static inline void sched_wait_unlock(sched_queue_root_t *queue)
{
  context_switch_to(sched_preempt_wait_unlock(queue));
}

/** enqueue scheduler context for execution. Must be called with
    interrupts disabled */
void sched_context_start(struct sched_context_s *sched_ctx);


/** switch to next context without pushing current context back. Must
    be called with interrupts disabled */
void sched_stop_unlock(lock_t *lock);

/** lock context queue */
error_t sched_queue_lock(sched_queue_root_t *queue);

/** unlock context queue */
void sched_queue_unlock(sched_queue_root_t *queue);

/** init context queue */
error_t sched_queue_init(sched_queue_root_t *queue);

/** destroy context queue */
void sched_queue_destroy(sched_queue_root_t *queue);

typedef void (sched_wait_cb_t)(void *ctx);

/** wake a context from this queue */
/* Must be called with interrupts disabled */
struct sched_context_s *sched_wake(sched_queue_root_t *queue);

/** scheduler intialization, must be called once */
void sched_global_init(void);

/** scheduler intialization, must be called for each processor */
void sched_cpu_init(void);

/** scheduler context will run on this cpu */
void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** scheduler context will not run on this cpu */
void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** scheduler context will run on a single cpu */
void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** scheduler context will run on all cpu */
void sched_affinity_all(struct sched_context_s *sched_ctx);

/** scheduler context will run on all cpu */
void sched_affinity_clear(struct sched_context_s *sched_ctx);

/** setup a scheduler context candidate checking function */
void sched_context_candidate_fcn(struct sched_context_s *sched_ctx, sched_candidate_fcn_t *fcn);


#endif
#endif

