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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef MUTEK_SCHEDULER_H_
#define MUTEK_SCHEDULER_H_

/**
 * @file
 * @module{Mutek}
 * @short Kernel execution context scheduler
 */

#include <hexo/decls.h>
#include <hexo/context.h>

struct sched_context_s;

/** scheduler context candidate checking function */
#define SCHED_CANDIDATE_FCN(n) bool_t (n)(struct sched_context_s *sched_ctx)

/** scheduler context candidate checking function type */
typedef SCHED_CANDIDATE_FCN(sched_candidate_fcn_t);


#ifdef CONFIG_MUTEK_SCHEDULER

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_dlist.h>

# define CONTAINER_LOCK_sched_queue HEXO_SPIN

struct sched_context_s
{
  CONTAINER_ENTRY_TYPE(DLIST) list_entry;
  struct scheduler_s *scheduler;		//< keep track of associated scheduler queue
  struct context_s	context;	//< execution context

  void			*priv;

# ifdef CONFIG_MUTEK_SCHEDULER_MIGRATION_AFFINITY
  cpu_bitmap_t		cpu_map;
# endif

# ifdef CONFIG_MUTEK_SCHEDULER_CANDIDATE_FCN
  sched_candidate_fcn_t	*is_candidate;
# endif

};

CONTAINER_TYPE       (sched_queue, DLIST, struct sched_context_s, list_entry);

CONTAINER_FUNC       (sched_queue, DLIST, static inline, sched_queue, list_entry);
CONTAINER_FUNC_NOLOCK(sched_queue, DLIST, static inline, sched_queue_nolock, list_entry);

# define SCHED_QUEUE_INITIALIZER CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST)

/** @internal */
extern CONTEXT_LOCAL struct sched_context_s *sched_cur;
/** @internal */
extern CPU_LOCAL struct sched_context_s sched_idle;

#else
typedef struct __empty_s sched_queue_root_t;
#endif

/** @this return current scheduler context */
config_depend_inline(CONFIG_MUTEK_SCHEDULER,
struct sched_context_s *sched_get_current(void),
{
  return CONTEXT_LOCAL_GET(sched_cur);
});

/** @this return a cpu local context for temporary stack use with @ref
    cpu_context_stack_use. This context must be used with interupts
    disabled. This is useful during context exit/destroy. */
config_depend_inline(CONFIG_MUTEK_SCHEDULER,
struct context_s * sched_tmp_context(void),
{
  return &CPU_LOCAL_ADDR(sched_idle)->context;
});

/** scheduler context preemption handler.
    Push current context back in running queue and
    return next scheduler candidate for preemption.
    @see context_set_preempt @see #CONTEXT_PREEMPT */
config_depend(CONFIG_MUTEK_SCHEDULER)
CONTEXT_PREEMPT(sched_preempt_switch);

/** scheduler context preemption handler.
    Return next scheduler candidate for preemption.
    @see context_set_preempt @see #CONTEXT_PREEMPT */
config_depend(CONFIG_MUTEK_SCHEDULER)
CONTEXT_PREEMPT(sched_preempt_stop);

/** scheduler context preemption handler.
    Return next scheduler candidate for preemption.
    @see context_set_preempt @see #CONTEXT_PREEMPT */
config_depend(CONFIG_MUTEK_SCHEDULER)
CONTEXT_PREEMPT(sched_preempt_wait_unlock);


/** initialize scheduler context. context_init(&sched_ctx->context)
    must be called before */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_context_init(struct sched_context_s *sched_ctx);

/** switch to next context */
/* Must be called with interrupts disabled */
config_depend_inline(CONFIG_MUTEK_SCHEDULER,
void sched_context_switch(void),
{
  struct context_s *next = sched_preempt_switch(NULL);

  if (next)
    context_switch_to(next);
});

/** jump to next context without saving current context. current
    context will be lost. Must be called with interrupts disabled and
    main sched queue locked */
/* Must be called with interrupts disabled */
config_depend_inline(CONFIG_MUTEK_SCHEDULER,
void sched_context_exit(void),
{
  context_jump_to(sched_preempt_stop(NULL));
});

/** push current context in the 'queue', unlock it and switch to next
   context available in the 'root' queue. Must be called with
   interrupts disabled */
config_depend_inline(CONFIG_MUTEK_SCHEDULER,
void sched_wait_unlock(sched_queue_root_t *queue),
{
  context_switch_to(sched_preempt_wait_unlock(queue));
});

/** enqueue scheduler context for execution. Must be called with
    interrupts disabled */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_context_start(struct sched_context_s *sched_ctx);

/** switch to next context without pushing current context back. Must
    be called with interrupts disabled */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_stop_unlock(lock_t *lock);

/** lock context queue */
config_depend(CONFIG_MUTEK_SCHEDULER)
error_t sched_queue_lock(sched_queue_root_t *queue);

/** unlock context queue */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_queue_unlock(sched_queue_root_t *queue);

/** init context queue */
config_depend(CONFIG_MUTEK_SCHEDULER)
error_t sched_queue_init(sched_queue_root_t *queue);

/** destroy context queue */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_queue_destroy(sched_queue_root_t *queue);

typedef void (sched_wait_cb_t)(void *ctx);

/** Remove first context from the queue and push it back in run queue.
    @return context or NULL if none found on queue.
    Must be called with interrupts disabled and queue locked. */
config_depend(CONFIG_MUTEK_SCHEDULER)
struct sched_context_s *sched_wake(sched_queue_root_t *queue);

/** Remove the context from its queue and push it back in run queue.
    Must be called with interrupts disabled and queue locked. */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_context_wake(sched_queue_root_t *queue, struct sched_context_s *sched_ctx);

/** scheduler intialization, must be called once */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_global_init(void);

/** scheduler intialization, must be called for each processor */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_cpu_init(void);

/** scheduler context will run on this cpu */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** scheduler context will not run on this cpu */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** scheduler context will run on a single cpu */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** scheduler context will run on all cpu */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_affinity_all(struct sched_context_s *sched_ctx);

/** scheduler context will run on all cpu */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_affinity_clear(struct sched_context_s *sched_ctx);

/** setup a scheduler context candidate checking function */
config_depend(CONFIG_MUTEK_SCHEDULER)
void sched_context_candidate_fcn(struct sched_context_s *sched_ctx, sched_candidate_fcn_t *fcn);

#endif

