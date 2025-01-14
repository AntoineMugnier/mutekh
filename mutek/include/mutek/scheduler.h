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
 * @module {Core::Kernel services}
 * @short Kernel execution context scheduler
 */

#include <hexo/decls.h>
#include <hexo/context.h>

struct sched_context_s;

/** scheduler context candidate checking function */
#define SCHED_CANDIDATE_FCN(n) bool_t (n)(struct sched_context_s *sched_ctx)

/** scheduler context candidate checking function type */
typedef SCHED_CANDIDATE_FCN(sched_candidate_fcn_t);


#ifdef CONFIG_MUTEK_CONTEXT_SCHED

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_dlist.h>

# define GCT_CONTAINER_ALGO_sched_queue DLIST
# define GCT_CONTAINER_LOCK_sched_queue HEXO_LOCK

struct sched_context_s
{
  GCT_CONTAINER_ENTRY(sched_queue, list_entry);
  struct scheduler_s *scheduler;  //< keep track of associated scheduler queue
  struct context_s   *context;	  //< execution context

  void			*priv;

# if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  uint8_t               priority;
# endif

# ifdef CONFIG_MUTEK_CONTEXT_SCHED_MIGRATION_AFFINITY
  cpu_bitmap_t		cpu_map;
# endif

# ifdef CONFIG_MUTEK_CONTEXT_SCHED_CANDIDATE_FCN
  sched_candidate_fcn_t	*is_candidate;
# endif

};

GCT_CONTAINER_TYPES      (sched_queue, struct sched_context_s *, list_entry);

GCT_CONTAINER_FCNS       (sched_queue, inline, sched_queue,
                          init, destroy, isempty, pushback, pop, head, wrlock, rdlock, unlock);

GCT_CONTAINER_NOLOCK_FCNS(sched_queue, inline, sched_queue_nolock,
                          isempty, pushback, pop, head, remove);

# define SCHED_QUEUE_INITIALIZER GCT_CONTAINER_ROOT_INITIALIZER(sched_queue)

/** @internal */
extern CONTEXT_LOCAL struct sched_context_s *sched_cur;
/** @internal */
extern CPU_LOCAL struct sched_context_s sched_idle;

#else
typedef struct __empty_s sched_queue_root_t;
#endif

/** @This returns the current scheduler context */
config_depend_alwaysinline(CONFIG_MUTEK_CONTEXT_SCHED,
struct sched_context_s *sched_get_current(void),
{
  return CONTEXT_LOCAL_GET(sched_cur);
});

/** @This returns a cpu local context for temporary stack use with
    @ref cpu_context_stack_use, useful to run other context
    exit/destroy. The processor idle context is actually returned. This
    context must be used with interrupts disabled. */
config_depend_alwaysinline(CONFIG_MUTEK_CONTEXT_SCHED,
struct context_s * sched_tmp_context(void),
{
  return CPU_LOCAL_ADDR(sched_idle)->context;
});

/** @This is a context preemption handler.

    @This pushes current context back on running queue and returns
    next scheduler candidate. If no other context is available on
    running queue, this function does nothing and return @tt NULL.

    @see context_set_preempt @see #CONTEXT_PREEMPT @see sched_context_switch */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
CONTEXT_PREEMPT(sched_preempt_switch);

/** @This is a context preemption handler.

    @This returns next scheduler candidate or processor idle context
    if none is available. Current context is not pushed back on
    running queue.

    @see context_set_preempt @see #CONTEXT_PREEMPT @see sched_context_stop */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
CONTEXT_PREEMPT(sched_preempt_stop);

/** @This is a context preemption handler.

    @This pushes current context on the wait queue specified in @ref
    sched_preempt_wait_unlock_q. The function unlocks the queue and
    returns the next candidate context which may be the processor idle
    context.

    @see context_set_preempt @see #CONTEXT_PREEMPT @see sched_wait_unlock */
config_depend_and2(CONFIG_MUTEK_CONTEXT_SCHED, CONFIG_HEXO_CONTEXT_PREEMPT)
CONTEXT_PREEMPT(sched_preempt_wait_unlock);

/** @see sched_preempt_wait_unlock */
config_depend_and2(CONFIG_MUTEK_CONTEXT_SCHED, CONFIG_HEXO_CONTEXT_PREEMPT)
extern CPU_LOCAL sched_queue_root_t *sched_preempt_wait_unlock_q;

/** @This initializes scheduler context. */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_context_init(struct sched_context_s *sched_ctx,
                        struct context_s *context);

/** @This switches to next context. Must be called with interrupts
    disabled. @see sched_preempt_switch */
config_depend_alwaysinline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_context_switch(void),
{
  struct context_s *next = sched_preempt_switch();

  if (next)
    context_switch_to(next);
});

/** @This jumps to next context without saving current context. current
    context will be lost. Must be called with interrupts disabled and
    main sched queue locked. @see sched_preempt_stop */
config_depend_alwaysinline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_context_exit(void),
{
  context_jump_to(sched_preempt_stop());
});

/** @internal */
struct context_s *
sched_wait_unlock_ctx(sched_queue_root_t *queue);

/** @This pushes current context in the 'queue', unlock it and switch
   to next context available in the 'root' queue. Must be called with
   interrupts disabled. @see sched_preempt_wait_unlock */
config_depend_alwaysinline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_wait_unlock(sched_queue_root_t *queue),
{
  context_switch_to(sched_wait_unlock_ctx(queue));
});

/** @This enqueues scheduler context for execution. Must be called
    with interrupts disabled */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_context_start(struct sched_context_s *sched_ctx);

/** @This switches to next context without pushing current context
    back in running queue and unlock passed scheduler queue. @see
    sched_context_stop */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_stop_unlock(lock_t *lock);

/** @This locks context queue for writting. */
config_depend_inline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_queue_wrlock(sched_queue_root_t *queue),);

/** @This locks context queue for reading. */
config_depend_inline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_queue_rdlock(sched_queue_root_t *queue),);

/** @This unlocks context queue. */
config_depend_inline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_queue_unlock(sched_queue_root_t *queue),);

/** @This initializes context queue. */
config_depend_inline(CONFIG_MUTEK_CONTEXT_SCHED,
error_t sched_queue_init(sched_queue_root_t *queue),);

/** @This frees resources associated with context queue. */
config_depend_inline(CONFIG_MUTEK_CONTEXT_SCHED,
void sched_queue_destroy(sched_queue_root_t *queue),);

typedef void (sched_wait_cb_t)(void *ctx);

/** @This removes first context from passed queue and push it back in
    running queue.  @This returns a pointer to context or NULL if
    queue was empty. @This Must be called with interrupts disabled and
    queue locked. */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
struct sched_context_s *sched_wake(sched_queue_root_t *queue);

/** @This function removes a given context from passed queue and push it back in
    running queue. @This Must be called with interrupts disabled and
    queue locked. */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_context_wake(sched_queue_root_t *queue, struct sched_context_s *sched_ctx);

/** @This function set processor affinity sothat scheduler context
    will run on this cpu */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** @This function set processor affinity sothat scheduler context
    will not run on this cpu */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** @This function set processor affinity sothat scheduler context
    will run on a single cpu */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu);

/** @This function set processor affinity sothat scheduler context
    will run on all cpu */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_affinity_all(struct sched_context_s *sched_ctx);

/** @This function set processor affinity sothat scheduler context
    will run on all cpu */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED)
void sched_affinity_clear(struct sched_context_s *sched_ctx);

/** @This function setups a scheduler context candidate checking
    function. */
config_depend(CONFIG_MUTEK_CONTEXT_SCHED_CANDIDATE_FCN)
void sched_context_candidate_fcn(struct sched_context_s *sched_ctx, sched_candidate_fcn_t *fcn);

#endif

