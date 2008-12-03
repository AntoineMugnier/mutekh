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

#ifndef __HEXO_SCHEDULER_H__
#define __HEXO_SCHEDULER_H__

#ifndef CONFIG_HEXO_SCHED
# warning hexo scheduler is not enabled in configuration file
#else

#include "context.h"

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_dlist.h>

struct sched_context_s;

/** scheduler context candidate checking function */
#define SCHED_CANDIDATE_FCN(n) bool_t (n)(struct sched_context_s *sched_ctx)

/** scheduler context candidate checking function type */
typedef SCHED_CANDIDATE_FCN(sched_candidate_fcn_t);


#define CONTAINER_LOCK_sched_queue HEXO_SPIN

CONTAINER_TYPE(sched_queue, DLIST, struct sched_context_s
{
  struct context_s	context;
  sched_queue_entry_t	list_entry;
  void			*private;

#ifdef CONFIG_HEXO_SCHED_MIGRATION_AFFINITY
  cpu_bitmap_t		cpu_map;
#endif

#ifdef CONFIG_HEXO_SCHED_STATIC
  sched_queue_root_t	*cpu_queue;
#endif

#ifdef CONFIG_HEXO_SCHED_CANDIDATE_FCN
  sched_candidate_fcn_t	*is_candidate;
#endif
}, list_entry);

CONTAINER_FUNC       (sched_queue, DLIST, static inline, sched_queue, list_entry);
CONTAINER_FUNC_NOLOCK(sched_queue, DLIST, static inline, sched_queue_nolock, list_entry);

extern CONTEXT_LOCAL struct sched_context_s *sched_cur;

static inline struct sched_context_s *
sched_get_current(void)
{
  return CONTEXT_LOCAL_GET(sched_cur);
}

/** lock main scheduler queue. */
void sched_lock(void);

/** release main scheduler queue. Must be used after thread entry */
void sched_unlock(void);

/** initialize scheduler context. context_init(&sched_ctx->context)
    must be called before */
void sched_context_init(struct sched_context_s *sched_ctx);

/** switch to next context */
/* Must be called with interrupts disabled */
void sched_context_switch(void);

/** jump to next context without saving current context. current
    context will be lost. Must be called with interrupts disabled and
    main sched queue locked */
void sched_context_exit(void);

/** enqueue scheduler context for execution. Must be called with
    interrupts disabled */
void sched_context_start(struct sched_context_s *sched_ctx);

/** switch to next context without pushing current context back. Must
    be called with interrupts disabled */
void sched_context_stop(void);

/** switch to next context without pushing current context back. Must
    be called with interrupts disabled */
void sched_context_stop_unlock(lock_t *lock);

/** lock context queue */
error_t sched_queue_lock(sched_queue_root_t *queue);

/** unlock context queue */
void sched_queue_unlock(sched_queue_root_t *queue);

/** init context queue */
error_t sched_queue_init(sched_queue_root_t *queue);

/** destroy context queue */
void sched_queue_destroy(sched_queue_root_t *queue);

/** add current context on the wait queue, unlock queue and switch to
    next context */
/* Must be called with interrupts disabled */
void sched_wait_unlock(sched_queue_root_t *queue);

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

