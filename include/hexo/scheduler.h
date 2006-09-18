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

#include "context.h"

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_dlist.h>

#define __SCHED_CONTAINER_ALGO		DLIST

CONTAINER_TYPE(sched_queue, __SCHED_CONTAINER_ALGO, struct sched_context_s, HEXO_SPIN_IRQ);

struct sched_context_s
{
  struct context_s	context;
  sched_queue_entry_t	list_entry;
  void			*private;
};

CONTAINER_FUNC(static inline, sched_queue, __SCHED_CONTAINER_ALGO, sched_queue, HEXO_SPIN, list_entry);
CONTAINER_FUNC(static inline, sched_queue, __SCHED_CONTAINER_ALGO, sched_queue_nolock, NOLOCK, list_entry);

/** lock main scheduler queue. Must be used before context exit */
void sched_lock(void);

/** release main scheduler queue. Must be used after thread entry */
void sched_unlock(void);

/** initialize scheduler context. context_init(&sched_ctx->context)
    must be called before */
void sched_context_init(struct sched_context_s *sched_ctx);

/** switch to next context */
/* Must be called with interrupts disabled */
void sched_context_switch(void);

/** switch to next context without saving current context. current
    context will be lost. Must be called with interrupts disabled and
    main sched queue locked */
void sched_context_exit(void);

/** enqueue scheduler context for execution. Must be called with
    interrupts disabled */
void sched_context_start(struct sched_context_s *sched_ctx);

/** switch to next context without pushing current context back. Must
    be called with interrupts disabled */
void sched_context_stop(void);

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

#endif

