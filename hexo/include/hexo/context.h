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

/**
   @file
   @module {Core::Hardware abstraction layer}
   @short Execution context management stuff

   The @ref context_s data structure is used to store execution state
   of a processor. Hexo provides some services associated with
   execution contexts which allow managing processor states but does
   not handle context scheduling directly.

   Available execution context operations are mainly creation and
   switch related.
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

# include <hexo/types.h>
# include <hexo/local.h>
# include <hexo/error.h>
# include <hexo/mmu.h>
# include <hexo/lock.h>
# include <assert.h> 

/** @internal context descriptor structure */
struct context_s
{
  /** context local storage address */
  void			*tls;

# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
  /** lock to release on context restore */
  lock_t                *unlock;
# endif

# ifdef CONFIG_HEXO_MMU
  struct mmu_context_s	*mmu;
# endif

# ifdef CONFIG_HEXO_CONTEXT_STATS
  size_t      enter_cnt;
#  ifdef CONFIG_HEXO_CONTEXT_PREEMPT
  size_t      preempt_cnt;
#  endif
# endif
};

/** @internal */
config_depend(CONFIG_HEXO_CONTEXT)
extern CONTEXT_LOCAL uintptr_t context_stack_start;

/** @internal */
config_depend(CONFIG_HEXO_CONTEXT)
extern CONTEXT_LOCAL uintptr_t context_stack_end;

/** @showvalue @This is the context entry point function prototype */
#define CONTEXT_ENTRY(n) void (n) (void *param)
/** @This is context entry point function type. @csee #CONTEXT_ENTRY */
typedef CONTEXT_ENTRY(context_entry_t);


/** @showvalue context preempt function prototype.

    A function pointer of this type is called on exception handler
    completion if valid. The function may return a pointer to a
    context to switch to or @tt NULL. When this function returns a
    context pointer, the interrupted context will not be resumed.

    The @ref context_set_preempt function can be called from within an
    exception handler to setup such a one-shot handler for the executing
    processor.

    The MutekH scheduler provides some useful preemption handlers for
    common context queuing operations: See @ref sched_preempt_switch,
    @ref sched_preempt_stop and @ref sched_preempt_wait_unlock.
 */
#define CONTEXT_PREEMPT(n) struct context_s * (n)(void)
/** context preempt function prototype. @csee #CONTEXT_PREEMPT */
typedef CONTEXT_PREEMPT(context_preempt_t);

/** @showvalue irq enable restore function prototype.
    @see context_set_irqen */
#define CONTEXT_IRQEN(n) void (n)(void)
/** context irq restore enable function prototype. @csee #CONTEXT_IRQEN */
typedef CONTEXT_IRQEN(context_irqen_t);

#include "cpu/hexo/context.h"

/** @internal @This only performs processor specific part of the job. @csee context_switch_to */
config_depend(CONFIG_HEXO_CONTEXT)
void cpu_context_switch(struct context_s *new);

/** @internal @This only performs processor specific part of the job. @csee context_jump_to */
config_depend(CONFIG_HEXO_CONTEXT)
__attribute__((noreturn))
void cpu_context_jumpto(struct context_s *new);

/** @This sets new stack pointer and jump to a new function. */
__attribute__((noreturn))
void cpu_context_set(uintptr_t stack, size_t stack_size, void *jumpto);

/** @internal @This executes a function using given context stack.
    Current context stack content is preserved.
    @This only performs processor specific part of the job. @csee context_stack_use */
config_depend(CONFIG_HEXO_CONTEXT)
__attribute__((noreturn))
void cpu_context_stack_use(struct context_s *context,
                           context_entry_t *func, void *param);

/** @internal @This intializes given context to match cpu current execution state.
    @This only performs processor specific part of the job. @csee context_bootstrap */
config_depend(CONFIG_HEXO_CONTEXT)
error_t cpu_context_bootstrap(struct context_s *context);

/** @internal @This initializes context and prepares first context execution.
    @This only performs processor specific part of the job. @csee context_init */
config_depend(CONFIG_HEXO_CONTEXT)
error_t cpu_context_init(struct context_s *context, context_entry_t *entry, void *param);

/** @internal @This cleanups given context resources.
    @This only performs processor specific part of the job. @csee context_init */
config_depend(CONFIG_HEXO_CONTEXT)
void cpu_context_destroy(struct context_s *context);

/** @This sets user stack pointer and jump to a new function in user mode. */
config_depend_and2(CONFIG_HEXO_CONTEXT, CONFIG_HEXO_USERMODE)
__attribute__((noreturn))
void cpu_context_set_user(uintptr_t stack_ptr, uintptr_t entry, reg_t param);

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
/** @This sets a preemption handler function local to the executing
    processor. The preemtion handler pointer is reset to @tt NULL on
    each exception entry and can be setup from an interrupt handler when
    context preemption is needed on interrupt return.

    This function returns an error if a handler has already been set
    during the current interrupt or if it is not called from an
    interrupt handler. This function must be called with interrupts
    disabled.

    @see #CONTEXT_PREEMPT */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
error_t context_set_preempt(context_preempt_t *func), ;
);
# endif

# ifdef CONFIG_HEXO_CONTEXT_IRQEN
/** @This sets an irq re-enable handler function local to the
    executing processor. It is called when the interrupt enable state
    is restored by the @ref cpu_interrupt_restorestate function. The
    handler pointer is reset to @tt NULL on context switch. */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
void context_set_irqen(context_irqen_t *func), ;
# endif

/** @This sets address of a lock which must be unlocked on next context
    restoration. The lock address is reset to NULL once unlock has been performed. */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
void context_set_unlock(struct context_s *context, lock_t *unlock),
{
# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
  context->unlock = unlock;
# endif
});

/** @internal */
config_depend(CONFIG_HEXO_CONTEXT)
extern CONTEXT_LOCAL struct context_s *context_cur;

/** @internal */
config_depend(CONFIG_HEXO_CONTEXT)
extern CPU_LOCAL struct context_s cpu_main_context;

/** @This executes the given function using given existing context
    stack while the context is not actually running. @see sched_tmp_context */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
__attribute__((noreturn))
void context_stack_use(struct context_s *context,
                       context_entry_t *func, void *param),
{
  cpu_context_stack_use(context, func, param);
});

/** @This initializes a context object using current processor execution state. */
config_depend(CONFIG_HEXO_CONTEXT)
error_t context_bootstrap(struct context_s *context, uintptr_t stack, size_t stack_size);

/** @This initializes a context object allocating a new context */
config_depend(CONFIG_HEXO_CONTEXT)
error_t context_init(struct context_s *context,
		     void *stack_start, void *stack_end,
		     context_entry_t *entry, void *param);

/** @This frees ressource associated with a context and return a pointer to
    context stack buffer  */
config_depend(CONFIG_HEXO_CONTEXT)
void * context_destroy(struct context_s *context);


/** @internal @This updates context stats when current context is preempted */
config_depend(CONFIG_HEXO_CONTEXT_STATS)
void context_preempt_stats(struct context_s *context);

/** @internal @This updates context stats when leaving current context on switch. */
config_depend(CONFIG_HEXO_CONTEXT_STATS)
void context_leave_stats(struct context_s *context);

/** @internal @This updates context stats when entering current context on switch. */
config_depend(CONFIG_HEXO_CONTEXT_STATS)
void context_enter_stats(struct context_s *context);


/** @This saves current context and restore given context. */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
void context_switch_to(struct context_s *context),
{
#ifdef CONFIG_HEXO_CONTEXT_STATS
  struct context_s *cur = CONTEXT_LOCAL_GET(context_cur);

  assert(cur != context);

  context_leave_stats(cur);
#endif

#ifdef CONFIG_HEXO_MMU
  mmu_context_switch_to(context->mmu);
#endif

  cpu_context_switch(context);

#ifdef CONFIG_HEXO_CONTEXT_STATS
  context_enter_stats(cur);
#endif
});

/** @This restores given context without saving current context. */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
void __attribute__((noreturn))
context_jump_to(struct context_s *context),
{
#if 0 //def CONFIG_HEXO_CONTEXT_STATS
  struct context_s *cur = CONTEXT_LOCAL_GET(context_cur);

  context_leave_stats(cur);
#endif

#ifdef CONFIG_HEXO_MMU
  mmu_context_switch_to(context->mmu);
#endif
  cpu_context_jumpto(context);
});

/** @This returns a pointer to current context */
config_depend_alwaysinline(CONFIG_HEXO_CONTEXT,
struct context_s * context_current(void),
{
  return CONTEXT_LOCAL_GET(context_cur);
});

#endif

