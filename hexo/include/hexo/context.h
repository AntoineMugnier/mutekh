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

/**
   @file
   @module{Hexo}
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

/** @internal offset of tls field in @ref context_s */
#define HEXO_CONTEXT_S_TLS              0
/** @internal offset of unlock field in @ref context_s */
#define HEXO_CONTEXT_S_UNLOCK           1

#ifndef __MUTEK_ASM__

# include <hexo/types.h>
# include <hexo/local.h>
# include <hexo/error.h>
# include <hexo/mmu.h>
# include <hexo/lock.h>
# include <assert.h> 

/** @internal context descriptor structure */
struct context_s
{
  /** context local storage address. @csee #HEXO_CONTEXT_S_TLS */
  void			*tls;

# ifdef CONFIG_ARCH_SMP
  /** pointer to atomic int to clear on context switch */
  atomic_int_t          *unlock;
# endif

# ifdef CONFIG_HEXO_MMU
  struct mmu_context_s	*mmu;
# endif

# ifdef CONFIG_HEXO_CONTEXT_STATS
  /** number of cpu cycles spent executing this context */
  __attribute__((aligned(8)))
  cpu_cycle_t cycles;
# endif
};

/** @internal */
extern CONTEXT_LOCAL uintptr_t context_stack_start;
/** @internal */
extern CONTEXT_LOCAL uintptr_t context_stack_end;

/** @showvalue @this is the context entry point function prototype */
#define CONTEXT_ENTRY(n) void (n) (void *param)
/** @this is context entry point function type. @csee #CONTEXT_ENTRY */
typedef CONTEXT_ENTRY(context_entry_t);


/** @showvalue context preempt function prototype.

    This function type is used on interrupt handler completion and may
    return a pointer to a context to switch to or NULL. When
    returning a context pointer, the interrupted context will not be
    resumed. @see context_set_preempt.

    The interrupted context will be completely saved before calling
    this function so that it may be immediately resumed on an other
    processor before the interrupt handler even returns.
 */
#define CONTEXT_PREEMPT(n) struct context_s * (n)(void *param)
/** context preempt function prototype. @csee #CONTEXT_PREEMPT */
typedef CONTEXT_PREEMPT(context_preempt_t);


#endif  /* __MUTEK_ASM__ */
#include "cpu/hexo/context.h"
#ifndef __MUTEK_ASM__

/** @internal @this only performs processor specific part of the job. @csee context_switch_to */
void cpu_context_switch(struct context_s *new);

/** @internal @this only performs processor specific part of the job. @csee context_jump_to */
__attribute__((noreturn))
void cpu_context_jumpto(struct context_s *new);

/** @this sets new stack pointer and jump to a new function. */
__attribute__((noreturn))
void cpu_context_set(uintptr_t stack, size_t stack_size, void *jumpto);

/** @internal @this executes a function using given context stack.
    Current context stack content is preserved.
    @this only performs processor specific part of the job. @csee context_stack_use */
__attribute__((noreturn))
void cpu_context_stack_use(struct context_s *context,
                           context_entry_t *func, void *param);

/** @internal @this intializes given context to match cpu current execution state.
    @this only performs processor specific part of the job. @csee context_bootstrap */
error_t cpu_context_bootstrap(struct context_s *context);

/** @internal @this initializes context and prepares first context execution.
    @this only performs processor specific part of the job. @csee context_init */
error_t cpu_context_init(struct context_s *context, context_entry_t *entry, void *param);

/** @internal @this cleanups given context resources.
    @this only performs processor specific part of the job. @csee context_init */
void cpu_context_destroy(struct context_s *context);

# if defined(CONFIG_HEXO_USERMODE)
/** @this sets user stack pointer and jump to a new function in user mode. */
__attribute__((noreturn))
void cpu_context_set_user(uintptr_t stack_ptr, uintptr_t entry, reg_t param);
# endif

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
/** @internal */
extern CPU_LOCAL context_preempt_t *cpu_preempt_handler;
/** @internal */
extern CPU_LOCAL void *cpu_preempt_param;

/** @this sets a preemption handler function for the processor. The
    preemtion handler is reset to NULL on each exception and irq and
    must be setup by the interrupt handler when context preemption is needed.
    @see #CONTEXT_PREEMPT */
static inline void context_set_preempt(context_preempt_t *func, void *param)
{
  CPU_LOCAL_SET(cpu_preempt_handler, func);
  CPU_LOCAL_SET(cpu_preempt_param, param);
}
# endif

/** @this sets address of a lock which must be unlocked on next context
    restoration. The lock address is reset to NULL once unlock has been performed. */
static inline void context_set_unlock(struct context_s *context, lock_t *lock)
{
# ifdef CONFIG_ARCH_SMP
  // FIXME use arch code to get atomic value address from lock
  context->unlock = (void*)lock;
# endif
}

/** @internal */
extern CONTEXT_LOCAL struct context_s *context_cur;

/** @this executes the given function using given existing context
    stack while the context is not actually running. */
__attribute__((noreturn))
static inline void context_stack_use(struct context_s *context,
                                     context_entry_t *func, void *param)
{
  cpu_context_stack_use(context, func, param);
}

/** init a context object using current execution context */
error_t context_bootstrap(struct context_s *context, uintptr_t stack, size_t stack_size);

/** init a context object allocating a new context */
error_t context_init(struct context_s *context,
		     void *stack_start, void *stack_end,
		     context_entry_t *entry, void *param);

/** free ressource associated with a context and return a pointer to
    context stack buffer  */
void * context_destroy(struct context_s *context);



#ifdef CONFIG_HEXO_CONTEXT_STATS
/** @internal timestamp of last context switch on this processor */
extern CPU_LOCAL cpu_cycle_t context_swicth_time;

/** @internal @this updates context stats when leaving current context on switch. */
static inline void context_leave_stats(struct context_s *context)
{
  if (context)
    context->cycles += cpu_cycle_diff(CPU_LOCAL_GET(context_swicth_time));
}

/** @internal @this updates context stats when entering current context on switch. */
static inline void context_enter_stats(struct context_s *context)
{
  CPU_LOCAL_SET(context_swicth_time, cpu_cycle_count());
}
#endif



/** @this saves current context and restore given context. */
static inline void context_switch_to(struct context_s *context)
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
}

/** @this restores given context without saving current context. */
static inline void
__attribute__((noreturn))
context_jump_to(struct context_s *context)
{
#if 0 //def CONFIG_HEXO_CONTEXT_STATS
  struct context_s *cur = CONTEXT_LOCAL_GET(context_cur);

  context_leave_stats(cur);
#endif

#ifdef CONFIG_HEXO_MMU
  mmu_context_switch_to(context->mmu);
#endif
  cpu_context_jumpto(context);
}

/** @this returns a pointer to current context */
static inline struct context_s * context_current(void)
{
  return CONTEXT_LOCAL_GET(context_cur);
}

#endif  /* __MUTEK_ASM__ */

#endif

