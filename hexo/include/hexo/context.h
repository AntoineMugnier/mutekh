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

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include <hexo/types.h>
#include <hexo/local.h>
#include <hexo/error.h>
#include <hexo/vmem.h>

/** cpu specific context structure */
struct cpu_context_s;

/** context descriptor structure */
struct context_s
{
  /* cpu specific context pointer if any */
  struct cpu_context_s	*ccontext;

  /* context local storage address */
  void			*tls;

  /* stack memory address */
  reg_t			*stack;

  /* current stack pointer value */
  reg_t			*stack_ptr;

#ifdef CONFIG_HEXO_VMEM
  struct vmem_context_s	*vmem;
#endif
};

/** context entry point function prototype */
#define CONTEXT_ENTRY(n) void (n) (void *param)
/** context entry point function type */
typedef CONTEXT_ENTRY(context_entry_t);

/** Switch context by saving/restoring all registers from/to context stack */
static void cpu_context_switch(struct context_s *old, struct context_s *new);

/** Jump to context from _non_ context */
static void cpu_context_jumpto(struct context_s *new);

/** set new stack pointer and jump to a new function */
static void cpu_context_set(uintptr_t stack, void *jumpto);

/** associate context and cpu current execution state */
error_t cpu_context_bootstrap(struct context_s *context);

/** Prepare context execution by setting up original stack values */
error_t cpu_context_init(struct context_s *context, context_entry_t *entry, void *param);

/** cleanup context */
void cpu_context_destroy(struct context_s *context);


#if defined(CONFIG_CPU_USER)

/** user entry point function prototype */
#define USER_ENTRY(n) void (n) (void *param)
/** user entry point function type */
typedef USER_ENTRY(user_entry_t);

/** set kernel stack, user stack pointer and jump to a new function in user mode */
void cpu_context_set_user(uintptr_t kstack, uintptr_t ustack,
			  user_entry_t *entry, void *param);

#endif


#include "cpu/hexo/context.h"

/** pointer to current context object */
extern CONTEXT_LOCAL struct context_s *context_cur;

/** init a context object using current execution context */
error_t context_bootstrap(struct context_s *context);

/** init a context object allocating a new context */
error_t context_init(struct context_s *context,
		     reg_t *stack_buf, size_t stack_size,
		     context_entry_t *entry, void *param);

/** free ressource associated with a context and return a pointer to
    context stack buffer  */
reg_t * context_destroy(struct context_s *context);

/** switch to a given context */
static inline void context_switch_to(struct context_s *context)
{
  struct context_s *cur = CONTEXT_LOCAL_GET(context_cur);

#if 0
  if (cur == context)
    cpu_trap();
#endif

#ifdef CONFIG_HEXO_VMEM
  vmem_context_switch_to(context->vmem);
#endif

  cpu_context_switch(cur, context);
}

/** jump to a given context without saving current context */
static inline void
__attribute__((noreturn))
context_jump_to(struct context_s *context)
{
#ifdef CONFIG_HEXO_VMEM
  vmem_context_switch_to(context->vmem);
#endif
  cpu_context_jumpto(context);
}

/** return current context object */
static inline struct context_s * context_current(void)
{
  return CONTEXT_LOCAL_GET(context_cur);
}

#endif

