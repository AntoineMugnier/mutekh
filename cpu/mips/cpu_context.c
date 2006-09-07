
#include <hexo/error.h>
#include <hexo/context.h>

error_t
cpu_context_bootstrap(struct context_s *context)
{
  /* set context local storage base pointer */
  CPU_LOCAL_SET(__cpu_context_data_base, context->tls);
  CONTEXT_LOCAL_SET(__context_data_base, context->tls);

  return 0;
}



/* fake context entry point, pop entry funciton param and entry function
   address from stack and perform jump to real entry function.  We
   need to do this to pass an argument to the context entry function. */
void __mips_context_entry(void);

asm(
    ".set push			\n"
    ".set noreorder		\n"
    ".set noat			\n"
    "__mips_context_entry:		\n"
    "	lw	$4,	0($sp)	\n" /* entry function param */
    "	lw	$1,	4($sp)	\n" /* entry function address */
    "	jr	$1		\n"
    "	addiu	$sp,	2*4	\n"
    ".set pop			\n"
    );



/* context init function */

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
  CONTEXT_LOCAL_FOREIGN_SET(context->tls, __context_data_base, context->tls);

  /* push entry function address and param arg */
  *--context->stack_ptr = (uintptr_t)entry;
  *--context->stack_ptr = (uintptr_t)param;

  /* fake entry point */
  *--context->stack_ptr = (uintptr_t)&__mips_context_entry;

  /* status register */
  *--context->stack_ptr = 0x0000ff00;

  /* context local storage address */
  *--context->stack_ptr = (uintptr_t)context->tls;

  return 0;
}



void
cpu_context_destroy(struct context_s *context)
{
#if 0
  reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

