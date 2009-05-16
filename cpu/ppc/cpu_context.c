
#include <hexo/error.h>
#include <hexo/context.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

error_t
cpu_context_bootstrap(struct context_s *context)
{
  /* set context local storage register base pointer */
  asm volatile("mtspr 0x114, %0" : : "r" (context->tls)); /* SPRG4 is tls */

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_change_id(cpu_id(), (uint32_t)&context->stack_ptr);
#endif

  return 0;
}



/* fake context entry point, pop entry function param and entry function
   address from stack and perform jump to real entry function.  We
   need to do this to pass an argument to the context entry function. */
void __ppc_context_entry(void);

asm(
    "__ppc_context_entry:		\n"
    "	lwz	3, 0(1)	\n" /* entry function param */
    "	lwz	0, 4(1)	\n" /* entry function address */
    "	addi	1, 1, 2*4		\n"
    "	mtctr	0			\n"
    "	bctrl				\n"
    );



/* context init function */

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_create_ctx((uint32_t)&context->stack_ptr,
			      context->stack_start, context->stack_end);
#endif

  context->stack_ptr = (reg_t*)context->stack_end - 1;

  /* push entry function address and param arg */
  *--context->stack_ptr = (uintptr_t)entry;
  *--context->stack_ptr = (uintptr_t)param;

  /* fake entry point */
  *--context->stack_ptr = (uintptr_t)&__ppc_context_entry;

  /* r14, r15, r30, r31 */
  context->stack_ptr -= 2;
  *--context->stack_ptr = 0;
  *--context->stack_ptr = 0;

  /* msr, interrupts are disabled */
  *--context->stack_ptr = 0;

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

