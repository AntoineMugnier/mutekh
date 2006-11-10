
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

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
  CONTEXT_LOCAL_FOREIGN_SET(context->tls, __context_data_base, context->tls);

  /* push context entry function return pointer */
  *--context->stack_ptr = (uintptr_t)0;

  /* push execution pointer */
  *--context->stack_ptr = (uintptr_t)entry;	/* EIP */

  /* push default flags */
#if defined(CONFIG_DEBUG)
  *--context->stack_ptr = 0x00040246;	/* EFLAGS with alignment chk */
#else
  *--context->stack_ptr = 0x00000246;	/* EFLAGS */
#endif

  /* room for general purpose registers default values */
  context->stack_ptr -= 15;

#if 0
  context->stack_ptr[14] = 0;	/* rax */
  context->stack_ptr[13] = 0;	/* rbx */
  context->stack_ptr[12] = 0;	/* rcx */
  context->stack_ptr[11] = 0;	/* rdx */
  context->stack_ptr[10] = 0;	/* rsi */
#endif
  context->stack_ptr[9] = (reg_t)param; /* rdi */
#ifdef CONFIG_COMPILE_DEBUG
  /* frame pointer initial value set to zero for debugger */
  context->stack_ptr[8] = 0;	/* rbp */
#endif
#if 0
  context->stack_ptr[7] = 0;	/* r8 */
  context->stack_ptr[6] = 0;	/* r9 */
  context->stack_ptr[5] = 0;	/* r10 */
  context->stack_ptr[4] = 0;	/* r11 */
  context->stack_ptr[3] = 0;	/* r12 */
  context->stack_ptr[2] = 0;	/* r13 */
  context->stack_ptr[1] = 0;	/* r14 */
  context->stack_ptr[0] = 0;	/* r15 */
#endif

  /* push tls address */
  *--context->stack_ptr = (reg_t)context->tls;

  return 0;
}

void
cpu_context_destroy(struct context_s *context)
{
}

