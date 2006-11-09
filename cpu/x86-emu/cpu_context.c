
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

  /* push param */
  *--context->stack_ptr = (uintptr_t)param;

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
  context->stack_ptr -= 8;
#if 0
  context->stack_ptr[0] = 0; 	/* eax */
  context->stack_ptr[-1] = 0; 	/* ecx */
  context->stack_ptr[-2] = 0; 	/* edx */
  context->stack_ptr[-3] = 0; 	/* ebx */
  context->stack_ptr[-4] = 0; 	/* esp (skiped by popa) */
#endif
#if defined (CONFIG_COMPILE_DEBUG)
  /* frame pointer initial value set to zero for debugger */
  context->stack_ptr[-5] = 0; 	/* ebp */
#endif
#if 0
  context->stack_ptr[-6] = 0; 	/* esi */
  context->stack_ptr[-7] = 0; 	/* edi */
#endif

  /* push tls address */
  *--context->stack_ptr = (reg_t)context->tls;

  return 0;
}

void
cpu_context_destroy(struct context_s *context)
{
}

