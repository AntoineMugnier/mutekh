
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/local.h>
#include <hexo/segment.h>
#include <hexo/interrupt.h>

/** pointer to current context */
CONTEXT_LOCAL struct context_s *context_cur;

/** syscall handler for current context */
CONTEXT_LOCAL cpu_syscall_handler_t  *cpu_syscall_handler;

/** init a context object using current execution context */
error_t
context_bootstrap(struct context_s *context)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return -ENOMEM;

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  /* FIXME ? initial stack space will never be freed */
  context->stack = NULL;

  /* setup cpu specific context data */
  if ((res = cpu_context_bootstrap(context)))
    {
      arch_contextdata_free(context->tls);
      return res;
    }

  return 0;
}

/** init a context object allocating a new context */
error_t
context_init(struct context_s *context,
	     reg_t *stack_buf, size_t stack_size,
	     context_entry_t *entry, void *param)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return ENOMEM;

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

    /* allocate context stack memory */
  context->stack = stack_buf;

  /* initial stack pointer address */
  context->stack_ptr = context->stack + stack_size - 1;

  /* setup cpu specific context data */
  if ((res = cpu_context_init(context, entry, param)))
    {
      arch_contextdata_free(context->tls);
      return res;
    }

#ifdef CONFIG_HEXO_VMEM
  context->vmem = vmem_get_kernel_context();
#endif

  return 0;
}

/** free ressource associated with a context */
reg_t *
context_destroy(struct context_s *context)
{
  cpu_context_destroy(context);
  arch_contextdata_free(context->tls);

  return context->stack;
}

void
cpu_syscall_sethandler_ctx(struct context_s *context,
			   cpu_syscall_handler_t *hndl)
{
  CONTEXT_LOCAL_TLS_SET(context->tls, cpu_syscall_handler, hndl);
}

