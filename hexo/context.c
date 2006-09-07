
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/local.h>
#include <hexo/segment.h>

/** pointer to current context */
CONTEXT_LOCAL struct context_s *context_cur;

/** init a context object using current execution context */
error_t
context_bootstrap(struct context_s *context)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return -ENOMEM;

  CONTEXT_LOCAL_FOREIGN_SET(context->tls, context_cur, context);

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
context_init(struct context_s *context, size_t stack_size, context_entry_t *entry, void *param)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return -ENOMEM;

  CONTEXT_LOCAL_FOREIGN_SET(context->tls, context_cur, context);

  /* allocate context stack memory */
  if (!(context->stack = arch_contextstack_alloc(stack_size * sizeof(reg_t))))
    {
      arch_contextdata_free(context->tls);
      return -ENOMEM;      
    }

  /* initial stack pointer address */
  context->stack_ptr = context->stack + stack_size - 1;

  /* setup cpu specific context data */
  if ((res = cpu_context_init(context, entry, param)))
    {
      arch_contextdata_free(context->tls);
      arch_contextstack_free(context->stack);
      return res;
    }

  return 0;
}

/** free ressource associated with a context */
void
context_destroy(struct context_s *context)
{
  cpu_context_destroy(context);
  arch_contextdata_free(context->tls);

  if (context->stack)
    arch_contextstack_free(context->stack);
}

