
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/local.h>
#include <hexo/segment.h>
#include <hexo/interrupt.h>

/** pointer to current context */
CONTEXT_LOCAL struct context_s *context_cur = NULL;

/** init a context object using current execution context */
error_t
context_bootstrap(struct context_s *context)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return -ENOMEM;

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  /* initial stack space will never be freed ! */
  context->stack_end = context->stack_start = NULL;

  /* setup cpu specific context data */
  if ((res = cpu_context_bootstrap(context)))
    {
      arch_contextdata_free(context->tls);
      return res;
    }

#ifdef CONFIG_HEXO_MMU
  context->mmu = mmu_get_kernel_context();
#endif

  return 0;
}

/** init a context object allocating a new context */
error_t
context_init(struct context_s *context,
	     void *stack_start, void *stack_end,
	     context_entry_t *entry, void *param)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return ENOMEM;

  assert((uintptr_t)context->tls % sizeof(reg_t) == 0);

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  assert(stack_end > stack_start);
  assert((uintptr_t)stack_end % sizeof(reg_t) == 0);

  context->stack_start = stack_start;
  context->stack_end = stack_end;

  /* setup cpu specific context data */
  if ((res = cpu_context_init(context, entry, param)))
    {
      arch_contextdata_free(context->tls);
      return res;
    }

#ifdef CONFIG_HEXO_MMU
  context->mmu = mmu_get_kernel_context();
#endif

  return 0;
}

/** free ressource associated with a context */
reg_t *
context_destroy(struct context_s *context)
{
  cpu_context_destroy(context);
  arch_contextdata_free(context->tls);

  return context->stack_start;
}

