
#include <hexo/cpu.h>
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/local.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>

#include <string.h>

#include <mutek/instrumentation.h>

CPU_LOCAL struct context_s cpu_main_context;

CONTEXT_LOCAL uintptr_t context_stack_start;
CONTEXT_LOCAL uintptr_t context_stack_end;

/** pointer to current context */
CONTEXT_LOCAL struct context_s *context_cur = NULL;

static void * context_data_alloc(void)
{
  void			*tls;
  extern __ldscript_symbol_t __context_data_start, __context_data_end;

  /* allocate memory and copy from template */
  if ((tls = mem_alloc((char*)&__context_data_end - (char*)&__context_data_start, (mem_scope_sys))))
    {
      memcpy(tls, (char*)&__context_data_start, (char*)&__context_data_end - (char*)&__context_data_start);
    }

  return tls;
}

/** init a context object using current execution context */
error_t
context_bootstrap(struct context_s *context, uintptr_t stack, size_t stack_size)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = context_data_alloc()))
    return -ENOMEM;

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_start, stack);
  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_end, stack + stack_size);

  /* setup cpu specific context data */
  if ((res = cpu_context_bootstrap(context)))
    {
      mem_free(context->tls);
      return res;
    }

# ifdef CONFIG_ARCH_SMP
  context->unlock = NULL;
# endif

#ifdef CONFIG_HEXO_MMU
  context->mmu = mmu_get_kernel_context();
#endif

#ifdef CONFIG_HEXO_CONTEXT_STATS
  context_enter_stats(context);
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
  if (!(context->tls = context_data_alloc()))
    return ENOMEM;

  assert((uintptr_t)context->tls % sizeof(reg_t) == 0);

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  assert(stack_end > stack_start);
  assert((uintptr_t)stack_end % CONFIG_HEXO_STACK_ALIGN == 0);

  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_start, (uintptr_t)stack_start);
  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_end, (uintptr_t)stack_end);

# ifdef CONFIG_ARCH_SMP
  context->unlock = NULL;
# endif

  instrumentation_context_create((uint32_t)context, (uintptr_t)stack_start,
                                 (uintptr_t)stack_end - (uintptr_t)stack_start);

  /* setup cpu specific context data */
  if ((res = cpu_context_init(context, entry, param)))
    {
      instrumentation_context_destroy((uint32_t)context);
      mem_free(context->tls);
      return res;
    }

#ifdef CONFIG_HEXO_MMU
  context->mmu = mmu_get_kernel_context();
#endif

#ifdef CONFIG_HEXO_CONTEXT_STATS
  context->enter_cnt = 0;
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
  context->preempt_cnt = 0;
# endif
#endif

  return 0;
}

/** free ressource associated with a context */
void *
context_destroy(struct context_s *context)
{
  void *stack = (void*)CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_start);

# ifdef CONFIG_ARCH_SMP
  assert(context->unlock == NULL);
# endif

  cpu_context_destroy(context);

  instrumentation_context_destroy((uint32_t)context);

  mem_free(context->tls);

  return stack;
}

#ifdef CONFIG_HEXO_CONTEXT_STATS

void context_leave_stats(struct context_s *context)
{
}

void context_enter_stats(struct context_s *context)
{
  context->enter_cnt++;
}

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
void context_preempt_stats(struct context_s *next)
{
  struct context_s *prev = CONTEXT_LOCAL_GET(context_cur);

  context_leave_stats(prev);
  prev->preempt_cnt++;
  context_enter_stats(next);
}
# endif

#endif


