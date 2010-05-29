
#include <hexo/cpu.h>
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/local.h>
#include <hexo/segment.h>
#include <hexo/interrupt.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
CPU_LOCAL context_preempt_t *cpu_preempt_handler;
CPU_LOCAL void *cpu_preempt_param;
#endif

#ifdef CONFIG_HEXO_CONTEXT_STATS
CPU_LOCAL cpu_cycle_t context_swicth_time;
#endif

CONTEXT_LOCAL uintptr_t context_stack_start;
CONTEXT_LOCAL uintptr_t context_stack_end;

/** pointer to current context */
CONTEXT_LOCAL struct context_s *context_cur = NULL;

extern __ldscript_symbol_t __initial_stack;

/** init a context object using current execution context */
error_t
context_bootstrap(struct context_s *context)
{
  error_t	res;

  /* allocate context local storage memory */
  if (!(context->tls = arch_contextdata_alloc()))
    return -ENOMEM;

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  /* FIXME initial stack space will never be freed ! */
  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_start, 0);
  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_end,
                        (uintptr_t)&__initial_stack - CONFIG_HEXO_RESET_STACK_SIZE * cpu_id());

  /* setup cpu specific context data */
  if ((res = cpu_context_bootstrap(context)))
    {
      arch_contextdata_free(context->tls);
      return res;
    }

#ifdef CONFIG_SOCLIB_MEMCHECK
    soclib_mem_check_change_id(cpu_id(), (uint32_t)context);
#endif


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
  if (!(context->tls = arch_contextdata_alloc()))
    return ENOMEM;

  assert((uintptr_t)context->tls % sizeof(reg_t) == 0);

  CONTEXT_LOCAL_TLS_SET(context->tls, context_cur, context);

  assert(stack_end > stack_start);
  assert((uintptr_t)stack_end % CONFIG_HEXO_STACK_ALIGN == 0);

  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_start, (uintptr_t)stack_start);
  CONTEXT_LOCAL_TLS_SET(context->tls, context_stack_end, (uintptr_t)stack_end);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_create_ctx((uint32_t)context, stack_start, stack_end);
#endif

  /* setup cpu specific context data */
  if ((res = cpu_context_init(context, entry, param)))
    {
#ifdef CONFIG_SOCLIB_MEMCHECK
      soclib_mem_check_delete_ctx((uint32_t)context);
#endif
      arch_contextdata_free(context->tls);
      return res;
    }

#ifdef CONFIG_HEXO_MMU
  context->mmu = mmu_get_kernel_context();
#endif

#ifdef CONFIG_HEXO_CONTEXT_STATS
  context->cycles = 0;
#endif

  return 0;
}

/** free ressource associated with a context */
void *
context_destroy(struct context_s *context)
{
  void *stack = (void*)CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_start);
  cpu_context_destroy(context);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_delete_ctx((uint32_t)context);
#endif

  arch_contextdata_free(context->tls);

  return stack;
}

