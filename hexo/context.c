
#include <hexo/cpu.h>
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/local.h>
#include <hexo/interrupt.h>
#include <mutek/startup.h>

#include <mutek/mem_alloc.h>

#include <string.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

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

# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
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

# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
  context->unlock = NULL;
# endif

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_create_ctx((uint32_t)context, stack_start, stack_end);
#endif

  /* setup cpu specific context data */
  if ((res = cpu_context_init(context, entry, param)))
    {
#ifdef CONFIG_SOCLIB_MEMCHECK
      soclib_mem_check_delete_ctx((uint32_t)context);
#endif
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

# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
  assert(context->unlock == NULL);
# endif

  cpu_context_destroy(context);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_delete_ctx((uint32_t)context);
#endif

  mem_free(context->tls);

  return stack;
}

void hexo_context_initsmp(void)
{
  struct context_s *context = CPU_LOCAL_ADDR(cpu_main_context);

#if defined(CONFIG_DEVICE_CPU)
  const struct cpu_tree_s *cpu = cpu_tree_lookup(cpu_id());
  assert(cpu != NULL && "processor id not found in the cpu tree.");

  ensure(context_bootstrap(context, cpu->stack, CONFIG_HEXO_CPU_STACK_SIZE) == 0);
# ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_change_id(cpu->stack, (uint32_t)context);
# endif

#else
  ensure(context_bootstrap(context, CONFIG_STARTUP_STACK_ADDR, CONFIG_STARTUP_STACK_SIZE) == 0);
#endif

  /* enable interrupts from now */
  cpu_interrupt_enable();
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


