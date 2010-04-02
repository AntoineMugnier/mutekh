
#include <hexo/error.h>
#include <hexo/context.h>

CONTEXT_LOCAL struct context_regs_s ppc_context_regs;

#ifdef CONFIG_HEXO_LAZY_SWITCH
/* last fpu restored context */
CPU_LOCAL struct context_regs_s *ppc_lazy_last = 0;
#endif

error_t
cpu_context_bootstrap(struct context_s *context)
{
  /* set context local storage register base pointer */
  asm volatile("mtspr 0x114, %0" : : "r" (context->tls)); /* SPRG4 is tls */

  /* nothing is saved for this context */
  CONTEXT_LOCAL_ADDR(ppc_context_regs)->save_mask = 0;

  return 0;
}


/* context init function */

#if CONFIG_HEXO_STACK_ALIGN < 16
# error PowerPc ABI requires 16 bytes alignment
#endif

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
  struct context_regs_s *regs = CONTEXT_LOCAL_TLS_ADDR(context->tls, ppc_context_regs);

  regs->save_mask = CPU_PPC_CONTEXT_RESTORE_CALLER; /* for r3 */
  regs->gpr[1] = CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_end)
               - CONFIG_HEXO_STACK_ALIGN;
  regs->gpr[3] = (uintptr_t)param;
  regs->cr = 0;

  /* msr, interrupts are disabled */
#if defined (CONFIG_HEXO_FPU) && !defined(CONFIG_HEXO_LAZY_SWITCH)
  regs->msr = PPC_MSR_FPU_ENABLED;
  reeg->fpscr = 0;
#else
  regs->msr = 0;
#endif

  regs->lr = 0xa5a5a5a5;        /* can not return from context entry */
  regs->pc = (uintptr_t)entry;

  return 0;
}

void
cpu_context_destroy(struct context_s *context)
{
#if 0
  reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

