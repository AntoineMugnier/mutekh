
#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <mutek/scheduler.h>

#ifdef CONFIG_HEXO_CONTEXT

CONTEXT_LOCAL struct cpu_context_s arm_context_regs;

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
CPU_LOCAL context_preempt_t *cpu_preempt_handler = (context_preempt_t*)1;
#endif

#if CONFIG_CPU_ARM32_ARCH_VERSION < 6
CPU_LOCAL void *__context_data_base;
#endif

void arm_setup_exception_stack(uintptr_t addr);

error_t
cpu_context_bootstrap(struct context_s *context)
{
    /* set context local storage register base pointer */
#if CONFIG_CPU_ARM32_ARCH_VERSION >= 6

    THUMB_TMP_VAR;
    asm volatile (
        THUMB_TO_ARM
        "mcr p15,0,%0,c13,c0,4 \n\t"
        ARM_TO_THUMB
        /*:*/ THUMB_OUT(:)
        : "r" (context->tls));
#else
    __context_data_base = context->tls;
#endif

    arm_setup_exception_stack((uintptr_t)CONTEXT_LOCAL_ADDR(arm_context_regs));

    return 0;
}


/* context init function */

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
    struct cpu_context_s *regs = CONTEXT_LOCAL_TLS_ADDR(context->tls, arm_context_regs);

    regs->save_mask =
        CPU_ARM_CONTEXT_RESTORE_CALLER |
        CPU_ARM_CONTEXT_RESTORE_CALLEE;
    regs->gpr[13] =
        CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_end);
#ifdef CONFIG_COMPILE_FRAMEPTR
    regs->gpr[11] = regs->gpr[13];
#endif
    regs->gpr[0] = (uintptr_t)param;

    regs->cpsr = ARM_PSR_MODE_SUPER | ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS;

#ifdef CONFIG_CPU_ARM32_BIG_BE8
    regs->cpsr |= ARM_PSR_EE;
#endif

    regs->gpr[14] = 0xa5a5a5a5; /* can not return from context entry */
    regs->gpr[15] = (uintptr_t)entry;

    return 0;
}



void
cpu_context_destroy(struct context_s *context)
{
#if 0
    reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

#endif /* CONFIG_HEXO_CONTEXT */

void cpu_exception_resume_pc(struct cpu_context_s *regs, uintptr_t pc)
{
  regs->gpr[15] = pc;
}

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
