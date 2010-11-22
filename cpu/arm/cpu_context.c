
#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <mutek/scheduler.h>

CONTEXT_LOCAL struct cpu_context_s arm_context_regs;

#if !defined(CONFIG_CPU_ARM_TLS_IN_C15)
CPU_LOCAL void *__context_data_base;
#endif

static void __arm_exception_setup()
{
    struct cpu_context_s *ctx = CONTEXT_LOCAL_ADDR(arm_context_regs);
    uintptr_t addr = (uintptr_t)&ctx->gpr[0];

#ifdef CONFIG_SOCLIB_MEMCHECK
	soclib_mem_check_disable(SOCLIB_MC_CHECK_SPFP);
#endif


#ifdef CONFIG_SOCLIB_MEMCHECK
	soclib_mem_check_enable(SOCLIB_MC_CHECK_SPFP);
#endif
}

error_t
cpu_context_bootstrap(struct context_s *context)
{
    /* set context local storage register base pointer */
#if defined(CONFIG_CPU_ARM_TLS_IN_C15)
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

	__arm_exception_setup();

    return 0;
}


/* context init function */

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
    struct cpu_context_s *regs = CONTEXT_LOCAL_TLS_ADDR(context->tls, arm_context_regs);

    regs->save_mask =
        CPU_ARM_CONTEXT_RESTORE_CALLER |
        CPU_ARM_CONTEXT_RESTORE_CALLEE |
        CPU_ARM_CONTEXT_RESTORE_PC;
    regs->sp =
        CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_end)
        - CONFIG_HEXO_STACK_ALIGN;
    regs->gpr[0] = (uintptr_t)param;

    regs->xpsr  = 0x01000000;
    regs->masks = 0x00000102; // interrupt disable, fault enable, Process Stack 

    regs->lr    = 0xa5a5a5a5; /* can not return from context entry */
    regs->pc    = (uintptr_t)entry;

    return 0;
}



void
cpu_context_destroy(struct context_s *context)
{
#if 0
    reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

__attribute__((noreturn))
extern void arm_context_jumpto_back();

struct context_s *arm_except_preempt()
{
    struct context_s *ctx = NULL;
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    context_preempt_t *handler = CPU_LOCAL_GET(cpu_preempt_handler);
    if ( handler ) {
        ctx = handler(CPU_LOCAL_GET(cpu_preempt_param));
    }
#endif

    return ctx;
}

#ifdef CONFIG_HEXO_USERMODE
extern CONTEXT_LOCAL cpu_exception_handler_t  *cpu_user_exception_handler = NULL;
#endif

extern CPU_LOCAL cpu_exception_handler_t  *cpu_exception_handler;

struct context_s *arm_exc_common(reg_t no, struct cpu_context_s *context)
{
    cpu_exception_handler_t *handler = NULL;
    uintptr_t *data_ptr = NULL;
    if(no == CPU_EXCEPTION_DATA_ERROR)
        data_ptr = *((uintptr_t *)0xE000ED38);
#ifdef CONFIG_HEXO_USERMODE
    if ( (context->cpsr & 0xf) == 0 )
        handler = CONTEXT_LOCAL_GET(cpu_user_exception_handler);
#endif  
    if ( handler == NULL )
        handler = CPU_LOCAL_GET(cpu_exception_handler);
    handler(no,
            (void *)context->pc,
            data_ptr,
            &context->gpr[0],
            context->sp);

    return arm_except_preempt();
}

#ifdef CONFIG_HEXO_USERMODE
extern CONTEXT_LOCAL cpu_syscall_handler_t  *cpu_syscall_handler = NULL;
#endif

struct context_s *arm_swi_common(reg_t unused, struct cpu_context_s *context)
{
#ifdef CONFIG_HEXO_USERMODE
    cpu_syscall_handler_t *handler =
        CONTEXT_LOCAL_GET(cpu_syscall_handler);
    handler(0, &context->gpr[0]);
#endif

    return arm_except_preempt();
}

#ifdef CONFIG_HEXO_IRQ
extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;
extern CPU_LOCAL struct device_s *cpu_interrupt_handler_dev;

struct context_s *arm_irq_common(reg_t no, struct cpu_context_s *context)
{
    cpu_interrupt_handler_t *handler =
        CPU_LOCAL_GET(cpu_interrupt_handler);
    handler(no);

    //    return arm_except_preempt();
    return NULL;
}
#endif

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
