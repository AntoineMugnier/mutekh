
#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <mutek/scheduler.h>

CONTEXT_LOCAL struct cpu_context_s arm_context_regs;

#if !defined(CONFIG_CPU_ARM_TLS_IN_C15)
CPU_LOCAL void *__context_data_base;
#endif

#define arm_setup_exception_stack(context, psr_mode)				   \
	asm volatile(													   \
		"mrs  r2, cpsr            \n\t"								   \
		"bic  r3, r2, #0x1f       \n\t"								   \
		"orr  r3, r3, %0          \n\t"								   \
		"msr  cpsr, r3            \n\t"								   \
		"mov  sp, %1              \n\t"								   \
		"msr  cpsr, r2            \n\t"								   \
		:															   \
		: "i"(psr_mode), "r"(context)								   \
		: "r2", "r3" );

static void __arm_exception_setup()
{
    struct cpu_context_s *ctx = CONTEXT_LOCAL_ADDR(arm_context_regs);
    uintptr_t addr = (uintptr_t)&ctx->gpr[0];

#ifdef CONFIG_SOCLIB_MEMCHECK
	soclib_mem_check_disable(SOCLIB_MC_CHECK_SPFP);
#endif

	arm_setup_exception_stack(addr, 0x12); // IRQ
	arm_setup_exception_stack(addr, 0x17); // Abort
	arm_setup_exception_stack(addr, 0x1b); // Undef

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
        CPU_ARM_CONTEXT_RESTORE_CALLEE;
    regs->gpr[13] =
        CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_end)
        - CONFIG_HEXO_STACK_ALIGN;
    regs->gpr[0] = (uintptr_t)param;

    regs->cpsr = 0x000000d3;

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

__attribute__((noreturn))
extern void arm_context_jumpto_back();

inline struct context_s *arm_except_preempt()
{
    struct context_s *ctx = NULL;
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    context_preempt_t *handler = CPU_LOCAL_GET(cpu_preempt_handler);
    if ( handler ) {
        ctx = handler(CPU_LOCAL_GET(cpu_preempt_param));
    }

# ifdef CONFIG_HEXO_CONTEXT_STATS
    if ( ctx ) {
        context_preempt_stats(ctx);
    }
# endif
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
#ifdef CONFIG_HEXO_USERMODE
    if ( (context->cpsr & 0xf) == 0 )
        handler = CONTEXT_LOCAL_GET(cpu_user_exception_handler);
#endif  
    if ( handler == NULL )
        handler = CPU_LOCAL_GET(cpu_exception_handler);
    handler(no,
            (void *)context->gpr[15],
            0,
            &context->gpr[0],
            context->gpr[13]);

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

    return arm_except_preempt();
}
#endif

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
