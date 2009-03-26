
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#if defined(CONFIG_CPU_USER)
CONTEXT_LOCAL uintptr_t context_kstack;
#endif

error_t
cpu_context_bootstrap(struct context_s *context)
{
  /* set context local storage base pointer */
  CPU_LOCAL_SET(__cpu_context_data_base, context->tls);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_change_id(cpu_id(), (uint32_t)&context->stack_ptr);
#endif

  return 0;
}


/* fake context entry point, pop entry function param and entry
   function address from stack and perform jump to real entry
   function.  We need to do this to pass an argument to the context
   entry function through register. */
void __mips_context_entry(void);

asm(
    ".set push			\n"
    ".set noreorder		\n"
    ".set noat			\n"
    "__mips_context_entry:		\n"
    "	lw	$4,	0($sp)	\n" /* entry function param */
    "	lw	$1,	4($sp)	\n" /* entry function address */
    "	jr	$1		\n"
    "	addiu	$sp,	2*4	\n"
    ".set pop			\n"
    );



/* context init function */

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_create_ctx((uint32_t)&context->stack_ptr,
			      context->stack_start, context->stack_end);
#endif

  context->stack_ptr = (reg_t*)context->stack_end - 5;

  /* push entry function address and param arg */
  *--context->stack_ptr = (uintptr_t)entry;
  *--context->stack_ptr = (uintptr_t)param;

  /* fake entry point */
  *--context->stack_ptr = (uintptr_t)&__mips_context_entry;

  /* frame pointer */
  *--context->stack_ptr = 0;

  /* status register, interrupts are disabled */
  *--context->stack_ptr = 0x0000ff00;

  /* context local storage address */
  *--context->stack_ptr = (uintptr_t)context->tls;

  return 0;
}



void
cpu_context_destroy(struct context_s *context)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_SPFP);
  soclib_mem_check_delete_ctx((uint32_t)&context->stack_ptr);
#endif
#if 0
  reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

# if defined(CONFIG_CPU_USER)

void __attribute__((noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack,
		     user_entry_t *entry, void *param)
{
  cpu_interrupt_disable();

  CONTEXT_LOCAL_SET(context_kstack, kstack);

  asm volatile (
		".set push				\n"
		".set noat				\n"
		/* set stack */
		"	move	$sp,	%0              \n"
                /* set arg */
                "	move    $4,     %2		\n"
                "	addiu   $sp,    -4*4		\n"
                /* set previous state as user */
                "	mfc0    $7,     $12		\n"
                "	ori     $7,     0xC		\n"
                "	mtc0    $7,     $12		\n"
                /* "restore" user mode and jump */
                ".set noreorder				\n"
                "	jr          %1			\n"
                "	rfe				\n"
                ".set pop				\n"
                :
                : "r" (ustack)
                , "r" (entry)
                , "r" (param)
                );
}

#endif

