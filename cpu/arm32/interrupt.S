/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.
  
   Copyright (c) 2010, Nicolas Pouillon <nipo@ssji.net>
*/

/*
    Use of register banks:

    Kernel code runs in SUPER mode.

    C code called from irq handler is executed in FIQ mode, r0-r4 are saved in
    FIQ shadow registers r8-r11. When CONFIG_HEXO_INTERRUPT_STACK is defined,
    the FIQ stack pointer register contains the address of the processor
    interrupt stack; The value of the SUPER stack pointer is used in
	the other case.

    The stack pointer registers of the IRQ, ABORT, UNDEF modes contain
	a pointer to the cpu_context_s struct for the current context
	running on the processor.
*/

#include <hexo/asm.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>
#include <hexo/context.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

.section .excep,"ax"

CPU_NAME_DECL(exception_vector):
        .globl CPU_NAME_DECL(exception_vector)
#ifndef CONFIG_LOAD_RESET_SEPARATE
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
#endif

#ifdef CONFIG_CPU_ARM32_BIG_BE8
        .irp t, 1f, 2f, 3f, 4f, 5f, 6f, 7f, 8f
        b     \t
        .endr
#endif

.macro ARM_EXCEP_ENTRY n, func
#ifdef CONFIG_CPU_ARM32_BIG_BE8
        setend  be
#else
        .org CPU_NAME_DECL(exception_vector) + \n * 4
#endif
        ldr     pc, =\func
.endm

1:
    ARM_EXCEP_ENTRY 0, mutekh_entry

2:
#ifdef CONFIG_HEXO_EXCEP
    ARM_EXCEP_ENTRY 1, arm_exc_undef
#endif

3:
#ifdef CONFIG_HEXO_USERMODE
    ARM_EXCEP_ENTRY 2, arm_exc_swi
#endif

4:
#ifdef CONFIG_HEXO_EXCEP
    ARM_EXCEP_ENTRY 3, arm_exc_pabt
#endif

5:
#ifdef CONFIG_HEXO_EXCEP
    ARM_EXCEP_ENTRY 4, arm_exc_dabt
#endif

6:
#ifdef CONFIG_HEXO_EXCEP
    ARM_EXCEP_ENTRY 5, arm_exc_undef // reserved
#endif

7:
#ifdef CONFIG_HEXO_IRQ
    ARM_EXCEP_ENTRY 6, arm_exc_irq
#endif

8:
#ifdef CONFIG_HEXO_IRQ
    b 8b   /* FIQ */
#endif

/**********************************************************/

#ifdef CONFIG_HEXO_CONTEXT
.macro ARM_EXC_SAVECONTEXT
    /* Save PC */
    str    lr, [sp, #CPU_ARM_CONTEXT_GPR(15)]

    /* Tell what to restore */
    mov    r2, #CPU_ARM_CONTEXT_RESTORE_CALLEE|CPU_ARM_CONTEXT_RESTORE_CALLER
    str    r2, [sp, #CPU_ARM_CONTEXT_SAVE_MASK]

    /* Take old msr, save it */
    mrs    r2, spsr
    str    r2, [sp, #CPU_ARM_CONTEXT_CPSR]

    /* we still have to save sp and lr from SUPER or USER mode */
    add    r4, sp, #CPU_ARM_CONTEXT_GPR(13)

    /* switch to SUPER */
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SUPER)

# if defined(CONFIG_HEXO_USERMODE)
    /* r2 is still cpsr */
    tst    r2, #0xf
    /* if 0 (eq) we come from user */

    /* from user mode, abuse stmia^ */
    stmeq  r4, {r13, r14}^
    /* We were from super, so save super's sp and lr */
    stmne  r4, {r13, r14}
# else
    stm    r4, {r13, r14}
# endif
.endm
#endif

.macro ARM_EXC_ENTRY mode id
#ifdef CONFIG_HEXO_CONTEXT
    /* SP points to cpu_context_s when mode is either IRQ, ABORT, UNDEF */
    stm    sp, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}
#else
    /* save r12 */
    mov    sp, r12
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SUPER)

    /* get stack pointer from SUPER */
    sub    r12, sp, #_sizeof(struct cpu_context_s)
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | \mode)

    /* store cpu_context_s on stack */
    stm    r12, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11}
    str    sp, [r12, #CPU_ARM_CONTEXT_GPR(12)]
    str    lr, [r12, #CPU_ARM_CONTEXT_GPR(15)]
    add    r4, r12, #_sizeof(struct cpu_context_s)
    str    r4, [r12, #CPU_ARM_CONTEXT_GPR(13)]

    /* setup sp reg */
    mov    sp, r12
#endif
    mov    r0, #\id
.endm


#ifdef CONFIG_HEXO_EXCEP
FUNC_START(.text, arm_exc_undef)
    /* r14 is exc pc + 4 */
    sub    lr, lr, #4
    ARM_EXC_ENTRY ARM_PSR_MODE_UNDEF CPU_EXCEPTION_ILLEGAL_INS
    b   arm_exc_common_asm
FUNC_END(arm_exc_undef)

FUNC_START(.text, arm_exc_pabt)
    /* r14 is error pc + 4 */
    sub    lr, lr, #4
    ARM_EXC_ENTRY ARM_PSR_MODE_ABORT CPU_EXCEPTION_INS_ERROR
    b   arm_exc_common_asm
FUNC_END(arm_exc_pabt)

FUNC_START(.text, arm_exc_dabt)
    /* r14 is error pc + 8 */
    sub    lr, lr, #8
    ARM_EXC_ENTRY ARM_PSR_MODE_ABORT CPU_EXCEPTION_DATA_ERROR
    b   arm_exc_common_asm
FUNC_END(arm_exc_dabt)


FUNC_START(.text, arm_exc_common_asm)

    /* SP points to cpu_context_s when mode is either IRQ, ABORT, UNDEF,
       cpu_context_s is stored on stack when !CONFIG_HEXO_CONTEXT */
    mov    r3, sp

#ifdef CONFIG_HEXO_CONTEXT
    ARM_EXC_SAVECONTEXT     /* switch to SUPER */

    mov    r1,  sp
    /* switch to FIQ because it has many shadow regs */
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_FIQ)
    and    sp,  r1, #~7      /* import our stack pointer from SUPER and align */

    /* passed to arm_context_jumpto_fast / arm_except_preempt */
    mov    r11, r3
#endif

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    /* clear preempt handler */
    mov    r12, #0
    CPU_LOCAL_st cpu_preempt_handler, r2, r4, r12
#endif

#ifdef CONFIG_HEXO_USERMODE
    tst    r2, #0xf
    bne    1f

    CONTEXT_LOCAL_ld cpu_user_exception_handler, r12, r2
    cmp    r12, #0
    bne    2f
1:
#endif

    CPU_LOCAL_ld cpu_exception_handler, r12, r2
2:
    /* first arg is number, already in r0 */
    /* fourth arg is cpu_context_s, already in r3 */

    /* fifth arg on stack */
    ldr     r2, [r3, #CPU_ARM_CONTEXT_GPR(13)]
    sub     sp, #4   /* keep alignment */
    push    {r2}

    /* second arg is pc */
    ldr     r1, [r3, #CPU_ARM_CONTEXT_GPR(15)]

    /* third arg */
    mov     r2, #0

    blx    r12
    add    sp, #8

#if defined(CONFIG_HEXO_CONTEXT_PREEMPT)
    b      arm_except_preempt
#elif defined(CONFIG_HEXO_CONTEXT)
    b      arm_context_jumpto_fast
#else
    ldm    sp,   {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}
    ldr    lr,   [sp, #CPU_ARM_CONTEXT_GPR(15)]
    movs   pc,   lr
#endif

FUNC_END(arm_exc_common_asm)
#endif /* CONFIG_HEXO_EXCEP */


#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
FUNC_START(.text, arm_except_preempt)
    /* test the preempt handler */
    CPU_LOCAL_ld cpu_preempt_handler, r12, r2
    movs   r12, r12
    beq    arm_context_jumpto_fast

    /* call the preempt handler and test the result */
    blx    r12
    movs   r0, r0
    beq    arm_context_jumpto_fast

# ifdef CONFIG_HEXO_CONTEXT_STATS
    mov    r4, r0
    bl     context_preempt_stats
    mov    r0, r4
# endif

    b      cpu_context_jumpto
FUNC_END(arm_except_preempt)
#endif

#if defined(CONFIG_HEXO_IRQ) && !defined(CONFIG_CPU_ARM32_CUSTOM_IRQ_HANDLER)
    /* r14 is error pc + 4 */
FUNC_START(.text, arm_exc_irq)
    sub    lr, lr, #4

    /* copy caller saved in fiq shadowed regs */
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_FIQ)
    mov    r8,   r0
    mov    r9,   r1
# ifdef CONFIG_HEXO_CONTEXT
    mov    r10,  r2
    mov    r11,  r3
# endif

    /* get stack pointer */
# ifndef CONFIG_HEXO_INTERRUPT_STACK
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SUPER)
    mov    r0,  sp
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_FIQ)
    and    sp, r0, #~7      /* import our stack pointer from SUPER and align */
# endif

    /* clear preempt handler */
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    mov    r3, #0
    CPU_LOCAL_st cpu_preempt_handler, r1, r2, r3
# endif

    /* call C handler */
    CPU_LOCAL_ld cpu_interrupt_handler, r12, r1
    mov    r0, #0
    blx    r12

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    /* test the preempt handler */
    CPU_LOCAL_ld cpu_preempt_handler, r2, r3
    movs   r2, r2
    bne    1f

    /* invalidate handler outside interrupt */
    mov    r3, #1
    CPU_LOCAL_st cpu_preempt_handler, r1, r2, r3
# endif

2:
    /* restore caller saved regs from fiq shadowed regs */
    mov    r0,   r8
    mov    r1,   r9
# ifdef CONFIG_HEXO_CONTEXT
    mov    r2,  r10
    mov    r3,  r11
# endif
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_IRQ)

    /* return from irq */
    movs   pc,   lr

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
1:
    /* call the preempt handler and test the result */
    blx    r2
    movs   r12, r0              /* r12_fiq */
    beq    2b

    /* save all regs */
    mov    r0,   r8
    mov    r1,   r9
    mov    r2,  r10
    mov    r3,  r11
    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_IRQ)
    stmia  sp, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}
    ARM_EXC_SAVECONTEXT

    msr    cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_FIQ)
#  ifdef CONFIG_HEXO_CONTEXT_STATS
    mov    r4, r12              /* r12_fiq */
    mov    r0, r12
    bl     context_preempt_stats
    mov    r0, r4
#  else
    mov    r0, r12
#  endif
    b      cpu_context_jumpto

# endif

FUNC_END(arm_exc_irq)
#endif


#ifdef CONFIG_HEXO_USERMODE
FUNC_START(.text, arm_exc_swi)
    /* r14 is swi pc + 4 */

    /* We must save r11, but we have nothing else than a stack, so
     * use it. Beware of side-effects when playing with sp (see below)
     */
    push   {r11}

    // Use r11 to reference cpu_context
    CONTEXT_LOCAL_addr arm_context_regs, r11, r12

    // Save return address
    str    lr, [r11, #CPU_ARM_CONTEXT_GPR(15)]

    mov    r12, #CPU_ARM_CONTEXT_RESTORE_CALLEE|CPU_ARM_CONTEXT_RESTORE_CALLER
    str    r12, [r11, #CPU_ARM_CONTEXT_SAVE_MASK]

    // Save caller-saved regs (arguments)
    stmia  r11, {r0, r1, r2, r3}

    // Save mode
    mrs    r1, spsr
    str    r1, [r11, #CPU_ARM_CONTEXT_CPSR]

    /* From user mode, use stmia^ to save user sp and lr */
    add    r12, r11, #CPU_ARM_CONTEXT_GPR(13)
    stmia  r12, {r13, r14}^

    /* And finally save r11 */
    pop    {r12}
    str    r12, [r11, #CPU_ARM_CONTEXT_GPR(11)]

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    /* clear preempt handler */
    mov    r3, #0
    CPU_LOCAL_st cpu_preempt_handler, r1, r2, r3
# endif

    CONTEXT_LOCAL_ld cpu_syscall_handler, r12, r2
    mov    r0, #0
    mov    r1,  r11
    blx    r12

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    b      arm_except_preempt
# else
    b      arm_context_jumpto_fast
# endif

FUNC_END(arm_exc_swi)
#endif

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
