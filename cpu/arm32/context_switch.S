/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2010, Nicolas Pouillon, <nipo@ssji.net>
*/

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

/*
   Callee saved: r4-r11 r13
   Caller saved: r12
    + Arguments: r0-r3
*/

FUNC_START(.text, cpu_context_switch)
    // get context registers save array
    CONTEXT_LOCAL_ADDR arm_context_regs, r2, r3

    // save "callee saved" registers
    add     r1, r2, #CPU_ARM_CONTEXT_GPR(4)
    stmia   r1, {r4, r5, r6, r7, r8, r9, r10, r11}
    str     r13, [r2, #CPU_ARM_CONTEXT_GPR(13)]

    // write save/restore mask
    mov     r4, #CPU_ARM_CONTEXT_RESTORE_CALLEE
    str     r4, [r2, #CPU_ARM_CONTEXT_SAVE_MASK]

    mrs     r4, cpsr
    str     r4, [r2, #CPU_ARM_CONTEXT_CPSR]
    str     r14, [r2, #CPU_ARM_CONTEXT_GPR(15)]
    
    b       cpu_context_jumpto
FUNC_END(cpu_context_switch)


.macro CPU_EXC_SP_SETUP rsp
#ifdef CONFIG_HEXO_IRQ
    msr  cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_IRQ)
    mov  sp, \rsp
#endif
    msr  cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_UNDEF)
    mov  sp, \rsp
    msr  cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_ABORT)
    mov  sp, \rsp
.endm


FUNC_START(.text, arm_setup_exception_stack)

#ifdef CONFIG_SOCLIB_MEMCHECK
    /* enter memchecker command mode */
    ldr  r2, =SOCLIB_MC_MAGIC_VAL 
    ldr  r1, =CONFIG_SOCLIB_MEMCHECK_ADDRESS 
    str  r2, [r1, #SOCLIB_MC_MAGIC_OFFSET]
#endif
        
    CPU_EXC_SP_SETUP r0
    msr  cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SUPER)

#ifdef CONFIG_SOCLIB_MEMCHECK
    mov  r3, #0
    str  r3, [r1, #SOCLIB_MC_MAGIC_OFFSET]
#endif

    mov pc, lr
FUNC_END(arm_setup_exception_stack)

        
FUNC_START(.text, cpu_context_jumpto)

# ifdef CONFIG_ARCH_SMP
    // unlock some atomic value on context restore if requested
    ldr   r2, [r0, #HEXO_CONTEXT_S_UNLOCK * 4]
    cmp   r2, #0
    movne r1, #0
    strne r1, [r0, #HEXO_CONTEXT_S_UNLOCK * 4]
    strne r1, [r2]
#endif

    // restore tls pointer from context struct
    ldr  r3, [r0, #HEXO_CONTEXT_S_TLS * 4]
    TLS_BASE_SET r3, r2

    // context registers save array
    CONTEXT_LOCAL_addr arm_context_regs, r2, r3

#ifdef CONFIG_SOCLIB_MEMCHECK
    /* enter memchecker command mode */
    ldr  r3, =SOCLIB_MC_MAGIC_VAL 
    ldr  r1, =CONFIG_SOCLIB_MEMCHECK_ADDRESS 
    str  r3, [r1, #SOCLIB_MC_MAGIC_OFFSET]

    /* switch to associated memchecker context */
    str  r0, [r1, #SOCLIB_MC_CTX_SET_OFFSET]

    mov  r3, #0
    str  r3, [r1, #SOCLIB_MC_MAGIC_OFFSET]
#endif

    // Set sp to context &arm_context_regs for all error modes
    CPU_EXC_SP_SETUP r2

arm_context_jumpto_fast:
    .globl arm_context_jumpto_fast

    /* We are in ABORT mode from here */
    msr  cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_ABORT)

    ldr     r1, [sp, #CPU_ARM_CONTEXT_CPSR]
    add     r0, sp, #CPU_ARM_CONTEXT_GPR(13)

    // restore r13 and r14
    msr     cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SUPER)

#if defined(CONFIG_HEXO_USERMODE)
    tst     r1, #0xf
    /* If we are from user mode, restore user regs */
    ldmeq   r0, {r13, r14}^
    /* restore super regs instead */
    ldmne   r0, {r13, r14}
#else
    ldm     r0, {r13, r14}
#endif

    msr     cpsr_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_ABORT)

    // Load pc
    ldr     lr, [sp, #CPU_ARM_CONTEXT_GPR(15)]

    // restore status
    msr     spsr_cxsf, r1

    // Test whether we need to restore caller saved
    ldr     r1, [sp, #CPU_ARM_CONTEXT_SAVE_MASK]
    tst     r1, #CPU_ARM_CONTEXT_RESTORE_CALLER

    // r0 is &r0
    ldmne   sp, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}
    // We dont
    addeq   r0, sp, #CPU_ARM_CONTEXT_GPR(4)
    ldmeq   r0, {r4, r5, r6, r7, r8, r9, r10, r11}

    // Restore
    movs     pc, lr

FUNC_END(cpu_context_jumpto)


FUNC_START(.text, cpu_context_set)
#ifdef CONFIG_SOCLIB_MEMCHECK
    /* enter memchecker command mode */
    ldr  r3, =SOCLIB_MC_MAGIC_VAL 
    ldr  r4, =CONFIG_SOCLIB_MEMCHECK_ADDRESS 
    str  r3, [r4, #SOCLIB_MC_MAGIC_OFFSET]

    /* mark current memchecker context as invalid */
    mov  r3, #SOCLIB_MC_CTX_ID_CURRENT
    str  r3, [r4, #SOCLIB_MC_CTX_INVALIDATE_OFFSET]

    /* create a new temporary memchecker context using passed stack */
    str  r0, [r4, #SOCLIB_MC_R1_OFFSET]
    str  r1, [r4, #SOCLIB_MC_R2_OFFSET]
    str  r0, [r4, #SOCLIB_MC_CTX_CREATE_OFFSET]

    /* switch to new memchecker context */
    str  r0, [r4, #SOCLIB_MC_CTX_SET_OFFSET]
#endif

    /* Set stack pointer, 64 bits aligned */
    add  r13, r0, r1
    bic  r13, #63
#ifdef CONFIG_COMPILE_FRAMEPTR
    mov  r11, r13
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
    /* leave memchecker command mode */
    mov  r0, #0
    str  r0, [r4, #SOCLIB_MC_MAGIC_OFFSET]
#endif

    /* Jump */
    bx   r2
FUNC_END(cpu_context_set)



FUNC_START(.text, cpu_context_stack_use)
#ifdef CONFIG_SOCLIB_MEMCHECK
    /* enter memchecker command mode */
    ldr  r3, =SOCLIB_MC_MAGIC_VAL 
    ldr  r4, =CONFIG_SOCLIB_MEMCHECK_ADDRESS 
    str  r3, [r4, #SOCLIB_MC_MAGIC_OFFSET]

    /* switch to associated memchecker context */
    str  r0, [r4, #SOCLIB_MC_CTX_SET_OFFSET]
#endif

    // restore tls pointer from context struct
    ldr  r5, [r0, #HEXO_CONTEXT_S_TLS * 4]
    TLS_BASE_SET r5, r6

    // context registers save array
    CONTEXT_LOCAL_addr arm_context_regs, r6, r3

    // use stack ptr
    ldr     r13, [r6, #CPU_ARM_CONTEXT_GPR(13)]
#ifdef CONFIG_COMPILE_FRAMEPTR
    mov  r11, r13
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
    /* leave memchecker command mode */
    mov  r0, #0
    str  r0, [r4, #SOCLIB_MC_MAGIC_OFFSET]
#endif

#ifndef CONFIG_RELEASE
    // prevent use of previous context local storage
    mov  r6, #0
    TLS_BASE_SET r6, r3
#endif

    // private param
    mov     r0, r2

    // call function
    bx      r1
FUNC_END(cpu_context_stack_use)




#ifdef CONFIG_HEXO_USERMODE

FUNC_START(.text, cpu_context_set_user)
    /* set user stack reg */
    push    {r0}
    ldm     sp, {r13}^

    /* set arg */
    mov     r0,     r2

    /* set USER mode psr */
# ifndef CONFIG_HEXO_IRQ
    mov     r3,     #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_USER)
# else
    mov     r3,     #(ARM_PSR_FIQ_DIS | ARM_PSR_MODE_USER)
# endif

# ifdef CONFIG_CPU_ARM32_BIG_BE8
    orr     r3,     #ARM_PSR_EE
# endif

    /* Restore to USER mode */
    msr     spsr_cxsf,   r3

    /* set SUPER mode stack pointer */
    CONTEXT_LOCAL_ld context_stack_end, r13, r3

    /* jump to user mode */
    mov     lr,     r1
    movs    pc,     lr
FUNC_END(cpu_context_set_user)

#endif

