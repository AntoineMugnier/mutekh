/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

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
   Caller saved: r0-r3 r12 xpsr lr
    + Arguments: r0-r3
*/

FUNC_START(.text, cpu_context_switch)
    // get context registers save array
    CONTEXT_LOCAL_ADDR arm_context_regs, r2, r3

    // save "callee saved" registers
    add     r1, r2, #CPU_ARM_CONTEXT_GPR(4)
    stmia   r1, {r4, r5, r6, r7, r8, r9, r10, r11}
    str     r13, [r2, #CPU_ARM_CONTEXT_SP]
        
    // write save/restore mask
    mov     r4, #CPU_ARM_CONTEXT_RESTORE_CALLEE
    str     r4, [r2, #CPU_ARM_CONTEXT_SAVE_MASK]

    // write special register                      
    mrs     r4, xpsr                               
    str     r4, [r2, #CPU_ARM_CONTEXT_XPSR]        
    // write masks                                 
    mrs     r4, basepri                            
    lsl     r5, r4, #16                                    
    mrs     r4, primask                            
    and     r4, r4, 0x1                            
    orr     r5, r5, r4, lsl #8                     
    mrs     r4, faultmask                          
    and     r4, r4, 0x1                            
    orr     r5, r5, r4                             
    str     r5, [r2, #CPU_ARM_CONTEXT_MASKS]       
    // write lr with the current pc                
    str     r14, [r2, #CPU_ARM_CONTEXT_PC]         
   
    b       cpu_context_jumpto
FUNC_END(cpu_context_switch)

FUNC_START(.text, cpu_context_jumpto)
    // restore tls pointer from context struct
    ldr  r3, [r0, #HEXO_CONTEXT_S_TLS * 4]
    TLS_BASE_SET r3, r2

    b arm_context_jumpto_internal
FUNC_END(cpu_context_jumpto)

        
FUNC_START(.text, arm_context_may_preempt)
#if defined(CONFIG_HEXO_CONTEXT_PREEMPT)
    movs  r0, r0
    IT    NE
    bne   cpu_context_jumpto
#endif
    b arm_context_jumpto_internal
FUNC_END(arm_context_may_preempt)

FUNC_START(.text, arm_context_jumpto_internal)
    // context registers save array
    CONTEXT_LOCAL_ADDR arm_context_regs, r2, r3

    // Setup r0 to be &r0
    add     r0, r2, #CPU_ARM_CONTEXT_GPR(0)

    // Take &[r13 r14] in r3
    add     r3, r2, #CPU_ARM_CONTEXT_SP

#if defined(CONFIG_HEXO_USERMODE) // not supported
    // Load spsr
    ldr     lr, [r2, #CPU_ARM_CONTEXT_CPSR]
#endif


#if defined(CONFIG_HEXO_USERMODE) // not supported
    /* From now on, we can only touch lr, which is spsr for now. */
    
    /* If we are from user mode, restore user regs */
    tst     lr, #0xf
    /* if 0 (eq), from user mode */
    /* from user mode, use ldmia^ */
    ldmeqia  sp, {r13, r14}^
    /* We were from super, so restore in place */
    ldmneia  sp, {r13, r14}
#else
    // sp is in r3
    ldmia    r3, {r4, r14}
    mov      r13, r4
#endif

    // Restore masks
    ldr      r5, [r2, #CPU_ARM_CONTEXT_MASKS]
    and      r4, r5, #1
    msr      faultmask, r4
    lsr      r5, r5, #8
    and      r4, r5, #1
    msr      primask, r4
    lsr      r5, r5, #8
    and      r4, r5, #255
    msr      basepri, r4

    // Test whether we need to restore caller saved
    ldr     r1, [r2, #CPU_ARM_CONTEXT_SAVE_MASK]
    tst     r1, #CPU_ARM_CONTEXT_RESTORE_CALLER
    ITT     EQ
    // We dont
    addeq   r0, r2, #CPU_ARM_CONTEXT_GPR(4)
    ldmiaeq r0, {r4, r5, r6, r7, r8, r9, r10, r11, r12, pc}
        
    // We do
    // Restore special registers
    ldr      r4, [r2, #CPU_ARM_CONTEXT_XPSR]
    msr      xpsr, r4
    // r0 is &r0
    ldmia   r0, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, pc}
FUNC_END(arm_context_jumpto_internal)


FUNC_START(.text, cpu_context_set)
    /* Set stack pointer, 64 bits aligned */
    add  r4, r0, r1
    bic  r4, #7
    mov  r13,r4

    /* Jump */
    bx   r2
FUNC_END(cpu_context_set)



FUNC_START(.text, cpu_context_stack_use)

    // restore tls pointer from context struct
    ldr  r5, [r0, #HEXO_CONTEXT_S_TLS * 4]
    TLS_BASE_SET r5, r6

    // context registers save array
    CONTEXT_LOCAL_ADDR arm_context_regs, r6, r3

    // use stack ptr
    ldr     r13, [r6, #CPU_ARM_CONTEXT_SP]

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




#if defined(CONFIG_HEXO_USERMODE) && 0 // broken

FUNC_START(.text, cpu_context_set_user)
        ".set push                     \n"
        ".set noat                     \n"
        /* set stack */
        "   move    $sp,    %[ustack]  \n"
        /* set arg */
        "   move    $4,     %[param]   \n"
        "   addiu   $sp,    -4*4       \n"
        "   move    $t9,    %[entry]   \n"
# if CONFIG_CPU_MIPS_VERSION >= 32
        /* entrypoint goes into epc */
        "   mtc0    $t9,    $14        \n"
        "   mfc0    $1,     $12        \n"
        /* status.ksu=10; status.exl=1; status.ie=1 */
#if 0
        /* if supervisor mode is not implemented,
         * ksu.0=R0="3rd bit of r_status" is ignored.
         * No need to reset it then.
         */
        "   andi    $1,     0xFFFFFFF7 \n"
#endif
        "   ori     $1,     0x00000013 \n"
        "   mtc0    $1,     $12        \n"
        ".set noreorder                \n"
        /* switch to user mode at @EPC with interrupt enabled */
        "   eret                       \n"
# else
        /* set previous state as user */
        "   mfc0    $1,     $12        \n"
        "   ori     $1,     0xC        \n"
        "   mtc0    $1,     $12        \n"
        /* "restore" user mode and jump */
        ".set noreorder                \n"
        "   jr      $t9                \n"
        "   rfe                        \n"
# endif
        ".set pop                      \n"

FUNC_END(cpu_context_set_user)

#endif

