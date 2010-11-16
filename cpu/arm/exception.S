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
  
   Copyright (c) 2010, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/asm.h>
#include <hexo/cpu.h>
#include <hexo/context.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

.macro prepare_exception offset
    // get context registers save array
    CONTEXT_LOCAL_ADDR arm_context_regs, r2, r3

    add    r1, r2, #CPU_ARM_CONTEXT_GPR(4)
    /* store most registers in tmp buffer */
    stmia  r1, {r4, r5, r6, r7, r8, r9, r10, r11}
    str    lr, [r2, #CPU_ARM_CONTEXT_LR]
    str    sp, [r2, #CPU_ARM_CONTEXT_SP]

    /* Test wether it is a bus fault or not */
    mrs    r1, ipsr
    and    r1, r1, #255
    cmp    r1, #5
    /* If the interrupt number is not 5, branch */
    bne    1f

    /* It is a bus fault, check if the stack is ok */
    /* Load the Bus Fault Status Register */
    ldr    r1, =0xE000ED29
    ldrb   r1, [r1]    
    and    r1, #16
    /* If the STKERR bit is null, branch */
    cbz    r1, 1f
    /* The bit is set, pass some register saving */
    b      2f
        
1:      
    /* Store remaining registers */
    /*   Store r0 to r4    */
    add    r1, r2, #CPU_ARM_CONTEXT_GPR(0)
    ldr    r4, [sp]
    ldr    r5, [sp, #4]
    ldr    r6, [sp, #8]
    ldr    r7, [sp, #12]
    stmia  r1, {r4,r5,r6,r7}

    /*   Store R12 and PC   */
    add    r1, r2, #CPU_ARM_CONTEXT_GPR(12)
    ldr    r4, [sp, #16]
    ldr    r5, [sp, #24]
    /* uniformize return address, but avoid a sub when 0 */
    .if \offset != 0
        sub    r5, r5, #\offset
    .endif
    stmia  r1, {r4, r5}

    /* Take old xpsr, save it */
    ldr    r0, [sp, #28]
    str    r0, [r2, #CPU_ARM_CONTEXT_XPSR]
        
2:      
    /* Tell what to restore */
    mov     r0, #CPU_ARM_CONTEXT_RESTORE_CALLEE|CPU_ARM_CONTEXT_RESTORE_CALLER
    str     r0, [r2, #CPU_ARM_CONTEXT_SAVE_MASK]

.endm

.macro preempt_clear
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
    mov    r3, #0
    CPU_LOCAL_SET cpu_preempt_handler, r1, r2, r3
#endif
    ldr    lr, =arm_context_may_preempt
.endm

.macro handle_exception arg
    preempt_clear
    mov    r0, #\arg
    b      arm_exc_common
.endm

#ifdef CONFIG_HEXO_IRQ
.macro handle_irq no
    preempt_clear
    mov    r0, #\no
    b      arm_irq_common
.endm
#endif
    
    /* sp + 28 is exc pc */
FUNC_START(.text, arm_exc_undef)
    prepare_exception 0
    handle_exception CPU_EXCEPTION_ILLEGAL_INS
FUNC_END(arm_exc_undef)

    /* sp + 28 is error pc */
FUNC_START(.text, arm_exc_dabt)
    prepare_exception 0
    handle_exception CPU_EXCEPTION_DATA_ERROR
FUNC_END(arm_exc_dabt)

    /* sp + 28 is error pc + 4 */
FUNC_START(.text, arm_exc_nmi)
    prepare_exception 4
    handle_exception CPU_EXCEPTION_NMI
FUNC_END(arm_exc_nmi)

    /* sp + 28 is error pc */
FUNC_START(.text, arm_exc_hfault)
    prepare_exception 0
    handle_exception CPU_EXCEPTION_HARDFAULT
FUNC_END(arm_exc_hfault)

    /* sp + 28 is error pc */
FUNC_START(.text, arm_exc_mpu)
    prepare_exception 0
    handle_exception CPU_EXCEPTION_MPU_ERROR
FUNC_END(arm_exc_mpu)

    /* sp + 28 is swi pc + 4 */
FUNC_START(.text, arm_exc_swi)
    /* We use caller saved register, which where saved
       on the stack by the hw.
     */

    // Use r2 to reference cpu_context
    CONTEXT_LOCAL_ADDR arm_context_regs, r4, r3

    // Save return address
    str    lr, [r4, #CPU_ARM_CONTEXT_PC]

    // Save SP
    str    sp, [r4, #CPU_ARM_CONTEXT_SP]

     // Save masks
    mrs     r1, basepri
    lsl     r2, r1, #16
    mrs     r1, primask
    and     r1, r1, 0x1
    orr     r2, r2, r1, lsl #8
    mrs     r1, faultmask
    and     r1, r1, 0x1
    orr     r2, r2, r1
    str     r2, [r4, #CPU_ARM_CONTEXT_MASKS]
        
    mov    r3, #CPU_ARM_CONTEXT_RESTORE_CALLEE
    str    r3, [r4, #CPU_ARM_CONTEXT_SAVE_MASK]

    
    preempt_clear
    mov    r1, r4
    b      arm_swi_common
        
FUNC_END(arm_exc_swi)

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
