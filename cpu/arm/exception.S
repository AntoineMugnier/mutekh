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
        /* uniformize return address, but avoid a sub when 0 */
        .if \offset != 0
            sub    lr, lr, #\offset
        .endif

        /* SP points to r0 in cpu_context_s */

        /* store most registers in tmp buffer */
        stmia  sp, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}

        /* Save PC */
        str    lr, [sp, #CPU_ARM_CONTEXT_GPR(15)-CPU_ARM_CONTEXT_GPR(0)]

        /* Take old msr, save it */
        mrs    r0, spsr
        str    r0, [sp, #CPU_ARM_CONTEXT_CPSR-CPU_ARM_CONTEXT_GPR(0)]

        /* last but not least, we have to save sp and lr, this
           may involve shadowed registers */
        add    r1, sp, #CPU_ARM_CONTEXT_GPR(13)-CPU_ARM_CONTEXT_GPR(0)

        /* Switch to super (After taking sp+x in r1) */
        msr      cpsr_c, #0xd3

#if defined(CONFIG_HEXO_USERMODE)
        /* r0 is still cpsr */
        tst    r0, #0xf
        /* if 0 (eq), from user mode */
        /* from user mode, abuse stmia^ */
        stmeqia  r1, {r13, r14}^
        /* We were from super, so save super */
        stmneia  r1, {r13, r14}
#else
        /* Save sp and lr */
        stmia  r1, {r13, r14}
#endif        

        /* Always restore stack pointer */
        CONTEXT_LOCAL context_stack_end, r2, r3
        bic    sp, r2, #63
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
        
        /* r14 is exc pc + 4 */
FUNC_START(.text, arm_exc_undef)
        prepare_exception 4
        handle_exception CPU_EXCEPTION_ILLEGAL_INS
FUNC_END(arm_exc_undef)

        /* r14 is error pc + 4 */
FUNC_START(.text, arm_exc_pabt)
        prepare_exception 4
        handle_exception CPU_EXCEPTION_INS_ERROR
FUNC_END(arm_exc_pabt)

        /* r14 is error pc + 8 */
FUNC_START(.text, arm_exc_dabt)
        prepare_exception 8
        handle_exception CPU_EXCEPTION_DATA_ERROR
FUNC_END(arm_exc_dabt)

#if defined(CONFIG_HEXO_IRQ) && !defined(CONFIG_CPU_ARM_CUSTOM_IRQ_HANDLER)
        /* r14 is error pc + 4 */
FUNC_START(.text, arm_exc_irq)
        prepare_exception 4
        handle_irq 0
FUNC_END(arm_exc_irq)

        /* r14 is error pc + 4 */
FUNC_START(.text, arm_exc_fiq)
        .warning "Context saving is not correct for fiq handling"
        prepare_exception 4
        handle_irq 1
FUNC_END(arm_exc_fiq)
#endif

        /* r14 is swi pc + 4 */
FUNC_START(.text, arm_exc_swi)
        /* Let's say our ABI tells we can trash r12 */
#if defined(CONFIG_HEXO_USERMODE)
        mrs    r12, spsr
        tst    r12, #0xf
        bne    1f // swi from kernel

        /* We are in swi from userland, we cant trust sp, restore it */
        CONTEXT_LOCAL context_stack_end, sp, r12
        bic    sp, sp, #63
1:
#endif

        // Now we have a trusted stack, we can work...
        push   {r11}
        // Use r11 to reference cpu_context
        CONTEXT_LOCAL_ADDR arm_context_regs, r11, r12

        // Save return address
        str    lr, [r11, #CPU_ARM_CONTEXT_GPR(15)]

        // Save caller-saved regs (arguments)
        add    r12, r11, #CPU_ARM_CONTEXT_GPR(2)
        stmia  r12, {r0, r1, r2, r3}

        // Save mode
        mrs    r12, spsr
        str    r12, [r11, #CPU_ARM_CONTEXT_CPSR]
#if defined(CONFIG_HEXO_USERMODE)
        /* r12 is still cpsr */
        tst    r12, #0xf
        /* if 0 (eq), from user mode */
        /* from user mode, abuse stmia^ */
        stmeqia  r11, {r13, r14}^
        /* We were from super, so save super */
        stmneia  r11, {r13, r14}
#else
        /* Save sp and lr */
        stmia  r11, {r13, r14}
#endif
        
        // And finally save r11 (through r12)
        pop    {r12}
        str    r12, [r11, #CPU_ARM_CONTEXT_GPR(11)]

        preempt_clear
        mov    r1, r11
        b      arm_swi_common
FUNC_END(arm_exc_swi)

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
