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
        /* store r0 and exc pc in tmp buffer */
        stmia  sp, {r0, r1, lr}
        /* Take old msr, save it */
        mrs    r1, spsr
        str    r1, [sp, #12]
        /* take pointer to tmp buffer */
        mov    r0, sp
        /* disable irqs, switch to supervisor */
        msr    cpsr_c, #0xd3
        /* save super's lr */
        str    lr, [sp, #16]
        CONTEXT_LOCAL arm_context_regs, r1, lr
        bl     arm_interrupt_save_ctx
.endm

        /* arm_interrupt_save_ctx(tmp_buf, context) */
FUNC_START(.text, arm_interrupt_save_ctx)
        add    r1, r1, #CPU_ARM_CONTEXT_GPR(2)
        /* move r1 to &r2 in order to save r2-r12 */
        stmia  r1, {r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}
        /* stmia is done, move back r1 to &cpu_context_s */
        sub    r1, r1, #CPU_ARM_CONTEXT_GPR(0)

        /* say what was saved */

        mov    r2, #CPU_ARM_CONTEXT_RESTORE_CALLEE | CPU_ARM_CONTEXT_RESTORE_CALLER
        /* reload r0, r1, r15, cpsr, lr(super) */
        ldmia  r0, {r3, r4, r5, r6, r7}
.if (CPU_ARM_CONTEXT_SAVE_MASK != 0) || (CPU_ARM_CONTEXT_GPR(0) != 4)
        .error Please update this hack if you change the layout
.endif
        /* save (contiguous) save_mask, r0, r1 */
        stmia  r1, {r2, r3, r4}

        /* save pc (r15) */
        str    r5, [r1, #CPU_ARM_CONTEXT_GPR(15)]
        /* save cpsr */
        str    r6, [r1, #CPU_ARM_CONTEXT_CPSR]

        /* last but not least, we have to save pc and lr, this
           may involve shadowed registers */
#if defined(CONFIG_HEXO_USERMODE)
        /* r6 is still cpsr */
        tst    r6, #0xf
        /* if 0 (eq), from user mode */
        bne    1f
        /* from user mode, abuse stmia^ */
        add    r5, r1, #CPU_ARM_CONTEXT_GPR(13)
        stmia  r5, {r13, r14}^
        /* use kernel stack */
        CONTEXT_LOCAL context_stack_end, sp, r3
        bic    sp, sp, #63
        /* return */
        bx     lr
1:
#endif        
        /* no user mode, the only lr is super's, sp is here */
        str    sp, [r1, #CPU_ARM_CONTEXT_GPR(13)]
        str    r7, [r1, #CPU_ARM_CONTEXT_GPR(14)]
        bx     lr
FUNC_END(arm_interrupt_save_ctx)

.macro preempt_clear
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        mov    r7, #0
        SET_GLOBAL cpu_preempt_handler, r7, r8
#endif
.endm
        
.macro handle_exception arg
        preempt_clear
        mov    r0, #\arg
        b arm_exc_common
.endm

.macro handle_irq no
        preempt_clear
        mov    r0, #\no
        b      arm_irq_common
.endm

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

#if !defined(CONFIG_CPU_ARM_CUSTOM_IRQ_HANDLER)
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
        prepare_exception 0
        b      arm_swi_common
FUNC_END(arm_exc_swi)

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
