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
  
   Copyright (c) 2013, Alexandre Becoulet <alexandre.becoulet@free.fr>
*/

#include <hexo/asm.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>
#include <hexo/context.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

.section .excep,"ax"

CPU_NAME_DECL(exception_vector):
        .globl CPU_NAME_DECL(exception_vector)
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)

        .word CONFIG_STARTUP_STACK_ADDR + CONFIG_STARTUP_STACK_SIZE

        /* reset */
        .word mutekh_entry

        /* nmi */
        .word arm_exc_dummy

        /* faults */
        .word arm_exc_fault /* hardfault */
#if CONFIG_CPU_ARM_ARCH_VERSION >= 7
        .word arm_exc_fault /* memmanage */
        .word arm_exc_fault /* busfault */
        .word arm_exc_fault /* usagefault */
#else
        .irp r, 4, 5, 6,
         .word arm_exc_dummy
        .endr
#endif
        /* reserved */
        .irp r, 7, 8, 9, 10
          .word arm_exc_dummy
        .endr

        /* syscall */
#ifdef CONFIG_HEXO_USERMODE
        .word arm_exc_svcall
#else
        .word arm_exc_dummy
#endif
        
        /* debug monitor (v7m only)*/
        .word arm_exc_dummy

        /* reserved */
        .word arm_exc_dummy

        /* pendsv */
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        .word cpu_context_jumpto_irq
#else
        .word arm_exc_dummy
#endif

#ifdef CONFIG_HEXO_IRQ
        /* systick */
        .word arm_exc_irq

        /* external irqs */
        .rept CONFIG_CPU_ARM_M_IRQ_COUNT
         .word arm_exc_irq
        .endr
#else
        .word arm_exc_dummy
        .rept CONFIG_CPU_ARM_M_IRQ_COUNT
         .word arm_exc_dummy
        .endr        
#endif

FUNC_START(.text, arm_exc_dummy)
1:      b 1b
FUNC_END(arm_exc_dummy)

/***********************************************************/

.macro  ARM_EXC_PREEMPT_CLEAR
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* clear context preempt handler */
        ldr     r0,     = cpu_preempt_handler
        mov     r1,     #0
        str     r1,     [r0]
# endif
.endm

/********************************************************** irq */

#ifdef CONFIG_HEXO_IRQ
FUNC_START(.text, arm_exc_irq)

        ARM_EXC_PREEMPT_CLEAR

        /* first arg is interrupt number */
        mrs     r0,     ipsr
        uxtb    r0,     r0

        /* call interrupt handler */
        ldr     r1,     = cpu_interrupt_handler
        ldr     r1,     [r1]
        blx     r1

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* context registers save array from tls */
        mrs     r1,     psp
        ldr     r2,     = arm_context_regs /* skip r0 - r3 */ + 16
        add     r2,     r1

        b       arm_exc_preempt
# else
        /* return */
        ldr     r1,     = 0xfffffff9
        bx      r1
# endif
FUNC_END(arm_exc_irq)
#endif


/********************************************************** fault */

FUNC_START(.text, arm_exc_fault)

        ARM_EXC_PREEMPT_CLEAR

        ldr     r1,     = cpu_exception_handler
        ldr     r1,     [r1]
        mov     r12,    r1

        /* tls */
        mrs     r3,     psp

        /* first arg is interrupt number */
        mrs     r0,     ipsr
        uxtb    r0,     r0

        /* second arg is pc */
        ldr     r1,     [sp, #24]

        /* fourth arg is cpu_context_s */
        ldr     r2,     = arm_context_regs
        add     r3,     r2

        str     r1,     [r3, #CPU_ARM_CONTEXT_PC]

        /* arg4 is stack pointer */
        mov     r2,     r13
        push    {r2}

        /* third arg, dataptr */
        mov     r2,     #0

        blx     r12
        add     sp,     #4

        mrs     r1,     psp
        ldr     r2,     = arm_context_regs
        add     r2,     r1

        ldr     r3,     [r1, #CPU_ARM_CONTEXT_PC]
        str     r3,     [sp, #24]

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* context registers save array from tls */
        add     r1,     #16  /* skip r0 - r3 */
        b       arm_exc_preempt
# else
        /* return */
        ldr     r1,     = 0xfffffff9
        bx      r1
# endif
FUNC_END(arm_exc_fault)


/********************************************************** syscall */

#ifdef CONFIG_HEXO_USERMODE
FUNC_START(.text, arm_exc_svcall)

        ARM_EXC_PREEMPT_CLEAR

        /* tls */
        mrs     r2,     psp

        /* first arg is syscall number */
        mov     r0,     r13
        ldr     r0,     [r0, #24]    /* get return address */
        sub     r0,     #2           /* get svc instruction opcode */
        ldrh    r0,     [r0]
        uxtb    r0,     r0           /* extract imm8 value */

        /* second arg is cpu_context_s */
        ldr     r1,     = arm_context_regs
        add     r1,     r2

        /* copy args from r0 - r3 on stack to cpu_context_s */
        ldr     r3,     [sp, #0]
        str     r3,     [r1, #CPU_ARM_CONTEXT_R0]
        ldr     r3,     [sp, #4]
        str     r3,     [r1, #CPU_ARM_CONTEXT_R1]
        ldr     r3,     [sp, #8]
        str     r3,     [r1, #CPU_ARM_CONTEXT_R2]
        ldr     r3,     [sp, #12]
        str     r3,     [r1, #CPU_ARM_CONTEXT_R3]
        ldr     r3,     [sp, #24]
        str     r3,     [r1, #CPU_ARM_CONTEXT_PC]

        /* call interrupt handler */
        push    {r1}
        ldr     r3,     = cpu_syscall_handler
        add     r3,     r2
        ldr     r3,     [r3]
        blx     r3
        pop     {r1}

        /* copy return value and pc from cpu_context_s to stack */
        ldr     r3,     [r1, #CPU_ARM_CONTEXT_R0]
        str     r3,     [sp, #0]
        ldr     r3,     [r1, #CPU_ARM_CONTEXT_PC]
        str     r3,     [sp, #24]

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        add     r1,     #16  /* skip r0 - r3 */
        b       arm_exc_preempt
# else
        /* return */
        ldr     r1,     = 0xfffffff9
        bx      r1
# endif
FUNC_END(arm_exc_svcall)
#endif
        
/********************************************************** restore with preempt */

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
FUNC_START(.text, arm_exc_preempt)        /* args: r1 = &arm_context_regs + 16 */

        /* check if a preempt handler has been registered */
        ldr     r0,     = cpu_preempt_handler
        ldr     r0,     [r0]
        tst     r0,     r0
        beq     1f

        /* call preempt handler */
        push    {r1}
        blx     r0
        pop     {r1}

        /* got pointer to context to switch to ? */
        tst     r0,     r0
        beq     1f

        /* save r4 - r11 */
        stmia   r1!,    {r4,r5,r6,r7}
        mov     r4,     r8
        mov     r5,     r9
        mov     r6,     r10
        mov     r7,     r11
        stmia   r1!,    {r4,r5,r6,r7}
        
        /* save sp */
        mov     r7,     r13
        str     r7,     [r1, #4]

        /* save primask and set ret_code to 1 */
        ldr     r6,     = 0xffff0000
        mrs     r7,     primask
        orr     r7,     r6
        str     r7,     [r1, #16]

        /* restore tls pointer from context struct */
        ldr     r0,     [r0, #(HEXO_CONTEXT_S_TLS * 4)]
        msr     psp,    r0

        /* context registers save array */
        ldr     r1,     = arm_context_regs /* skip r0 - r3 */ + 16
        add     r0,     r1

        /* restore callee saved regs */
        ldmia   r0!,    {r4, r5, r6, r7}
        ldmia   r0!,    {r1, r2}
        mov     r8,     r1
        mov     r9,     r2
        ldmia   r0!,    {r1, r2}
        mov     r10,    r1
        mov     r11,    r2

        /* restore sp */
        ldr     r3,     [r0, #4]
        mov     r13,    r3

        /* restore primask and get ret_code */
        ldr     r1,     [r0, #16]
        msr     primask, r1

        /* test ret_code to determine if we a are restoring an interrupted context */
        lsr     r1,     #17
        bcs     1f

        /* push exception return data: xpsr and pc, r0-r3,r12,r14 are garbage */
        ldr     r0,     [r0, #12]
        ldr     r1,     = 0x01000000      /* xpsr: stack 8 bytes aligned, exception number is 0 */
        push    {r0, r1}
        sub     sp,     #24

1:
        /* regular return from irq */
        ldr     r1,     = 0xfffffff9
        bx      r1
FUNC_END(arm_exc_preempt)
#endif

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
