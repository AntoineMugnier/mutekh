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

    Copyright (c) 2013, Alexandre Becoulet <alexandre.becoulet@free.fr>
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

        /* context registers save array from tls */
        mrs     r1,     psp
        ldr     r2,     = arm_context_regs /* skip r0 - r3 */ + 16
        add     r1,     r2

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

        /* save pc */
        mov     r7,     r14
        str     r7,     [r1, #12]

        /* save primask and set ret_code to 0 */
        mrs     r7,     primask
        uxth    r7,     r7
        str     r7,     [r1, #16]

        b       cpu_context_jumpto

FUNC_END(cpu_context_switch)



FUNC_START(.text, cpu_context_jumpto)

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

        /* get primask and exc_return */
        ldr     r1,     [r0, #16]

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* test exc_return */
        mov     r3,     r1
        lsr     r3,     #17
        bcs     1f
#endif

        /* restore primask and jump to pc */
        ldr     r2,     [r0, #12]
        msr     primask, r1
        bx      r2

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
1:
        /* trigger pendsv irq, jump to cpu_context_jumpto_irq.
           pendsv must have a higher priority than other irqs. */
        ldr     r2, = 0x10000000
        ldr     r3, = 0xe000ed04
        str     r2, [r3]
        nop
        cpsie   i
1:      b       1b
#endif

FUNC_END(cpu_context_jumpto)

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
FUNC_START(.text, cpu_context_jumpto_irq)
        /* discard pushed registers so that sp points to the previous set
	   of caller saved registers we actually want to restore from handler mode. */
        add     r13,    #32
        msr     primask, r1

        bx      lr
FUNC_END(cpu_context_jumpto_irq)
#endif



FUNC_START(.text, cpu_context_set)
	add  r1, r0, r1
	mov  r13, r1
	bx   r2
FUNC_END(cpu_context_set)


FUNC_START(.text, cpu_context_entry)
        mov     r0,     r5
        bx      r4
FUNC_END(cpu_context_entry)


FUNC_START(.text, cpu_context_stack_use)

        /* get tls pointer from context struct */
        ldr     r0,     [r0, #(HEXO_CONTEXT_S_TLS * 4)]

        /* context registers save array */
        ldr     r3,     = arm_context_regs
        add     r0,     r3

        /* get context sp */
        ldr     r0,     [r0, #CPU_ARM_CONTEXT_SP]
        mov     r0,     r13

        /* function call */
        mov     r0,     r2
        bx      r1

FUNC_END(cpu_context_stack_use)




#ifdef CONFIG_HEXO_USERMODE

FUNC_START(.text, cpu_context_set_user)

        mov  r4, #0x3
        msr  control, r4

	mov  r13, r0
      	mov  r0, r2
	bx   r1

FUNC_END(cpu_context_set_user)

#endif

