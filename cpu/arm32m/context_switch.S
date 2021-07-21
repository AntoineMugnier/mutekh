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
#include <hexo/lock.h>

#include <cpu/arm32m/v7m.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

/*
   Callee saved: r4-r11 r13
   Caller saved: r12
    + Arguments: r0-r3
*/

.syntax unified

#ifdef CONFIG_HEXO_CONTEXT

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


#ifdef CONFIG_CPU_ARM32M_MPU_STACK_GUARD
/* in: r1 = stack top address */
FUNC_START(.text, cpu_context_stack_guard_)

        /* disable guard if stack is not 32 bytes aligned */
        movs    r7,     0
        lsls    r3,     r1,     #27
        bne     2f

        /* mask */
        ldr     r7,     = (1 << ARM_M_STACK_GUARD_MPU_SUBREGION_COUNT) - 1
        uxtb    r3,     r1
        lsrs    r3,     r3,     #5
        lsls    r7,     r7,     r3
2:
        /* MPU base */
        ldr     r5,     = ARMV7M_MPU_TYPE_ADDR

        /* region index */
        movs    r3,     #0
1:
        /* MPU_RNR = index */
        str     r3,     [r5, #8]
        adds    r3,     r3,     #1

        /* MPU_RBAR */
        lsrs    r6,     r1,     #8
        lsls    r6,     r6,     #8
        str     r6,     [r5, #12]

        /* MPU_RASR:srd = (mask << 8) & 0xff */
        mvns    r6,     r7
        lsls    r6,     r6,     #24
        lsrs    r6,     r6,     #16
        /* MPU_RASR:attr = region enabled, 256 bytes, no access */
        adds    r6,     #15
        str     r6,     [r5, #16]

        /* next region */
        lsrs    r7,     r7,     #8
        adds    r1,     #128
        adds    r1,     #128

        /* regions count */
        cmp     r3,     #ARM_M_STACK_GUARD_MPU_REGION_COUNT
        bcc     1b

        bx      r14

FUNC_END(cpu_context_stack_guard_)

FUNC_START(.text, cpu_context_stack_guard)
        push    {r5, r6, r7, lr}
        mov     r1,     r0
        bl      cpu_context_stack_guard_
        pop     {r5, r6, r7, pc}
FUNC_END(cpu_context_stack_guard)
#endif

/* in:  r0 = struct context_s*
   out: r0 = &arm_context_regs[r12]
        r1 = primask
*/
FUNC_START(.text, cpu_context_jumpto_restore)

#if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
        // unlock some atomic value on context restore if requested
        ldr     r1,     [r0, #_offsetof(context_s, unlock, 4)]
        tst     r1,     r1
        beq     1f

        movs    r2,     #0
        str     r2,     [r0, #_offsetof(context_s, unlock, 4)]
        str     r2,     [r1, #_offsetof(__arch_lock_s, __arch_lock_unlock, 4)]
1:
#endif

        /* restore tls pointer from context struct */
        ldr     r0,     [r0, #(_offsetof(context_s, tls, 4))]
        msr     psp,    r0

#ifdef CONFIG_CPU_ARM32M_MPU_STACK_GUARD
        /* context stack top */
        ldr     r1,     = context_stack_start
        ldr     r1,     [r0, r1]

        mov     r2,     r14
        bl      cpu_context_stack_guard_
        mov     r14,     r2
#endif

#ifdef CONFIG_HEXO_CONTEXT_IRQEN
        // clear irq enable hook
        ldr     r1,     = cpu_irqen_handler
        movs    r2,     #0
        str     r2,     [r1]
#endif

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

        bx      r14

FUNC_END(cpu_context_jumpto_restore)


/* in: r0 = struct context_s* */
FUNC_START(.text, cpu_context_jumpto)

        bl      cpu_context_jumpto_restore

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* test exc_return */
        lsrs    r3,     r1,     #17
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
        ldr     r2, = ARMV7M_ICSR_PENDSVSET
        ldr     r3, = ARMV7M_ICSR_ADDR
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

#endif /* CONFIG_HEXO_CONTEXT */

FUNC_START(.text, cpu_context_set)

	add     r1,     r0,     r1
        mov     r13,    r1

#if defined(CONFIG_CPU_ARM32M_MPU_STACK_GUARD) && \
    defined(CONFIG_HEXO_CONTEXT)
	mov     r1,     r0
        bl      cpu_context_stack_guard_
#endif

	bx      r2

FUNC_END(cpu_context_set)

#ifdef CONFIG_HEXO_CONTEXT

FUNC_START(.text, cpu_context_entry)
        mov     r0,     r5
        bx      r4
FUNC_END(cpu_context_entry)


FUNC_START(.text, cpu_context_stack_use)

        mov     r4,     r1

        /* get tls pointer from context struct */
        ldr     r0,     [r0, #(_offsetof(context_s, tls, 4))]

#ifdef CONFIG_CPU_ARM32M_MPU_STACK_GUARD
        /* context stack top */
        ldr     r1,     = context_stack_start
        ldr     r1,     [r0, r1]

        bl      cpu_context_stack_guard_
#endif

        /* context registers save array */
        ldr     r1,     = arm_context_regs
        add     r0,     r1

        /* get context sp */
        ldr     r0,     [r0, #CPU_ARM_CONTEXT_SP]
        mov     r0,     r13

        /* function call */
        mov     r0,     r2
        bx      r4

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

#endif /* CONFIG_HEXO_CONTEXT */
