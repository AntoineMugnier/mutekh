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

#include <cpu/arm32m/v7m.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

.syntax unified

.section .excep,"ax"

CPU_NAME_DECL(exception_vector):
        .globl CPU_NAME_DECL(exception_vector)
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)

#ifdef CONFIG_CPU_ARM32M_BARE_BOOT
        .2byte 0x4800   /* ldr r0, = mutekh_entry */
        .2byte 0x4700   /* bx r0 */
#else
        .word CONFIG_STARTUP_STACK_ADDR + CONFIG_STARTUP_STACK_SIZE
#endif
        /* reset */
        .word mutekh_entry

        /* nmi */
        .org CPU_NAME_DECL(exception_vector) + 2 * 4
        .word arm_exc_dummy

        /* faults */
        .org CPU_NAME_DECL(exception_vector) + 3 * 4
#ifdef CONFIG_HEXO_EXCEP
# if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
        .word arm_exc_hardfault /* hardfault */
        .word arm_exc_memmanage /* memmanage */
        .word arm_exc_busfault /* busfault */
        .word arm_exc_usagefault /* usagefault */
# else
        .word arm_exc_fault /* hardfault */
# endif

#else /* !CONFIG_HEXO_EXCEP */
        .irp r, 3, 4, 5, 6
         .word arm_exc_dummy
        .endr
#endif

        /* syscall */
#ifdef CONFIG_HEXO_USERMODE
        .org CPU_NAME_DECL(exception_vector) + 11 * 4
        .word arm_exc_svcall
#endif

        /* pendsv */
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        .org CPU_NAME_DECL(exception_vector) + 14 * 4
        .word cpu_context_jumpto_irq
#endif

#ifdef CONFIG_HEXO_IRQ
        .org CPU_NAME_DECL(exception_vector) + 15 * 4
        /* systick */
        .word arm_exc_irq

        /* external irqs */
        .rept CONFIG_CPU_ARM32M_M_IRQ_COUNT
         .word arm_exc_irq
        .endr
#endif

FUNC_START(.text, arm_exc_dummy)
1:      wfi
        b 1b
FUNC_END(arm_exc_dummy)

/***********************************************************/

.macro  ARM_EXC_PREEMPT_CLEAR
# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* clear context preempt handler */
        ldr     r0,     = cpu_preempt_handler
        movs    r3,     #0
        str     r3,     [r0]
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
        b       arm_exc_preempt
# else
        /* return */
        ldr     r1,     = 0xfffffff9
        bx      r1
# endif
FUNC_END(arm_exc_irq)
#endif


/********************************************************** fault */

#ifdef CONFIG_HEXO_EXCEP
# if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
FUNC_START(.text, arm_exc_hardfault)
        ldr     r1,     = ARMV7M_HFSR_ADDR
        ldr     r1,     [r1]

        /* third arg, dataptr */
        mvn     r2,     #0

        b       arm_exc_fault
FUNC_END(arm_exc_hardfault)

FUNC_START(.text, arm_exc_memmanage)
        ldr     r1,     = ARMV7M_CFSR_ADDR
        ldr     r1,     [r1]

        /* third arg, dataptr */
        ldr     r2,     = ARMV7M_MMFAR_ADDR
        ldr     r2,     [r2]

        b       arm_exc_fault
FUNC_END(arm_exc_memmanage)

FUNC_START(.text, arm_exc_busfault)
        ldr     r1,     = ARMV7M_CFSR_ADDR
        ldr     r1,     [r1]

        /* third arg, dataptr */
        ldr     r2,     = ARMV7M_BFAR_ADDR
        ldr     r2,     [r2]

        b       arm_exc_fault
FUNC_END(arm_exc_busfault)

FUNC_START(.text, arm_exc_usagefault)
        ldr     r1,     = ARMV7M_CFSR_ADDR
        ldr     r1,     [r1]

        /* third arg, dataptr */
        mvn     r2,     #0

        b       arm_exc_fault
FUNC_END(arm_exc_usagefault)
# endif

FUNC_START(.text, arm_exc_fault)

        ARM_EXC_PREEMPT_CLEAR

        ldr     r0,     = cpu_exception_handler
        ldr     r0,     [r0]
        mov     r12,    r0

#ifdef CONFIG_HEXO_CONTEXT
        /* tls */
        mrs     r3,     psp

        /* fourth arg is cpu_context_s */
        ldr     r0,     = arm_context_regs
        add     r3,     r0
#else
        /* store cpu_context_s on stack */
        mov     r3,     sp
        adds    r3,     #- _sizeof(struct cpu_context_s)
#endif

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
        str     r1,     [r3, #CPU_ARM_CONTEXT_CFSR]
#endif

        /* store all registers in cpu_context_s */
        ldr     r1,     [sp, #0]
        str     r1,     [r3, #CPU_ARM_CONTEXT_R0]
        ldr     r1,     [sp, #4]
        str     r1,     [r3, #CPU_ARM_CONTEXT_R1]
        ldr     r1,     [sp, #8]
        str     r1,     [r3, #CPU_ARM_CONTEXT_R2]
        ldr     r1,     [sp, #12]
        str     r1,     [r3, #CPU_ARM_CONTEXT_R3]
        str     r4,     [r3, #CPU_ARM_CONTEXT_R4]
        str     r5,     [r3, #CPU_ARM_CONTEXT_R5]
        str     r6,     [r3, #CPU_ARM_CONTEXT_R6]
        str     r7,     [r3, #CPU_ARM_CONTEXT_R7]
#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
        str     r8,     [r3, #CPU_ARM_CONTEXT_R8]
        str     r9,     [r3, #CPU_ARM_CONTEXT_R9]
        str     r10,    [r3, #CPU_ARM_CONTEXT_R10]
        str     r11,    [r3, #CPU_ARM_CONTEXT_R11]
#else
        mov     r1,     r8
        str     r1,     [r3, #CPU_ARM_CONTEXT_R8]
        mov     r1,     r9
        str     r1,     [r3, #CPU_ARM_CONTEXT_R9]
        mov     r1,     r10
        str     r1,     [r3, #CPU_ARM_CONTEXT_R10]
        mov     r1,     r11
        str     r1,     [r3, #CPU_ARM_CONTEXT_R11]
#endif
        ldr     r1,     [sp, #16]
        str     r1,     [r3, #CPU_ARM_CONTEXT_R12]
        ldr     r1,     [sp, #20]
        str     r1,     [r3, #CPU_ARM_CONTEXT_LR]

        /* second arg is pc */
        ldr     r1,     [sp, #24]
        str     r1,     [r3, #CPU_ARM_CONTEXT_PC]

        /* fifth arg is stack pointer */
        mov     r0,     sp
        adds    r0,     0x20     /* skip values pushed by cpu */
#ifndef CONFIG_HEXO_CONTEXT
        /* skip cpu_context_s stored on stack */
        mov     sp,     r3
#endif
        str     r0,     [r3, #CPU_ARM_CONTEXT_SP]
        push    {r0}

        /* first arg is interrupt number */
        mrs     r0,     ipsr
        uxtb    r0,     r0

#if CONFIG_CPU_ARM32M_ARCH_VERSION < 7
        /* third arg, dataptr */
        movs    r2,     #0
        mvns    r2,     r2
#endif

        blx     r12
        /* pop fifth arg */
        add     sp,     #4

#ifdef CONFIG_HEXO_CONTEXT
        mrs     r1,     psp
        ldr     r2,     = arm_context_regs
        add     r1,     r2
#else
        mov     r1,     sp
        /* pop cpu_context_s */
        add     sp,     #_sizeof(struct cpu_context_s)
#endif

        ldr     r3,     [r1, #CPU_ARM_CONTEXT_PC]
        str     r3,     [sp, #20]

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        b       arm_exc_preempt
# else
        /* return */
        ldr     r1,     = 0xfffffff9
        bx      r1
# endif
FUNC_END(arm_exc_fault)
#endif

/********************************************************** syscall */

#ifdef CONFIG_HEXO_USERMODE
FUNC_START(.text, arm_exc_svcall)

        ARM_EXC_PREEMPT_CLEAR

        /* tls */
        mrs     r2,     psp

        /* first arg is syscall number */
        mov     r0,     r13
        ldr     r0,     [r0, #20]    /* get return address */
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
        ldr     r3,     [sp, #20]
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
        str     r3,     [sp, #20]

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
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
FUNC_START(.text, arm_exc_preempt)

        /* check if a preempt handler has been registered */
        ldr     r2,     = cpu_preempt_handler
        ldr     r0,     [r2]
        tst     r0,     r0
        beq     2f

        /* call preempt handler */
        blx     r0

        /* got pointer to context to switch to ? */
        tst     r0,     r0
        beq     1f

        /* context registers save array from tls */
        mrs     r1,     psp
        ldr     r2,     = arm_context_regs /* skip r0 - r3 */ + CPU_ARM_CONTEXT_R4
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

        /* save primask and set ret_code to 1 */
        ldr     r6,     = 0xffff0000
        mrs     r7,     primask
        orrs    r7,     r6
        str     r7,     [r1, #16]

# ifdef CONFIG_HEXO_CONTEXT_STATS
        mov     r4,     r0
        bl      context_preempt_stats
        mov     r0,     r4
# endif

        bl      cpu_context_jumpto_restore

        msr     primask, r1

        /* test ret_code to determine if we a are restoring an interrupted context */
        lsrs    r1,     r1,     #17
        bcs     1f

        /* push exception return data: xpsr and pc, r0-r3,r12,r14 are garbage */
        ldr     r0,     [r0, #12]
        ldr     r1,     = 0x01000000      /* xpsr: stack 8 bytes aligned, exception number is 0 */
        push    {r0, r1}
        sub     sp,     #24
2:
        // invalidate handler outside interrupt
        ldr     r2,     = cpu_preempt_handler
        str     r2,     [r2]
1:
        /* regular return from irq */
        ldr     r1,     = 0xfffffff9
        bx      r1
FUNC_END(arm_exc_preempt)
#endif

