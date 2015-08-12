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

#include <cpu/arm_v7m.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

.syntax unified

.section .excep,"ax"
#ifndef CONFIG_LOAD_HEXO_BOOTLOADER

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
#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
        .word arm_exc_hardfault /* hardfault */
        .word arm_exc_memmanage /* memmanage */
        .word arm_exc_busfault /* busfault */
        .word arm_exc_usagefault /* usagefault */
#else
        .word arm_exc_fault /* hardfault */
        .irp r, 4, 5, 6,
         .word arm_exc_dummy
        .endr
#endif
        /* reserved */
        .irp r, 7, 8, 9, 10
          .word arm_exc_dummy
        .endr

        /* syscall */
#if defined(CONFIG_HEXO_BOOTLOADER) || defined(CONFIG_HEXO_USERMODE)
        .word arm_exc_svcall
#else
        .word arm_exc_dummy
#endif
        
        /* debug monitor (v7m only)*/
        .word arm_exc_dummy

        /* reserved */
        .word arm_exc_dummy

        /* pendsv */
#if defined(CONFIG_HEXO_BOOTLOADER)
        .word arm_exc_pendsv
#elif defined(CONFIG_HEXO_CONTEXT_PREEMPT)
        .word cpu_context_jumpto_irq
#else
        .word arm_exc_dummy
#endif

#ifdef CONFIG_HEXO_IRQ
        /* systick */
        .word arm_exc_irq

        /* external irqs */
        .rept CONFIG_CPU_ARM32M_M_IRQ_COUNT
         .word arm_exc_irq
        .endr
#else
        .word arm_exc_dummy
        .rept CONFIG_CPU_ARM32M_M_IRQ_COUNT
         .word arm_exc_dummy
        .endr        
#endif
#else /* CONFIG_LOAD_HEXO_BOOTLOADER */
/*
    We are bootloaded firmware, we do not need usual exception
	decoding, all we have to declare is a structure with various entry
	points. Bootloader will peek in this structure to forward relevant
	exceptions to us.
*/

CPU_NAME_DECL(exception_info):
        .globl CPU_NAME_DECL(exception_info)
        .word 0xa78f8d79               /* magic */
		.word mutekh_entry             /* entry point */
        .word CONFIG_STARTUP_STACK_ADDR
        .word CONFIG_STARTUP_STACK_SIZE
		.word arm_exc_irq
		.word arm_exc_fault
#ifdef CONFIG_HEXO_USERMODE
		.word arm_exc_svcall
#else
        .word 0
#endif
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
		.word cpu_context_jumpto_irq
#else
        .word 0
#endif
        .size CPU_NAME_DECL(exception_info), .-CPU_NAME_DECL(exception_info)

#endif

FUNC_START(.text, arm_exc_dummy)
1:      b 1b
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

#if !defined(CONFIG_HEXO_BOOTLOADER)
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
        add     r1,     r2

        b       arm_exc_preempt
# else
        /* return */
        ldr     r1,     = 0xfffffff9
        bx      r1
# endif
FUNC_END(arm_exc_irq)
#endif


/********************************************************** fault */

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
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
#endif

FUNC_START(.text, arm_exc_fault)

        ARM_EXC_PREEMPT_CLEAR

        ldr     r0,     = cpu_exception_handler
        ldr     r0,     [r0]
        mov     r12,    r0

        /* tls */
        mrs     r3,     psp

        /* fourth arg is cpu_context_s */
        ldr     r0,     = arm_context_regs
        add     r3,     r0

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
        str     r1,     [r3, #CPU_ARM_CONTEXT_CFSR]
#endif

        /* second arg is pc */
        ldr     r1,     [sp, #20]

        str     r1,     [r3, #CPU_ARM_CONTEXT_PC]

        /* fifth arg is stack pointer */
        mov     r0,     r13
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
        add     sp,     #4

        mrs     r1,     psp
        ldr     r2,     = arm_context_regs
        add     r1,     r2

        ldr     r3,     [r1, #CPU_ARM_CONTEXT_PC]
        str     r3,     [sp, #20]

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        /* context registers save array from tls */
        adds    r1,     #16  /* skip r0 - r3 */
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
        ldr     r2,     = cpu_preempt_handler
        ldr     r0,     [r2]
        tst     r0,     r0
        beq     2f

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

#else /* CONFIG_HEXO_BOOTLOADER */

/*
    Trampolines when build acts as a bootloader. All IRQs are
	forwarded to loaded firmware.
*/

FUNC_START(.text, arm_exc_irq)
        ldr     r0,     = CONFIG_HEXO_BOOTLOADER_DESC_ADDRESS
        ldr     r0,     [r0, #16]
        bx      r0
FUNC_END(arm_exc_irq)

FUNC_START(.text, arm_exc_fault)
        ldr     r0,     = CONFIG_HEXO_BOOTLOADER_DESC_ADDRESS
        ldr     r0,     [r0, #20]
        bx      r0
FUNC_END(arm_exc_fault)

FUNC_START(.text, arm_exc_svcall)
        ldr     r0,     = CONFIG_HEXO_BOOTLOADER_DESC_ADDRESS
        ldr     r0,     [r0, #24]
        bx      r0
FUNC_END(arm_exc_svcall)

FUNC_START(.text, arm_exc_pendsv)
        ldr     r0,     = CONFIG_HEXO_BOOTLOADER_DESC_ADDRESS
        ldr     r0,     [r0, #28]
        bx      r0
FUNC_END(arm_exc_pendsv)

#endif

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
