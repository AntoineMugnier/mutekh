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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2011
*/

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>

.section .excep,"ax",@progbits

.macro LM32_EXCEPT_ENTRY id

        // get context registers save array
        ori     r31,    r0,      lo(lm32_context_regs)
        add     r31,    r25,     r31

        // save r1
        sw      (r31 + CPU_LM32_CONTEXT_GPR(1)), r1
        // store excep id
        mvi     r1,     \id

        // goto asm handler
        seta    r31,    lm32_excep_entry
        b       r31

.endm

__exception_base_ptr:

      /* reset */
        xor     r0,     r0,     r0
        seta    r1, cpu_boot
        b       r1
        nop
        nop
        nop
        nop

        /* breakpoint */
        .org __exception_base_ptr + 1*32
        mv      r30,    r31
        LM32_EXCEPT_ENTRY CPU_FAULT_BREAKPOINT

        /* iberr*/
        .org __exception_base_ptr + 2*32
        LM32_EXCEPT_ENTRY CPU_FAULT_INS_BERR

        /* watchpoint */
        .org __exception_base_ptr + 3*32
        mv      r30,    r31
        LM32_EXCEPT_ENTRY CPU_FAULT_WATCHPOINT

        /* dberr */
        .org __exception_base_ptr + 4*32
        LM32_EXCEPT_ENTRY CPU_FAULT_DATA_BERR

        /* div by 0 */
        .org __exception_base_ptr + 5*32
        LM32_EXCEPT_ENTRY CPU_FAULT_DIV_BY_0

        /* irq */
        .org __exception_base_ptr + 6*32

#ifdef CONFIG_HEXO_IRQ
        seta    r31, lm32_irq_entry
        b       r31
#else
        LM32_EXCEPT_ENTRY CPU_FAULT_LOST_IRQ
#endif

        /* syscall */
        .org __exception_base_ptr + 7*32

        LM32_EXCEPT_ENTRY CPU_FAULT_SYSCALL

        .org __exception_base_ptr + 8*32

.macro LM32_EXCEPT_RESTORE
        .irp r, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 27, 28, 29
        lw      r\r, (r31 + CPU_LM32_CONTEXT_GPR(\r))
        .endr

        eret
.endm
       
FUNC_START(.text, lm32_excep_entry)

        // save caller saved, fp, sp (r1 already saved)
        .irp r, 2, 3, 4, 5, 6, 7, 8, 9, 10, 27, 28, 29
        sw      (r31 + CPU_LM32_CONTEXT_GPR(\r)), r\r
        .endr

        // save PC
        sw      (r31 + CPU_LM32_CONTEXT_PC), r30

        // call fault handler
        addi    r2,     r31,    CPU_LM32_CONTEXT_PC
        mv      r3,     r0
        mv      r4,     r31
        mv      r5,     sp

        seta    r10,    cpu_exception_handler
        lw      r10,    (r10 + 0)
        call    r10

        // readback PC
        lw      r30, (r31 + CPU_LM32_CONTEXT_PC)

        // restore regs
        .irp r, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 27, 28, 29
        lw      r\r, (r31 + CPU_LM32_CONTEXT_GPR(\r))
        .endr

        // return
        eret

FUNC_END(lm32_excep_entry)

#ifdef CONFIG_HEXO_IRQ
FUNC_START(.text, lm32_irq_entry)

        // get context registers save array
        ori     r31,    r0,      lo(lm32_context_regs)
        add     r31,    r25,     r31

        // save caller saved regs, fp, sp
        .irp r, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 29
        sw      (r31 + CPU_LM32_CONTEXT_GPR(\r)), r\r
        .endr

        // find lowest irq number
        rcsr    r1,     IP
        mv      r11,    r1
        calli   __ffssi2
        addi    r1,     r1,     -1

        // call irq handler
        seta    r2,     cpu_interrupt_handler
        lw      r2,     (r2 + 0)
        call    r2

        // acknowledge lowest pending irq
        addi    r2,     r11,    -1
        and     r2,     r11,    r2
        xor     r2,     r11,    r2
        wcsr    IP,     r2

        // restore regs
        .irp r, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 29
        lw      r\r, (r31 + CPU_LM32_CONTEXT_GPR(\r))
        .endr

        // return
        eret

FUNC_END(lm32_irq_entry)
#endif

// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

