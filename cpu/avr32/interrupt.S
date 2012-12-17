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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012
*/

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>

.section .excep,"ax",@progbits

.macro AVR32_EXCEPT_ENTRY id

        // save r1 & r2

        // store excep id

        // goto asm handler

.endm

__exception_base_ptr:
        .globl __exception_base_ptr
        nop
      /* reset */

        /* breakpoint */

        /* iberr*/

        /* watchpoint */

        /* dberr */

        /* div by 0 */

        /* irq */

#ifdef CONFIG_HEXO_IRQ
#else
#endif

        /* syscall */

FUNC_START(.text, avr32_excep_entry)

        // save caller saved & r11, ra (r1 & r2 already saved)

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // clear context preempt handler

#endif

        // save PC

        // call fault handler

        // readback PC

FUNC_END(avr32_excep_entry)

#ifdef CONFIG_HEXO_IRQ
FUNC_START(.text, avr32_irq_entry)

        // save caller saved regs & r11, ra

# ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // clear context preempt handler

# endif

        // find lowest irq number

        // call irq handler

        // acknowledge lowest pending irq

FUNC_END(avr32_irq_entry)
#endif

FUNC_START(.text, avr32_restore_preempt)

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT

        // check if a preempt handler has been registered

        // call preempt handler
        
        // got pointer to context to switch to ?

        // save callee saved registers

        // save PC

        // save irq status

        // write save/restore mask

# ifdef CONFIG_HEXO_CONTEXT_STATS
        // update context stats

# endif
        // switch to new context
1:      
#endif

        // restore regs

        // return

FUNC_END(avr32_restore_preempt)

// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

