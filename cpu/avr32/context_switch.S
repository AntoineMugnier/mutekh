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
#include <hexo/cpu.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

/*
   AVR32 ABI:

        r0 - r7 : callee saved
        r8 - r12  : caller saved
        r12 : return value
        r13 : stack ptr
        r14 : link reg
        r15 : pc
*/

FUNC_START(.text, cpu_context_switch)

        // context registers save array
        mfsr    r8,     8           // context local storage is ACBA sysreg
        lddpc   r9,     .cregs
        add     r8,     r9

        // save callee saved, fp, sp
        
        // resume directly at return address

        // save irq status

        // write save/restore mask

.cregs: .int avr32_context_regs
FUNC_END(cpu_context_switch)



FUNC_START(.text, cpu_context_jumpto)

#ifdef CONFIG_ARCH_SMP
        // unlock some atomic value on context restore if requested

        // unlock and clear unlock address

1:      
#endif

        // restore tls pointer from context struct
        
        // context registers save array

        // restore callee saved stack pointer

#if 0   // callee should almost always get restored (except on context start)

#endif

        // restore callee saved
1:      

        // prepare eret
        
        // restore caller saved

1:      

FUNC_END(cpu_context_jumpto)



FUNC_START(.text, cpu_context_set)


        // set stack pointer
#ifdef CONFIG_COMPILE_FRAMEPTR

#endif

FUNC_END(cpu_context_set)



FUNC_START(.text, cpu_context_stack_use)

        // get context local storage
        
        // context registers save array

#ifdef CONFIG_COMPILE_FRAMEPTR

#endif

#ifndef CONFIG_RELEASE
        // ensure we do not use context local storage from here
#endif

FUNC_END(cpu_context_stack_use)

