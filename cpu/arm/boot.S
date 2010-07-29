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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009-2010
*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

//.syntax unified

FUNC_START(.text, arm_boot)
        CPU_ID r4
        cmp    r4, # CONFIG_CPU_MAXCOUNT
        bgt    cpu_boot_idle

        ldr.w  r0, =__initial_stack

        /* adjust stack pointer, 64 bits aligned */
        bic    r0, #7
        mov    sp, r0

        /* Force the hardware exception handler to align stack */
/*       ldr    r1, .nvic
        ldr    r0, [r1]
        orr    r0, r0, #0x200
        str    r0, [r1]
*/
        ldr.w  r12, =boot_from_reset_vector
        bx     r12

.nvic:  .word 0xE000ED14
FUNC_END(arm_boot)

FUNC_START(.text, cpu_boot_idle)
        .thumb_func
        mov   r0, #0
        CPU_YIELD r0
        b     cpu_boot_idle
FUNC_END(cpu_boot_idle)

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
