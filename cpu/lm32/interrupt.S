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
1:      
        bi      1b
        nop
        nop
        nop
        nop
        nop
        nop
        nop

        /* iberr*/
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        
        /* watchpoint */
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop

        /* dberr */
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop

        /* div by 0 */
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop

        /* irq */
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop

        /* syscall */
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop

.section .text,"ax",@progbits

// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

