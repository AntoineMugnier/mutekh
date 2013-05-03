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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012

*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        lddpc   r0, .entry
        ijmp    r0
.entry: .int    mutekh_entry
#endif

FUNC_START(.text, mutekh_entry)

        /* setup stack */
        lddpc   sp, .is

#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     r7, sp
#endif

        /* set exception entry point */
        lddpc   r0, .ex
        mtsr    4, r0     // EVBA register

        /* jump to C code */
        lddpc   r0, .start
        icall   r0

        /* idle state */
1:      sleep   0
        rjmp    1b

.is:    .int CONFIG_STARTUP_STACK_ADDR + CONFIG_STARTUP_STACK_SIZE
.ex:    .int CPU_NAME_DECL(exception_vector)
.start: .int mutekh_startup

FUNC_END(mutekh_entry)

