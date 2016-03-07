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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2011

*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        xor     r0,     r0,     r0
        seta    r1,     mutekh_entry
        b       r1
#endif

FUNC_START(.text, mutekh_entry)

        /* set exception base pointer */
        seta    r1,     CPU_NAME_DECL(exception_vector)
        wcsr    EBA,    r1

        /* set startup stack pointer */
        seta    sp,     CONFIG_STARTUP_STACK_ADDR \
                      + CONFIG_STARTUP_STACK_SIZE
#ifdef CONFIG_COMPILE_FRAMEPTR
        mv      fp,     sp
#endif

        /* jump to C code */
        bi      mutekh_startup

FUNC_END(mutekh_entry)

