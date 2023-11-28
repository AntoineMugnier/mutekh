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

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_LOAD_RESET_SEPARATE
# error
#endif

FUNC_START(.text, mutekh_entry)

#ifdef CONFIG_CPU_EXCEPTION_RELOCATABLE
        /* set exception base pointer */
        ldr     r4, = CPU_NAME_DECL(exception_vector)
        ldr     r1, = 0xe000ed08 /* VTOR */
        str     r4, [r1]
#endif

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
        mov  r4, #0x0
        msr  control, r4
#endif

#ifdef CONFIG_CPU_ARM32M_BARE_BOOT
        ldr     r0, = CONFIG_STARTUP_STACK_ADDR + CONFIG_STARTUP_STACK_SIZE
        mov     r13, r0
#endif

#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     r11, r13
#endif

        /* disable irqs */
        cpsid   i

        ldr  r4, =mutekh_startup
        bx   r4

FUNC_END(mutekh_entry)

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
