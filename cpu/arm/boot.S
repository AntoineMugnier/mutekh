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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009-2010
*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax"
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        ldr     pc, =mutekh_entry
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section .smpreset,"ax"
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
        ldr     pc, =mutekh_entry
#endif

FUNC_START(.text, mutekh_entry)

#ifdef CONFIG_CPU_EXCEPTION_RELOCATABLE
        /* set exception base pointer */
        ldr    r4, = CPU_NAME_DECL(exception_vector)
        mcr    p15, 0, r4, c12, c0, 0    /* VBAR register */
#endif

#ifdef CONFIG_ARCH_SMP_CAPABLE

        CPU_ID r4

        /* bootstrap processor ? */
        cmp    r4, # CONFIG_ARCH_BOOTSTRAP_CPU_ID
        beq    1f

# ifdef CONFIG_ARCH_SMP
        /* acceptable cpuid ? */
        cmp    r4, # CONFIG_ARCH_LAST_CPU_ID
        bgt    2f

        /* get address of the stacks pool array */
        ldr  r0, = cpu_stacks_pool
        ldr  r0, [r0]

        /* get the stack pointer for this cpu */
        lsl  r1, r4, #2
        add  r0, r0, r1
        ldr  r13, [r0]
        ldr  r0, = CONFIG_HEXO_CPU_STACK_SIZE
        add  r13, r0, r13
#  ifdef CONFIG_COMPILE_FRAMEPTR
        mov  r11, r13
#  endif

        ldr  r12, =mutekh_startup_smp
        bx   r12

# endif
2:
        /* spin if this cpuid is not permitted */
        mov   r0, #0
        CPU_YIELD r0
        b   2b
1:
#endif

        /* set startup stack pointer */
        ldr  r13, =CONFIG_STARTUP_STACK_ADDR \
                 + CONFIG_STARTUP_STACK_SIZE
#ifdef CONFIG_COMPILE_FRAMEPTR
        mov  r11, r13
#endif

        ldr  r12, =mutekh_startup
        bx   r12

FUNC_END(mutekh_entry)

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
