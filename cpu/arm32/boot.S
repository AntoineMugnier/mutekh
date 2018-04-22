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

#ifdef CONFIG_ARCH_SMP
# include <device/class/cpu.h>
#endif

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax"
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
# ifdef CONFIG_CPU_ARM32_BIG_BE8
        setend  be
# endif
        ldr     pc, =mutekh_entry
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section .smpreset,"ax"
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
# ifdef CONFIG_CPU_ARM32_BIG_BE8
        setend  be
# endif
        ldr     pc, =mutekh_entry
#endif

FUNC_START(.text, mutekh_entry)

#ifdef CONFIG_CPU_ARM32_BIG_BE8
        setend  be
#endif

#ifdef CONFIG_CPU_EXCEPTION_RELOCATABLE
        /* set exception base pointer */
        ldr    r4, = CPU_NAME_DECL(exception_vector)
        mcr    p15, 0, r4, c12, c0, 0    /* VBAR register */
#endif

#if CONFIG_CPU_ARM32_ARCH_VERSION == 6
        /* enable non aligned memory access on ARMv6 */
        mrc    p15, 0, r4, c1, c0, 0 /* system control register */
        orr    r4, r4, #(1 << 22)    /* set bit U */
        bic    r4, r4, #(1 << 1)     /* clear bit A */
        mcr    p15, 0, r4, c1, c0, 0
#endif

#ifdef CONFIG_ARCH_SMP_CAPABLE

        CPU_ID r4

        /* bootstrap processor ? */
        cmp    r4, # CONFIG_ARCH_BOOTSTRAP_CPU_ID
        beq    1f

# ifdef CONFIG_ARCH_SMP

        /* get cpu tree root */
        ldr  r5, =cpu_tree
        ldr  r5, [r5]

        ldr  r2, =CPU_TREE_XOR_VALUE
        eor  r4, r2, r4
        
4:
        /* not in tree */
        cmp  r5, #0
        beq  2f

        /* get node cpu id */
        ldr  r3, [r5, #_offsetof(cpu_tree_s, cpu_id)]
        eor  r3, r2, r3

        /* matching node */
        cmp  r3, r4
        beq  3f

        /* get next child */
        movhi r6, #0
        movls r6, #4
        add  r6, r5, r6
        ldr  r5, [r6, #_offsetof(cpu_tree_s, childs)]

        b   4b

3:      
        /* get address of the cpu stack */
        ldr  r13, [r5, #_offsetof(cpu_tree_s, stack)]
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
# if CONFIG_CPU_ARM32_ARCH_VERSION >= 6
        mcr p15, 0, r0, c7, c0, 4
# endif
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
