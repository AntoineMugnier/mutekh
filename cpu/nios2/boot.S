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

    Copyright Francois Charot <charot@irisa.fr>  (c) 2008
    INRIA Rennes Bretagne Atlantique

    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2011

*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_ARCH_SMP
# include <device/class/cpu.h>
#endif

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        movia   r16,            mutekh_entry
        jmp     r16
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section .smpreset,"ax",@progbits
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
        movia   r16,            mutekh_entry
        jmp     r16
#endif


FUNC_START(.text, mutekh_entry)

    .set noat

        /* clear status */
        wrctl   status,         r0

#ifdef CONFIG_CPU_NIOS2_EXEPT_BASE_CTL17
        /* set exception base pointer */
        movia   r4,             CPU_NAME_DECL(exception_vector)
        wrctl   ctl17,          r4
#endif

#ifdef CONFIG_ARCH_SMP_CAPABLE

        /* get cpu id */
        rdctl   r16,            cpuid
        andi    r16,    r16,    0x000003ff

        /* bootstrap processor ? */
        movia   r3,             CONFIG_ARCH_BOOTSTRAP_CPU_ID
        beq     r16,    r3,     1f

# ifdef CONFIG_ARCH_SMP

        /* get cpu tree root */
        movia   r5,     cpu_tree
        ldw     r5,     (r5)

        /* get xor scrambled cpu id */
        movia   r2,     CPU_TREE_XOR_VALUE
        xor     r4,     r2,     r16

4:
        /* not in tree */
        beq     zero,   r5,     2f

        /* get node cpu id */
        ldw     r3,     _offsetof(cpu_tree_s, cpu_id)(r5)
        xor     r3,     r2,     r3

        /* matching node */
        beq     r4,     r3,     3f

        /* compare and get next child */
        cmpltu  r6,     r3,     r4
        slli    r6,     r6,     2
        add     r6,     r6,     r5
        ldw     r5,     _offsetof(cpu_tree_s, childs)(r6)

        br      4b
3:
        
        /* get address of the cpu stack */
        ldw     sp,             _offsetof(cpu_tree_s, stack)(r5)
        movia   r3,             CONFIG_HEXO_CPU_STACK_SIZE
        add     sp,     sp,     r3
#  ifdef CONFIG_COMPILE_FRAMEPTR
        mov     fp,     sp
#  endif

        /* jump to C code */
        movia   r16,             mutekh_startup_smp
        jmp     r16

# endif
2:
        /* spin if this cpuid is not permitted */
        br      2b
1:
#endif

        /* set startup stack pointer */
        movia   sp,     CONFIG_STARTUP_STACK_ADDR \
                      + CONFIG_STARTUP_STACK_SIZE
#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     fp,     sp
#endif

        /* jump to C code */
        movia   r16,             mutekh_startup
        jmp     r16

FUNC_END(mutekh_entry)

