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

*/

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_ARCH_SMP
# include <device/class/cpu.h>
#endif

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
1:      
        LA32	3,	mutekh_entry
        mtctr   3
        bctr
        .org 0x80 - 4
        b      	1b
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section .smpreset,"ax",@progbits
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
        LA32	3,	mutekh_entry
        mtctr   3
        bctr
#endif

FUNC_START(.text, mutekh_entry)

        /* clear status */
        li      3,      0
        mtmsr   3

        /* set cpu interrupts entry point */
        LA32    3,              CPU_NAME_DECL(exception_vector)
        mtevpr  3

#ifdef CONFIG_ARCH_SMP_CAPABLE

        /* get CPU id */
        mfdcr   29,             0

        /* bootstrap processor ? */
        cmpwi   cr0,    29,     CONFIG_ARCH_BOOTSTRAP_CPU_ID
        beq     cr0,            1f

# ifdef CONFIG_ARCH_SMP

        /* get cpu tree root */
        LA32    5,              cpu_tree
        lwz     5,              0(5)

        /* get xor scrambled cpu id */
        LI32    2,              CPU_TREE_XOR_VALUE
        xor     4,      2,      29

4:
        /* not in tree */
        cmpwi   5,      0
        beq     2f

        /* get node cpu id */
        lwz     3,              _offsetof(cpu_tree_s, cpu_id)(5)
        xor     3,      2,      3

        /* matching node */
        cmpw    4,      3
        beq     3f

        /* compare and get next child */
        subfc   6,      4,      3
        subfe   6,      6,      6
        neg     6,      6

        slwi    6,      6,      2
        add     6,      6,      5
        lwz     5,              _offsetof(cpu_tree_s, childs)(6)

        b       4b

3:
        /* get address of the cpu stack */        
        lwz     1,              _offsetof(cpu_tree_s, stack)(5)
        LI32    2,              (CONFIG_HEXO_CPU_STACK_SIZE \
                              - CONFIG_HEXO_STACK_ALIGN)
        add     1,      1,      2

        /* jump to C code */
        LA32    2,              mutekh_startup_smp
        mtctr   2
        bctrl

# endif
2:
# ifdef CONFIG_CPU_PPC_WAIT_OPCODE
        wait
# endif
        b       2b
1:
#endif

        /* set startup stack */
        LI32    1       CONFIG_STARTUP_STACK_ADDR \
                      + CONFIG_STARTUP_STACK_SIZE \
                      - CONFIG_HEXO_STACK_ALIGN

        /* jump to C code */
        LA32    2,              mutekh_startup
        mtctr   2
        bctrl

FUNC_END(mutekh_entry)

