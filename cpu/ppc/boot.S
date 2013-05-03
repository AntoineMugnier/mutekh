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

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
1:      
        LA32	3,	mutekh_entry
        mtctr   3
        bctr
        .org 0x80 - 4
        b      	1f
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
        /* acceptable cpuid ? */
        cmpwi   cr0,    29,     CONFIG_ARCH_LAST_CPU_ID
        bgt     cr0,            2f

        /* get address of the stacks pool array */
        LA32    2,              cpu_stacks_pool
        lwz     2,              0(2)

        /* get the stack pointer for this cpu */
        slwi    3,      29,     2
        add     3,      2,      3
        lwz     1,              0(3)
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

