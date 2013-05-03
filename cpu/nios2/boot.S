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

        /* acceptable cpuid ? */
        movia   r3,             CONFIG_ARCH_LAST_CPU_ID
        bgt     r16,    r3,     2f

        /* get address of the stacks pool array */
        movia   r3,             cpu_stacks_pool
        ldw     r3,             (r3)

        /* get the stack pointer for this cpu */
        slli    r4,     r16,    2
        add     r3,     r4,     r3
        ldw     sp,             (r3)
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

