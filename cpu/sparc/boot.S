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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech
*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        set     mutekh_entry, %g1
        jmp     %g1
        nop
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section .smpreset,"ax",@progbits
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
        set     mutekh_entry, %g1
        jmp     %g1
        nop
#endif

FUNC_START(.text, mutekh_entry)

        /* set exception base pointer */
        set     CPU_NAME_DECL(exception_vector), %g1
        mov     %g1,    %tbr

        /* set wim and psr. window -1 is reserved for traps */
        wr      %g0, SPARC_PSR_SUSER_MODE \
                   | SPARC_PSR_PIL_MASK   \
                   | SPARC_PSR_TRAP_ENABLED, %psr
        
        wr      %g0, 0, %wim

#ifdef CONFIG_ARCH_SMP_CAPABLE

        CPU_ID  %g3

        /* bootstrap processor ? */
        cmp     %g3,    CONFIG_ARCH_BOOTSTRAP_CPU_ID
        be      1f
        nop

# ifdef CONFIG_ARCH_SMP
        /* acceptable cpuid ? */
        cmp     %g3,    CONFIG_ARCH_LAST_CPU_ID
        bg      2f
        nop

        /* get address of the stacks pool array */
        set     cpu_stacks_pool, %g1
        ld      [%g1],  %g2

        /* get the stack pointer for this cpu */
        sll     %g3,     2,     %g1
        ld      [%g2 + %g1],  %sp
        set     CONFIG_HEXO_CPU_STACK_SIZE, %g2
        add     %sp,    %g2,    %sp
#  ifdef CONFIG_COMPILE_FRAMEPTR
        mov     %sp,            %i7
#  endif

        /* jump to C code */
        set     mutekh_startup_smp, %g1
        jmp     %g1
        nop

# endif
2:
        /* spin if this cpuid is not permitted */
        WAIT
        b               2b
        nop
1:
#endif

        /* set startup stack pointer */
        set     CONFIG_STARTUP_STACK_ADDR \
              + CONFIG_STARTUP_STACK_SIZE,        %sp

#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     %sp,            %i7
#endif

        /* jump to C code */
        set     mutekh_startup, %g1
        jmp     %g1
        nop

FUNC_END(mutekh_startup)

