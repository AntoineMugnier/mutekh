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

#ifdef CONFIG_ARCH_SMP
# include <device/class/cpu.h>
#endif

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

        /* get cpu tree root */
        set     cpu_tree, %g5
        ld      [%g5],   %g5

        /* get xor scrambled cpu id */
        set     CPU_TREE_XOR_VALUE, %g2
        xor     %g3,    %g2,    %g4

4:
        /* not in tree */
        tst     %g5
        be      2f
        nop

        /* get node cpu id and compare */
        ld      [%g5 + _offsetof(cpu_tree_s, cpu_id)], %g3
        xor     %g3,    %g2,    %g3
        cmp     %g3,    %g4

        /* matching node */
        be      3f
        nop

        /* get next child */
        addx    %g0, 0, %g6
        sll     %g6,    2,     %g6
        add     %g6,    %g5,   %g6
        ld      [%g6 + _offsetof(cpu_tree_s, childs)], %g5

        b       4b
        nop

3:      
        /* get address of the cpu stack */
        ld      [%g5 + _offsetof(cpu_tree_s, stack)], %sp
        set     CONFIG_HEXO_CPU_STACK_SIZE - SPARC_STACK_REDZONE, %g2
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
              + CONFIG_STARTUP_STACK_SIZE - SPARC_STACK_REDZONE,        %sp

#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     %sp,            %i7
#endif

        /* jump to C code */
        set     mutekh_startup, %g1
        jmp     %g1
        nop

FUNC_END(mutekh_entry)

