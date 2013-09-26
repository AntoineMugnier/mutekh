/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
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
        la      $8,             mutekh_entry
        j       $8
        nop
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section .smpreset,"ax",@progbits
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
        la      $8,             mutekh_entry
        j       $8
        nop
#endif

FUNC_START(.text, mutekh_entry)

        .set push
        .set noreorder

        /* set up IT disable and kernel mode */
        li      $8,             CPU_MIPS_STATUS_IM
        MTC0_   $8,             CPU_MIPS_STATUS
        MTC0_   $0,             CPU_MIPS_CAUSE

#ifdef CONFIG_CPU_EXCEPTION_RELOCATABLE
        /* set cpu interrupts entry point */
        la      $4,             CPU_NAME_DECL(exception_vector)
        mtc0    $4,     $15,    1
#endif

#ifdef CONFIG_ARCH_SMP_CAPABLE

        CPU_ID  $8

        /* bootstrap processor ? */
        li      $4,             CONFIG_ARCH_BOOTSTRAP_CPU_ID
        beq     $8,     $4,     1f
        nop

# ifdef CONFIG_ARCH_SMP

        /* get cpu tree root */
        la      $5,     cpu_tree
        lw      $5,     ($5)

        /* get xor scrambled cpu id */
        li      $2,     CPU_TREE_XOR_VALUE
        xor     $4,     $2,     $8

4:
        /* not in tree */
        beq     $0,     $5,     2f
        nop

        /* get node cpu id */
        lw      $3,     CPU_TREE_CPU_ID($5)
        xor     $3,     $2,     $3

        /* matching node */
        beq     $4,     $3,     3f
        nop

        /* compare and get next child */
        slt     $6,     $3,     $4
        sll     $6,     $6,     2
        add     $6,     $6,     $5
        lw      $5,     CPU_TREE_CHILDS(0)($6)

        b       4b
        nop

3:        
        /* get address of the cpu stack */
        lw      $sp,    CPU_TREE_STACK($5)
        li      $4,     CONFIG_HEXO_CPU_STACK_SIZE
        add     $sp,    $4,     $sp
#  ifdef CONFIG_COMPILE_FRAMEPTR
        move    $fp,            $sp
#  endif

        /* jump to C code */
        la      $8,             mutekh_startup_smp
        j       $8
        nop

# endif
2:
# ifdef CONFIG_CPU_WAIT_IRQ
        wait
# endif
        b               2b
        nop
1:
#endif

        li      $sp,            CONFIG_STARTUP_STACK_ADDR + CONFIG_STARTUP_STACK_SIZE
#ifdef CONFIG_COMPILE_FRAMEPTR
        move    $fp,            $sp
#endif

        /* jump to C code */
        la      $8,             mutekh_startup
        j       $8
        nop

        .set pop
FUNC_END(mutekh_entry)

