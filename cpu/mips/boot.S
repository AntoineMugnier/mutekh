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

#ifdef CONFIG_ARCH_SMP
# include <device/class/cpu.h>
#endif

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        .set noreorder

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

#ifdef CONFIG_CPU_EXCEPTION_RELOCATABLE
        /* set cpu interrupts entry point */
        la      $4,             CPU_NAME_DECL(exception_vector)
        mtc0    $4,     $15,    1
#endif

#if defined(CONFIG_CPU_CACHE) && CONFIG_CPU_MIPS_VERSION >= 32
        /* Test if CONFIG 1 */
        mfc0    $2,     $16,    0
        srl     $3,     $2,     31
        beqz    $3,     2f
        nop
        mfc0    $2,     $16,    1

        /* Icache way */
        srl     $3,     $2,     16
        andi    $3,     7
        addiu   $3,     1
        /* Icache line count */
        srl     $4,     $2,     22
        andi    $4,     7
        li      $5,     64
        sllv    $4,     $5,     $4
        /* Icache line size */
        srl     $5,     $2,     19
        andi    $5,     7
        li      $6,     2
        sllv    $5,     $6,     $5
        /* way * line count */
        mul     $3,     $3,     $4
        addiu   $3,     $3,     -1
        move    $4,     $0
3:      cache   MIPS32_CACHE_INSTRUCTION | MIPS32_CACHE_OP_INDEX_INVALIDATE, 0($4)
        addu    $4,     $5    
        bne     $3,     $0,    3b
        addiu   $3,     -1   

        mtc0    $0,     $28,    0
        /* Dcache way */
        srl     $3,     $2,     7
        andi    $3,     7
        addiu   $3,     1
        /* Dcache line count */
        srl     $4,     $2,     13
        andi    $4,     7
        li      $5,     64
        sllv    $4,     $5,     $4
        /* Icache line size */
        srl     $5,     $2,     10
        andi    $5,     7
        li      $6,     2
        sllv    $5,     $6,     $5
        /* way * line count */
        mul     $3,     $3,     $4
        addiu   $3,     $3,     -1
        move    $4,     $0
3:      cache   MIPS32_CACHE_DATA | MIPS32_CACHE_OP_INDEX_STORE_TAG, 0($4)
        addiu   $3,     -1   
        bne     $3,     $0,    3b
        addu    $4,     $5    

        /* Enable write through cache */
        mfc0    $2,     $16,    0
        ori     $2,     7
        xori    $2,     (7 - CPU_MIPS_CACHE_WTNA)
        mtc0    $2,     $16,    0
2:
#endif

        /* set up IT disable and kernel mode */
        MTC0_   $0,             CPU_MIPS_CAUSE
        li      $8,             CPU_MIPS_STATUS_IM
        MTC0_   $8,             CPU_MIPS_STATUS

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
        lw      $3,     _offsetof(cpu_tree_s, cpu_id)($5)
        xor     $3,     $2,     $3

        /* matching node */
        beq     $4,     $3,     3f
        nop

        /* compare and get next child */
        slt     $6,     $3,     $4
        sll     $6,     $6,     2
        add     $6,     $6,     $5
        lw      $5,     _offsetof(cpu_tree_s, childs)($6)

        b       4b
        nop

3:        
        /* get address of the cpu stack */
        lw      $sp,    _offsetof(cpu_tree_s, stack)($5)
        li      $4,     CONFIG_HEXO_CPU_STACK_SIZE - (4 * INT_REG_SIZE/8)
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

        li      $sp,            CONFIG_STARTUP_STACK_ADDR + CONFIG_STARTUP_STACK_SIZE - (4 * INT_REG_SIZE/8)
#ifdef CONFIG_COMPILE_FRAMEPTR
        move    $fp,            $sp
#endif

        /* jump to C code */
        la      $8,             mutekh_startup
        j       $8
        nop

        .set pop
FUNC_END(mutekh_entry)

