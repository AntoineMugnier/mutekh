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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/cpu.h>
#include <hexo/lock.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

/*
   Callee saved: r16-r23, r28-r31, f20-f31
   Caller saved: r1-r15, r24-r25, lo, hi, f0-f19, fcsr
*/

#ifdef CONFIG_HEXO_CONTEXT

FUNC_START(.text, cpu_context_switch)
        .set push
        .set noat

        // get context local storage
        CPU_LOCAL_op lw,   __context_data_base, $1, $1
        // get context registers save array
#ifdef CONFIG_HEXO_CONTEXT_NESTED
        lw      $2,             %lo(mips_context_regs_ptr)($1)
#else
        addiu   $2,     $1,     %lo(mips_context_regs)
#endif

        // save "callee saved" registers
        // save gp ptr, stack ptr, frame ptr, return address
        .irp    r, 16, 17, 18, 19, 20, 21, 22, 23, 28, 29, 30, 31
        sw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($2)
        .endr

        // resume directly at return address
        sw      $31,    CPU_MIPS_CONTEXT_PC($2)

        // save status register
        mfc0    $8,     $CPU_MIPS_STATUS
        sw      $8,     CPU_MIPS_CONTEXT_SR($2)

#ifdef CONFIG_HEXO_FPU
        // test fpu enabled
        li      $7,     CPU_MIPS_STATUS_FPU
        and     $7,     $7,     $8
        beq     $7,     $0,     1f

# ifdef CONFIG_HEXO_LAZY_SWITCH
        // keep track of last context using fpu on this processor
        CPU_LOCAL_op sw    mips_lazy_last, $2, $7
# endif

        // save fpu registers 20 to 30
        .irp    r, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
        SxC1    \r,            CPU_MIPS_CONTEXT_FR(\r)($2)
        .endr
1:      
#endif

        // write save/restore mask
        li      $9,     CPU_MIPS_CONTEXT_RESTORE_CALLEE
        sw      $9,     CPU_MIPS_CONTEXT_SAVE_MASK($2)

        b       cpu_context_jumpto
        .set pop                                
FUNC_END(cpu_context_switch)





FUNC_START(.text, cpu_context_jumpto)
        .set push                       
        .set noat                       

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* enter memchecker command mode */
        li      $1,     SOCLIB_MC_MAGIC_VAL  
        sw      $1,     SOCLIB_MC_MAGIC($0) 

        /* switch to associated memchecker context */
        sw      $4,     SOCLIB_MC_CTX_SET($0) 
#endif

#if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
        // unlock some atomic value on context restore if requested
        lw      $25,    _offsetof(context_s, unlock, 4) ($4)
        beq     $25,    $0,     1f

        // clear unlock address/memory barrier
        sw      $0,     _offsetof(context_s, unlock, 4) ($4)
        sync
        // unlock
        sw      $0,     _offsetof(__arch_lock_s, __arch_lock_unlock, 4) ($25)
        // flush the write buffer if present
        //  -> the write is sent faster to memory
        sync
1:
# endif

        // restore tls pointer from context struct
        lw      $25,    _offsetof(context_s, tls, 4) ($4)
        CPU_LOCAL_op sw,   __context_data_base, $25, $1

#ifdef CONFIG_HEXO_CONTEXT_IRQEN
        // clear irq enable hook
        CPU_LOCAL_op sw,   cpu_irqen_handler, $0, $1
#endif

        // context registers save array
#ifdef CONFIG_HEXO_CONTEXT_NESTED
        lw      $25,    %lo(mips_context_regs_ptr)($25)
#else
        addiu   $25,    %lo(mips_context_regs)
#endif

        // restore gp ptr, stack ptr, frame ptr, return address
        .irp    r, 28, 29, 30, 31
        lw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($25)
        .endr

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* leave memchecker command mode */
        sw      $0,      SOCLIB_MC_MAGIC($0) 
#endif

        // restore status
        lw      $4,     CPU_MIPS_CONTEXT_SR($25)

        // keep IM2-IM7/IPL bits from status
        mfc0    $5,     $CPU_MIPS_STATUS
        xor     $5,     $4
        andi    $5,     0xfc00
        xor     $4,     $5

#ifdef CONFIG_CPU_MIPS_USE_ERET
        // context will be restored as an exception return
        ori     $4,     CPU_MIPS_STATUS_EXL
#endif
        MTC0_   $4,     CPU_MIPS_STATUS

        // restore pc
        lw      $26,    CPU_MIPS_CONTEXT_PC($25)
#ifdef CONFIG_CPU_MIPS_USE_ERET
        MTC0_   $26,    CPU_MIPS_EPC
#endif

        lw      $24,    CPU_MIPS_CONTEXT_SAVE_MASK($25)

#ifdef CONFIG_HEXO_FPU
        // check if next context use fpu
        li      $5,     CPU_MIPS_STATUS_FPU
        and     $1,     $4,     $5
        beq     $0,     $1,     1f

# ifdef CONFIG_HEXO_LAZY_SWITCH
        // do not need to restore if no other context used processor fpu since
        CPU_LOCAL lw    mips_lazy_last, $1, $1
        beq     $1,     $25,    1f
# endif

        // restore callee saved fpu registers
        .irp    r, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
        LxC1    \r,            CPU_MIPS_CONTEXT_FR(\r)($25)
        .endr

        lw      $1,     CPU_MIPS_CONTEXT_FCSR($25)
        ctc1    $1,     $31

        andi    $7,     $24,    CPU_MIPS_CONTEXT_RESTORE_CALLER
        beq     $0,     $7,     1f

        // restore caller saved fpu registers
        .irp    r, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
        LxC1    \r,            CPU_MIPS_CONTEXT_FR(\r)($25)
        .endr
1:      
#endif

#if 0   // callee should almost always get restored (except on context start)
        andi    $7,     $24,    CPU_MIPS_CONTEXT_RESTORE_CALLEE
        beq     $0,     $7,     1f
#endif

        // restore "callee saved" registers
        .irp    r, 16, 17, 18, 19, 20, 21, 22, 23
        lw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($25)
        .endr
1:

        andi    $7,     $24,    CPU_MIPS_CONTEXT_RESTORE_CALLER
        beq     $0,     $7,     1f

        // restore "caller saved" register
        lw      $1,             CPU_MIPS_CONTEXT_LO($25)
        mtlo    $1
        lw      $1,             CPU_MIPS_CONTEXT_HI($25)
        mthi    $1

        .irp    r, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 24, 25
        lw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($25)
        .endr

1:

#ifdef CONFIG_CPU_MIPS_USE_ERET
        eret
#else
        .set noreorder                                  
        jr      $26                                  
        rfe                                          
#endif
        .set pop

FUNC_END(cpu_context_jumpto)

#endif /* CONFIG_HEXO_CONTEXT */


FUNC_START(.text, cpu_context_set)
        .set push                       
        .set noat

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* enter memchecker command mode */
        li      $1,     SOCLIB_MC_MAGIC_VAL  
        sw      $1,     SOCLIB_MC_MAGIC($0) 

        /* mark current memchecker context as invalid */
        addiu   $1,     $0,     SOCLIB_MC_CTX_ID_CURRENT
        sw      $1,             SOCLIB_MC_CTX_INVALIDATE ($0) 

        /* create a new memchecker context using passed stack */
        sw      $4,      SOCLIB_MC_R1 ($0) 
        sw      $5,      SOCLIB_MC_R2 ($0) 
        sw      $4,      SOCLIB_MC_CTX_CREATE ($0) 

        /* switch to new memchecker context */
        sw      $4,      SOCLIB_MC_CTX_SET ($0) 
#endif

        addu    $sp,     $4,     $5 
#ifdef CONFIG_COMPILE_FRAMEPTR
        move    $fp,            $sp
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* leave memchecker command mode */
        sw      $0,      SOCLIB_MC_MAGIC ($0) 
#endif

        /* Jump */
        addiu   $sp,    $sp,     -4 * INT_REG_SIZE/8
        j       $6
        .set pop                        
FUNC_END(cpu_context_set)


#ifdef CONFIG_HEXO_CONTEXT

FUNC_START(.text, cpu_context_stack_use)
        .set push                       
        .set noat

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* enter memchecker command mode */
        li      $1,     SOCLIB_MC_MAGIC_VAL  
        sw      $1,     SOCLIB_MC_MAGIC($0) 

        /* switch to associated memchecker context */
        sw      $4,     SOCLIB_MC_CTX_SET($0) 
#endif

        // get tls pointer from context struct
        lw      $25,     _offsetof(context_s, tls, 4) ($4)
        // get context registers save array
#ifdef CONFIG_HEXO_CONTEXT_NESTED
        lw      $25,    %lo(mips_context_regs_ptr)($25)
#else
        addiu   $25,    %lo(mips_context_regs)
#endif

        // use context stack
        lw      $sp,    CPU_MIPS_CONTEXT_GPR(CPU_MIPS_SP)($25)
#ifdef CONFIG_COMPILE_FRAMEPTR
        move    $fp,            $sp
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* leave memchecker command mode */
        sw      $0,      SOCLIB_MC_MAGIC ($0) 
#endif

#ifndef CONFIG_RELEASE
        // prevent use of previous context local storage
        CPU_LOCAL_op sw,   __context_data_base, $0, $1
#endif

        // private param
        move    $4,     $6

        // call function
        addiu   $sp,    $sp,    -4 * INT_REG_SIZE/8
        jr      $5

        .set pop                        
FUNC_END(cpu_context_stack_use)


#if defined(CONFIG_HEXO_USERMODE)

FUNC_START(.text, cpu_context_set_user)
        .set push
        .set noat

        /* set stack */
        addiu   $sp,    $4,     -4 * INT_REG_SIZE/8

        /* set arg */
        move    $4,     $6

# ifdef CONFIG_CPU_MIPS_USE_ERET
        /* entrypoint goes into epc */
        MTC0_   $5,             CPU_MIPS_EPC

        /* set user mode in status */
        mfc0    $1,             $CPU_MIPS_STATUS
        ori     $1,     $1,     CPU_MIPS_STATUS_UM | CPU_MIPS_STATUS_EXL
        MTC0_   $1,     CPU_MIPS_STATUS

        .set noreorder
        eret
# else
        /* set previous state as user */
        mfc0    $1,             $CPU_MIPS_STATUS
        ori     $1,     $1,     CPU_MIPS_STATUS_KUp
        MTC0_   $1,             CPU_MIPS_STATUS

        /* "restore" user mode and jump */
        .set noreorder
        jr      $5
        rfe
# endif

        .set pop

FUNC_END(cpu_context_set_user)

#endif

#endif /* CONFIG_HEXO_CONTEXT */
