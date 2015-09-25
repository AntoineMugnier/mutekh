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
    Copyright (c) 2010, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/context.h>
#include <hexo/cpu.h>
#include <hexo/asm.h>

/*
  The exception handling code share the processor state
  array used to save context registers values. When an exception occurs,
  we only save "caller saved" registers before calling the handler.
  When the handler returns we have two possible choices:
  
    * Restore "caller saved" registers and return to interrupted code
    * Complete current context saving with "callee saved" and fpu registers
      and switch to an other context.
  
  The context preemtion handler is used to take the switching decision.
  This preemption handler must be setup by C code during exception handling
  if their is a need to switch context instead of returning.
  
  Doing this allow us to perform as few register saving operations as
  possible because registers are already partially saved for exception
  handling purpose if we decide to switch context.
*/


/*
   Callee saved: r16-r23, r28-r31
   Caller saved: r1-r15, r24-r25, lo, hi
*/

.section .excep,"ax",@progbits

CPU_NAME_DECL(exception_vector):
        .globl CPU_NAME_DECL(exception_vector)

        .set push                                       
        .set noreorder                                  
        .set noat

/* MIPS32: TLB Refill, EXL = 0 */
        .org    0x0
        la      $26,   mips_exception
        j       $26
        nop

/* MIPS64: XTLB refill  */
        .org    0x80
        la      $26,   mips_exception
        j       $26
        nop

/* MIPS32: Cache error */
        .org    0x100
        la      $26,   mips_exception
        j       $26
        nop

/* MIPS32: General Exception */
        .org    0x180
        la      $26,   mips_exception
        j       $26
        nop

/* MIPS32: Interrupt, Cause IV = 1 */
        .org    0x200
        la      $26,   mips_exception
        j       $26
        nop

        .set pop                                        


/************************************************************************/

FUNC_START(.text, mips_exception)

        .set push                                       
        .set noat                                       

#if defined(CONFIG_HEXO_USERMODE) && defined(CONFIG_ARCH_SMP)
        /* restore cpu local storage */
        CPU_ID  $26
        sll     $26,    $26,    2
        lw      $27,    cpu_local_storage($26)       
# endif

        // get context local storage
        CPU_LOCAL_op lw,   __context_data_base, $26, $26
        // get context registers save array
#ifdef CONFIG_HEXO_CONTEXT_NESTED
        lw      $26,            %lo(mips_context_regs_ptr)($26)
#else
        addiu   $26,            %lo(mips_context_regs)
#endif

        // save only 6 "caller saved" registers yet (syscall args ragtable)
        // save gp ptr, stack ptr, frame ptr, return address
        .irp    r, 4, 5, 6, 7, 8, 9, 16, 28, 29, 30, 31
        sw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($26)
        .endr

        // use $16 as a preserved register accross C functions to keep pointer to regs
        move    $16,    $26

        // save status register
        mfc0    $9,     $CPU_MIPS_STATUS
        sw      $9,     CPU_MIPS_CONTEXT_SR($16)

#if (defined(CONFIG_HEXO_FPU) && !defined(CONFIG_RELEASE)) || defined(CONFIG_HEXO_CONTEXT_NESTED)
        // we need to change the status register
        move    $8,     $9

# ifdef CONFIG_HEXO_CONTEXT_NESTED
        // EI = EXL = ERL = KSU = 0, switch to normal kernel mode so
	// that irqs can be re-enabled in C code
        ori     $8,     $8,     31
        xori    $8,     $8,     31

        // nested regs context stack push
        CPU_LOCAL_op lw,   __context_data_base, $7, $7
        addiu   $26,            CPU_MIPS_CONTEXT_SIZE
        sw      $26,            %lo(mips_context_regs_ptr)($7)
# endif

# if defined(CONFIG_HEXO_FPU) && !defined(CONFIG_RELEASE)
        // disable fpu access
        li      $5,     CPU_MIPS_STATUS_FPU
        nor     $6,     $0,     $5
        and     $8,     $8,     $6
# endif

        MTC0_   $8,     CPU_MIPS_STATUS
#endif

#ifdef CONFIG_HEXO_USERMODE

        // from user mode ?
# ifdef CONFIG_CPU_MIPS_USE_ERET
        andi    $7,     $9,     CPU_MIPS_STATUS_UM
# else
        andi    $7,     $9,     CPU_MIPS_STATUS_KUp
# endif
        beq     $7,     $0,     1f

        // get tls
        CPU_LOCAL_op lw,   __context_data_base, $8, $8

        // use kernel context stack when from user mode
        lw      $sp,    context_stack_end($8)
        addiu   $sp,    $sp,    -4 * INT_REG_SIZE/8
1:
#endif

        // get cause
        mfc0    $6,     $CPU_MIPS_CAUSE                          
        andi    $4,     $6,     0x3c                 

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // clear context preempt handler
        CPU_LOCAL_op sw    cpu_preempt_handler, $0, $7
#endif

#ifdef CONFIG_HEXO_USERMODE
        // handle system call ?
        li      $7,     32                           
        beq     $4,     $7,     interrupt_sys        
#endif

        // save all remaining "caller saved" registers (not syscall args)
        .irp    r, 1, 2, 3, 10, 11, 12, 13, 14, 15, 24, 25
        sw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($16)
        .endr

        mflo    $1
        sw      $1,             CPU_MIPS_CONTEXT_LO($16)
        mfhi    $1
        sw      $1,             CPU_MIPS_CONTEXT_HI($16)

#ifdef CONFIG_HEXO_IRQ
        // handle irq ?
        beq     $4,     $0,     interrupt_hw
#endif

        /*************************************************************
          exception handling
         **************************************************************/
interrupt_ex:                                   

        // get epc
        mfc0    $5,     $CPU_MIPS_EPC
        sw      $5,     CPU_MIPS_CONTEXT_PC($16)

        // faulty instruction is epc + 4 if 'cause branch delay' bit is set
        li      $7,     CPU_MIPS_CAUSE_BD
        and     $6,     $6,     $7
        beq     $6,     $0,     1f
        addiu   $5,     $5,     4
1:      

#ifdef CONFIG_HEXO_USERMODE

        // from user mode ?
# ifdef CONFIG_CPU_MIPS_USE_ERET
        andi    $7,     $9,     CPU_MIPS_STATUS_UM
# else
        andi    $7,     $9,     CPU_MIPS_STATUS_KUp
# endif
        beq     $7,     $0,     1f
        
        // use context local fault handler for user mode when available
        lw      $1,     cpu_user_exception_handler($8)
        bne     $1,     $0,     2f
1:
#endif
        // use cpu local fault handler
        CPU_LOCAL_op lw,   cpu_exception_handler, $1, $1
2:

        addiu   $sp,    $sp,    -5 * INT_REG_SIZE/8

        lw      $6,             CPU_MIPS_CONTEXT_GPR(29)($16)
        sw      $6,             (4 * INT_REG_SIZE/8)($sp) // arg5: stackptr

        srl     $4,     $4,     2                  // arg0 type
        //      arg1: $5 is execptr
        mfc0    $6,     $CPU_MIPS_BADADDR          // arg2 data error
        move    $7,     $16                        // arg3 reg table

        // call handler
        jalr    $1

#ifdef CONFIG_CPU_MIPS_USE_ERET
        // read back pc
        lw      $1,    CPU_MIPS_CONTEXT_PC($16)
        MTC0_   $1,    CPU_MIPS_EPC
#endif

        j       mips_preempt_and_restore

        /*************************************************************
          syscall handling
         **************************************************************/
#ifdef CONFIG_HEXO_USERMODE
interrupt_sys:                                  

        // get epc
        mfc0    $5,     $CPU_MIPS_EPC
        // resume pc += 4
        addiu   $5,     $5,     4
        sw      $5,     CPU_MIPS_CONTEXT_PC($16)

        move    $4,     $0                             // arg0 id == 0
        move    $5,     $16                            // arg1 reg table

        lw      $1,     cpu_syscall_handler($8)

        addiu   $sp,    $sp,    -4 * INT_REG_SIZE/8
        jalr    $1                                   

# ifdef CONFIG_CPU_MIPS_USE_ERET
        // read back pc
        lw      $1,     CPU_MIPS_CONTEXT_PC($16)
        MTC0_   $1,     CPU_MIPS_EPC
# endif

        // FIXME cleanup all caller saved registers here for security reasons ?
        // FIXME do not need to restore everything for syscall
        j       mips_preempt_and_restore
#endif

        /*************************************************************
          hardware interrupts handling
         **************************************************************/
#ifdef CONFIG_HEXO_IRQ
interrupt_hw:                                   

        // get epc
# if defined (CONFIG_HEXO_CONTEXT_PREEMPT) || !defined (CONFIG_CPU_MIPS_USE_ERET) || defined(CONFIG_CPU_MIPS_WAIT_NEEDS_IE)
        mfc0    $5,     $CPU_MIPS_EPC
# endif

# ifdef CONFIG_CPU_MIPS_WAIT_NEEDS_IE
        // skip 4 instructions if inside cpu_interrupt_wait
        la      $4,     mips_interrupt_wait
        subu    $4,     $5,     $4
        sltiu   $4,     $4,     16
        sll     $4,             4
        addu    $5,     $4
        mtc0    $5,     $CPU_MIPS_EPC
# endif

# if defined (CONFIG_HEXO_CONTEXT_PREEMPT) || !defined (CONFIG_CPU_MIPS_USE_ERET)
        sw      $5,     CPU_MIPS_CONTEXT_PC($16)
# endif

        /* pending hw irq enabled in status */
        and     $4,     $6,     $9
        srl     $4,     $4,     CPU_MIPS_STATUS_IM_SHIFT
        andi    $4,     $4,     CPU_MIPS_STATUS_IM >> CPU_MIPS_STATUS_IM_SHIFT

        /* find lowest pending irq line */
#if (CPU_MIPS_STATUS_IM >> CPU_MIPS_STATUS_IM_SHIFT) > 256
# warning Interrupt mask larger than 8 bits
        jal     __ffssi2
        addiu   $4,     $2,     -1
#else
        // optimized 8 bits version
        subu    $2,	$0,	$4
        and     $2,	$2,	$4
        addiu   $2,	$2,	-1
        srl     $4,	$2,	1
        andi    $4,	$4,	0x55
        subu    $4,	$2,	$4
        srl     $2,	$4,	2
        andi    $2,	$2,	0x33
        andi    $4,	$4,	0x33
        addu    $2,	$2,	$4
        srl     $3,	$2,	4
        addu    $4,	$3,	$2
#endif

        CPU_LOCAL_op lw,   cpu_interrupt_handler, $1, $1

        addiu   $sp,    $sp,    -4 * INT_REG_SIZE/8
        jalr    $1                                   

        // j       mips_preempt_and_restore
#endif

mips_preempt_and_restore:

#ifdef CONFIG_HEXO_CONTEXT_NESTED
        // nested regs context stack pop
        CPU_LOCAL_op lw,   __context_data_base, $8, $8
        sw      $16,    %lo(mips_context_regs_ptr)($8)
#endif

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT

        // check if a preempt handler has been registered
        CPU_LOCAL_op lw    cpu_preempt_handler, $1, $1
        beq     $1,     $0,     mips_restore

        // call preempt handler
        jalr    $1

        // got pointer to context to switch to ?
        beq     $2,     $0,     mips_restore

        // save callee saved registers
        .irp    r, 17, 18, 19, 20, 21, 22, 23
        sw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($16)
        .endr

        // prepare cpu_context_jumpto arg
        move    $4,     $2

# ifdef CONFIG_HEXO_CONTEXT_STATS
        move    $17,    $2
        jal     context_preempt_stats
        move    $4,     $17
# endif

# ifdef CONFIG_HEXO_FPU
        // test fpu enabled for previous context
        lw      $9,     CPU_MIPS_CONTEXT_SR($16)
        li      $7,     CPU_MIPS_STATUS_FPU
        and     $7,     $7,     $9
        beq     $7,     $0,     1f

#  ifdef CONFIG_HEXO_LAZY_SWITCH
        // keep track of last context using fpu on this processor
        CPU_LOCAL_op sw    mips_lazy_last, $16, $1
#  endif

        // save fpu registers 0 to 31
        .irp    r, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
        SxC1    \r,            CPU_MIPS_CONTEXT_FR(\r)($16)
        .endr
        .irp    r, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
        SxC1    \r,            CPU_MIPS_CONTEXT_FR(\r)($16)
        .endr

        // save fpu status
        cfc1    $1,     $31
        sw      $1,     CPU_MIPS_CONTEXT_FCSR($16)
1:
# endif

        li      $1,     CPU_MIPS_CONTEXT_RESTORE_CALLER | CPU_MIPS_CONTEXT_RESTORE_CALLEE
        sw      $1,     CPU_MIPS_CONTEXT_SAVE_MASK($16)

        // switch to new context
        b       cpu_context_jumpto
#endif

mips_restore:

#if (defined(CONFIG_HEXO_FPU) && !defined(CONFIG_RELEASE)) || defined(CONFIG_HEXO_CONTEXT_NESTED)
        // we need to restore the status either because the fpu have
	// been disabled during irq handling or because exl/erl flags have been cleared
        lw      $4,     CPU_MIPS_CONTEXT_SR($16)

        // keep IM2-IM7/IPL bits from status
        mfc0    $5,     $CPU_MIPS_STATUS
        xor     $5,     $4
        andi    $5,     0xfc00
        xor     $4,     $5

        MTC0_   $4,     CPU_MIPS_STATUS
#endif

        // restore some "caller saved" registers
        lw      $1,             CPU_MIPS_CONTEXT_LO($16)
        mtlo    $1
        lw      $1,             CPU_MIPS_CONTEXT_HI($16)
        mthi    $1

        move    $26,    $16

        .irp    r, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 25
        lw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($26)
        .endr

        // FIXME syscall could restore from this point
        // restore return value register
        lw      $2,             CPU_MIPS_CONTEXT_GPR(2)($26)

        // restore frame, stack...
        .irp    r, 28, 29, 30, 31
        lw      $\r,            CPU_MIPS_CONTEXT_GPR(\r)($26)
        .endr

#ifdef CONFIG_CPU_MIPS_USE_ERET
        eret
#else
        lw      $26,    CPU_MIPS_CONTEXT_PC($26)
        .set noreorder                                  
        jr      $26                                  
        rfe                                          
#endif
        .set pop                                        

FUNC_END(mips_exception)

#if defined(CONFIG_HEXO_IRQ) && defined(CONFIG_CPU_WAIT_IRQ)
FUNC_START(.text, mips_interrupt_wait)
        .set push
        .set noreorder
        .set noat

#ifdef CONFIG_CPU_MIPS_WAIT_NEEDS_IE
        // enable irq before wait, race handled in interrupt_hw
        mfc0    $1,     $12     // must be at mips_interrupt_wait + 0
        ori     $1,     1
        mtc0    $1,     $12
#endif
        wait
#ifdef CONFIG_CPU_MIPS_WAIT_NEEDS_IE
        jr      $31             // must be at mips_interrupt_wait + 16
        nop
        nop
        nop
#else
        // enable irq after wait
        mfc0    $1,     $12
        ori     $1,     1
        mtc0    $1,     $12
#endif
        jr      $31
        nop

        .set pop
FUNC_END(mips_interrupt_wait)
#endif

// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

