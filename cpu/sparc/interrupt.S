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

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>

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

  Have a look to context_switch.S for more.
*/

.macro PUT_ENTRY n k
        sethi   %hi(\k),          %l4
        or      %l4,    %lo(\k),  %l4
        jmp     %l4
        mov     \n, %l4
.endm

.macro PUT_ENTRIES i j k
        PUT_ENTRY \i, \k
    .if \i+1<=\j
        PUT_ENTRY (\i+1), \k
    .endif
    .if \i+2<=\j
        PUT_ENTRY (\i+2), \k
    .endif
    .if \i+3<=\j
        PUT_ENTRY (\i+3), \k
    .endif
    .if \i+4<=\j
        PUT_ENTRIES (\i+4), \j, \k
    .endif
.endm

.macro PUT_EQ_ENTRIES i j k id
        PUT_ENTRY \id, \k
    .if \i+1<=\j
        PUT_ENTRY \id, \k
    .endif
    .if \i+2<=\j
        PUT_ENTRY \id, \k
    .endif
    .if \i+3<=\j
        PUT_ENTRY \id, \k
    .endif
    .if \i+4<=\j
        PUT_EQ_ENTRIES (\i+4), \j, \k, \id
    .endif
.endm

.section .excep,"ax",@progbits

CPU_NAME_DECL(exception_vector):
        .globl CPU_NAME_DECL(exception_vector)

      /* reset 0x00 */
#ifndef CONFIG_LOAD_RESET_SEPARATE
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
#endif
        PUT_ENTRY       0,          mutekh_entry

#ifdef CONFIG_HEXO_EXCEP
      /* 11 faults 0x01 - 0x0b */
        .org CPU_NAME_DECL(exception_vector) + 0x01 * 16
        PUT_ENTRIES     1,    11,   sparc_excep_entry
        PUT_EQ_ENTRIES  0x0c, 0x10, sparc_excep_entry,  0
#endif

#ifdef CONFIG_HEXO_IRQ
      /* 15 irqs 0x11 - 0x1f */
        .org CPU_NAME_DECL(exception_vector) + 0x11 * 16
# ifdef CONFIG_CPU_SPARC_BLOCK_NMI
        PUT_ENTRIES     0,    13,   sparc_irq_entry
        PUT_ENTRY       14,         sparc_nmi_entry
# else
        PUT_ENTRIES     0,    14,   sparc_irq_entry
# endif
#endif

#ifdef CONFIG_HEXO_EXCEP
        .org CPU_NAME_DECL(exception_vector) + 0x20 * 16
      /* 10 faults 0x20 - 0x3c */
        PUT_ENTRIES     12,   13,   sparc_excep_entry /* 0x20, 0x21 */
        PUT_EQ_ENTRIES  0x22, 0x23, sparc_excep_entry, 0
        PUT_ENTRIES     14,   15,   sparc_excep_entry /* 0x24, 0x25 */
        PUT_EQ_ENTRIES  0x26, 0x27, sparc_excep_entry, 0
        PUT_ENTRIES     16,   20,   sparc_excep_entry /* 0x28 - 0x2c */
        PUT_EQ_ENTRIES  0x2d, 0x3b, sparc_excep_entry, 0
        PUT_ENTRY       21,         sparc_excep_entry /* 0x3c */
        PUT_EQ_ENTRIES  0x3d, 0x7f, sparc_excep_entry, 0
#endif

#ifdef CONFIG_HEXO_USERMODE
        .org CPU_NAME_DECL(exception_vector) + 0x80 * 16
      /* syscalls */
        PUT_ENTRIES     0x0, 0x7f, sparc_syscall_entry
#endif


/************************************************************************/

/* This macro use all local regs to save caller saved register before calling C code */
.macro SPARC_EXCEPT_SAVE

#ifdef CONFIG_HEXO_USERMODE
# ifdef CONFIG_ARCH_SMP
        // restore cls
        CPU_ID  %g5
        sll     %g5,    2,      %g5
        set     cpu_local_storage, %g6
        ld      [%g6 + %g5],    %g6
# endif
        // restore tls
        CPU_LOCAL_ld  __context_data_base, %g7
#endif

#ifdef CONFIG_HEXO_CONTEXT
        // get context registers save array in g5
        add     %g7, %lo(sparc_context_regs), %g5
#else
        // store cpu_context_s on stack
        sub     %i6, _sizeof(struct cpu_context_s), %g5
#endif

#if CONFIG_CPU_SPARC_WINCOUNT < 3
        /* need to save %o* registers to preserve %i* registers in
	   overlapping window of interrupted code if we have only 2 windows */
        
        std     %o0, [%g5 + CPU_SPARC_CONTEXT_I(0)]
        std     %o2, [%g5 + CPU_SPARC_CONTEXT_I(2)]
        std     %o4, [%g5 + CPU_SPARC_CONTEXT_I(4)]
        std     %o6, [%g5 + CPU_SPARC_CONTEXT_I(6)]
#endif

#if defined(CONFIG_CPU_SPARC_APP_REGS)
        std     %g2, [%g5 + CPU_SPARC_CONTEXT_G(2)]
        st      %g4, [%g5 + CPU_SPARC_CONTEXT_G(4)]
#endif

        // preserve other caller saved registers
        mov     %y,     %l0
        mov     %g1,    %l3
        mov     %psr,   %l5

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // clear context preempt handler
        CPU_LOCAL_st    cpu_preempt_handler, %g0, %o0
#endif

        // use interrupted code stack
#ifdef CONFIG_HEXO_CONTEXT
        mov     %i6,    %o6
#else
        mov     %g5,    %o6
#endif

.endm


.macro SPARC_EXCEPT_SET_STACK        
#ifdef CONFIG_HEXO_USERMODE
        // from usermode ?
        andcc   %l5,    SPARC_PSR_PREV_SUSER_MODE, %g0
        bne     1f
        nop

        // use kernel stack
        ld      [%g7 + %lo(context_stack_end)], %o6
1:      
#endif

        // skip register window save area
        add     %sp,    -SPARC_STACK_REDZONE,    %sp

.endm


#ifdef CONFIG_HEXO_EXCEP
FUNC_START(.text, sparc_excep_entry)

        SPARC_EXCEPT_SAVE

#ifdef CONFIG_HEXO_USERMODE
        // from usermode ?
        andcc   %l5,    SPARC_PSR_PREV_SUSER_MODE, %g0
        bne     1f
        nop

        // use kernel stack
        ld      [%g7 + %lo(context_stack_end)], %o6

        // use context local handler
        ld      [%g7 + %lo(cpu_user_exception_handler)], %o5
        tst     %o5

        bne     2f
        nop
1:      
#endif

        // skip register window save area
        add     %sp,    -SPARC_STACK_REDZONE,    %sp

        // use cpu local handler
        CPU_LOCAL_ld cpu_exception_handler, %o5
2:      

        // dump some registers
        st     %l5, [%g5 + CPU_SPARC_CONTEXT_PSR]
        st     %l1, [%g5 + CPU_SPARC_CONTEXT_PC]
        st     %l2, [%g5 + CPU_SPARC_CONTEXT_NPC]

        mov     %l4,    %o0 // arg1 is exception id
        mov     %l1,    %o1 // arg2 execptr
        mov     %g0,    %o2 // arg3 dataptr /* FIXME data access address ? */
        mov     %g5,    %o3 // arg4 regtable
        // call exception handler
        call    %o5
        mov     %i6,    %o4 // arg5 stack

        // read some registers back in case they were modified by c code
        ld     [%g5 + CPU_SPARC_CONTEXT_PSR], %l5
        ld     [%g5 + CPU_SPARC_CONTEXT_PC] , %l1
        b       sparc_except_restore
        ld     [%g5 + CPU_SPARC_CONTEXT_NPC], %l2

FUNC_END(sparc_excep_entry)
#endif

#ifdef CONFIG_HEXO_USERMODE
FUNC_START(.text, sparc_syscall_entry)
        
        SPARC_EXCEPT_SAVE
        SPARC_EXCEPT_SET_STACK

        // skip trap instruction
        add     %l1,    4,      %l1
        add     %l2,    4,      %l2

        // dump some registers
        st     %g1, [%g5 + CPU_SPARC_CONTEXT_G(1)]
        std    %i0, [%g5 + CPU_SPARC_CONTEXT_O(0)]
        std    %i2, [%g5 + CPU_SPARC_CONTEXT_O(2)]
        std    %i4, [%g5 + CPU_SPARC_CONTEXT_O(4)]
        std    %i6, [%g5 + CPU_SPARC_CONTEXT_O(6)]

        ld      [%g7 + %lo(cpu_syscall_handler)], %o2

        mov     %l4,    %o0
        call    %o2
        mov     %g5,    %o1

        // read back return value
        b       sparc_except_restore
        ld     [%g5 + CPU_SPARC_CONTEXT_O(0)], %i0


FUNC_END(sparc_syscall_entry)
#endif


#ifdef CONFIG_HEXO_IRQ
FUNC_START(.text, sparc_nmi_entry)
        // FIXME handle nmi here
        jmp     %l1
        rett    %l2
FUNC_END(sparc_nmi_entry)

FUNC_START(.text, sparc_irq_entry)

        SPARC_EXCEPT_SAVE
        SPARC_EXCEPT_SET_STACK

        // get irq handler address
        CPU_LOCAL_ld     cpu_interrupt_handler, %o1

        // call irq handler
        call    %o1
        mov     %l4,    %o0 // arg1 irq id

#if 0
        b       sparc_except_restore
        nop
#endif

FUNC_END(sparc_irq_entry)
#endif


/* This function restore caller saved register from local regs */
FUNC_START(.text, sparc_except_restore)
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        
        // check if a preempt handler has been registered
        CPU_LOCAL_ld    cpu_preempt_handler, %o0

        tst     %o0
        beq     1f
        nop

        // call preempt handler
        call    %o0
        nop

        // got pointer to context to switch to ?
        tst     %o0
        beq     2f
        // save all registers available from this exception window
        st      %l0,    [%g5 + CPU_SPARC_CONTEXT_Y]
        st      %l1,    [%g5 + CPU_SPARC_CONTEXT_PC]
        st      %l2,    [%g5 + CPU_SPARC_CONTEXT_NPC]
        st      %l3,    [%g5 + CPU_SPARC_CONTEXT_G(1)]
        st      %l5,    [%g5 + CPU_SPARC_CONTEXT_PSR]

        std     %i0,    [%g5 + CPU_SPARC_CONTEXT_O(0)]
        std     %i2,    [%g5 + CPU_SPARC_CONTEXT_O(2)]
        std     %i4,    [%g5 + CPU_SPARC_CONTEXT_O(4)]
        std     %i6,    [%g5 + CPU_SPARC_CONTEXT_O(6)]

# ifdef CONFIG_HEXO_CONTEXT_STATS
        // update context stats
        call    context_preempt_stats
# endif
        // copy cpu_context_jumpto arg in previous window %o0
        mov     %o0,    %i0

        restore
        // save all remaining registers from this point

        std     %l0, [%g5 + CPU_SPARC_CONTEXT_L(0)]
        std     %l2, [%g5 + CPU_SPARC_CONTEXT_L(2)]
        std     %l4, [%g5 + CPU_SPARC_CONTEXT_L(4)]
        std     %l6, [%g5 + CPU_SPARC_CONTEXT_L(6)]

# if CONFIG_CPU_SPARC_WINCOUNT >= 3
        // already saved if we have only 2 windows because of overlap
        std     %i0, [%g5 + CPU_SPARC_CONTEXT_I(0)]
        std     %i2, [%g5 + CPU_SPARC_CONTEXT_I(2)]
        std     %i4, [%g5 + CPU_SPARC_CONTEXT_I(4)]
        std     %i6, [%g5 + CPU_SPARC_CONTEXT_I(6)]
# endif

        // write save/restore mask
        mov     CPU_SPARC_CONTEXT_RESTORE_CALLEE | CPU_SPARC_CONTEXT_RESTORE_CALLER, %l0
        // switch to new context
        b       cpu_context_jumpto
        st      %l0, [%g5 + CPU_SPARC_CONTEXT_SAVE_MASK]
1:
        // invalidate handler outside interrupt
        CPU_LOCAL_st    cpu_preempt_handler, %g5, %o0
2:
#endif
        
        // restore caller saved registers
        mov     %l0,     %y
        mov     %l5,   %psr

        mov     %l3,    %g1

#if CONFIG_CPU_SPARC_WINCOUNT < 3
        /* need to restore %o* overlapping registers */

        ldd     [%g5 + CPU_SPARC_CONTEXT_I(0)], %o0
        ldd     [%g5 + CPU_SPARC_CONTEXT_I(2)], %o2
        ldd     [%g5 + CPU_SPARC_CONTEXT_I(4)], %o4
        ldd     [%g5 + CPU_SPARC_CONTEXT_I(6)], %o6
#endif

#if defined(CONFIG_CPU_SPARC_APP_REGS)
        ldd     [%g5 + CPU_SPARC_CONTEXT_G(2)], %g2
        ld      [%g5 + CPU_SPARC_CONTEXT_G(4)], %g4
#endif

        /* return from exception */
        jmp     %l1
        rett    %l2
FUNC_END(sparc_except_restore)

// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

