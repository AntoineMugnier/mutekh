/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Luc Delecroix <luc D delecroix A thalesgroup D com> (c) 2011
    Copyright Laurent Gantel <laurent D gantel A ensea D fr> (c) 2011
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



/*
  The MBZ_EXCEPT_SAVE macro contains code to patially save processor
  state. Only "caller saved" registers are saved here because "callee saved"
  registers won't be clobbered by C execption handler: it's enough to
  completly restore processor state on return.

  Fpu registers are not saved because irq handlers should not rely on
  hardware floating point operations.

*/
/*
	Microblaze ABI
	r0  : 0
	r1  : stack pointer
	r2  : read_only small data area anchor
	r3 - r4 : return values/temporaries
	r5 - r10 : passing parameters/temporaries
	r11 - r12 : temporaries
	r13 : read-write small data area anchor
	r14 : return address for Interrupt
	r15 : return addres for sub-routine
	r16 : return address for trap
	r17 : return address for exceptions
	r18 : reserved for assembler/compiler temporaries
	r19 : callee-save
	r20 : callee-save. Reserved for storing a pointer to GOT(global offset table) in PIC.
	r21 - r31 : callee-save

    Callee saved: r19->r31
    Caller saved: r3->r12
    Arguments: r5 -> r10

*/

.macro MBZ_EXCEPT_SAVE
        // get context local storage
		lwi		r11, 	r0, 	__context_data_base

#ifdef MICROBLAZE_AREA_OPTIMIZED
        // nop if microblaze is area optimized, load take two cycles
		nop
#endif
		// get context registers save array
		addi	r12, 	r11,	mbz_context_regs
        // save "caller saved" registers yet (syscall args ragtable)
       	swi		r3, 	r12, 	CPU_MBZ_CONTEXT_GPR(3)
		swi		r4, 	r12, 	CPU_MBZ_CONTEXT_GPR(4)
		swi		r5, 	r12, 	CPU_MBZ_CONTEXT_GPR(5)
		swi		r6, 	r12, 	CPU_MBZ_CONTEXT_GPR(6)
		swi		r7, 	r12, 	CPU_MBZ_CONTEXT_GPR(7)
		swi		r8, 	r12, 	CPU_MBZ_CONTEXT_GPR(8)
		swi		r9, 	r12, 	CPU_MBZ_CONTEXT_GPR(9)
		swi		r10, 	r12, 	CPU_MBZ_CONTEXT_GPR(10)
		swi		r11, 	r12, 	CPU_MBZ_CONTEXT_GPR(11)
        // save gp ptr, stack ptr, frame ptr, return address
       	swi		r1, 	r12, 	CPU_MBZ_CONTEXT_GPR(1)
       	swi		r15, 	r12, 	CPU_MBZ_CONTEXT_GPR(15)
		// TODO : check if FPU registers have to be saved?

#if defined(CONFIG_HEXO_FPU) && !defined(CONFIG_RELEASE)
        // disable fpu access during exception handling
        mfmsr   0
        ori     0,      0,      PPC_MSR_FPU_ENABLED
        xori    0,      0,      PPC_MSR_FPU_ENABLED
        mtmsr   0
#endif


#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // save pc
        stw     pc,		r12,	CPU_MBZ_CONTEXT_PC
#endif

        // save status register
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
		mfs 	r10,  	rmsr
		swi 	r10, 	r12, 	CPU_MBZ_CONTEXT_MSR
#endif
)
.endm


/*
  The MBZ_EXCEPT_PREEMPT_CLEAR macro contains code to unregister preemption handler
  so that we can check on handler return if a new handler has been registered.
*/

.macro  MBZ_EXCEPT_PREEMPT_CLEAR
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // clear context preempt handler
        addi	r7,		r0,		cpu_preempt_handler
        swi 	r0,		r0,		r7
#endif
.endm


/*
  The MBZ_EXCEPT_FAULT macro contains code to call exception fault handlers
*/

.macro PPC_EXCEPT_FAULT type

        // prepare handler call
        li      r5,      \type           // arg0 type
        addi    r6,      r12,      	CPU_MBZ_CONTEXT_PC // arg1 execptr
        mfdear  r7                       // arg2 data error
        addi    r8,      r12,      	CPU_MBZ_CONTEXT_GPR(0) // arg3 reg table
        addi    r9,      r0,		r1     // arg4 sp, skip redzone

        // find right handler
#ifdef CONFIG_HEXO_USERMODE
        // from user mode ?
        andi.   r11,     r10,      	0x00000800	// MBZ_MSR_USERMODE
        beq     1f

        // get tls
        lwi     r8,     r0,		mbz_context_regs
        sub     r8,     r12,    r8

        // use kernel context stack when from user mode
        lw      r1,    r8,		context_stack_end
        addiu   r1,    r1,      -CONFIG_HEXO_STACK_ALIGN

        // use context local fault handler for user when available
        CONTEXT_LOCAL   lwz     cpu_user_exception_handler, 9, 10
        and.    9,      9,      9
        bne     2f
1:
#endif
        // use cpu local fault handler
		addi	r9,		r0,	 	cpu_exception_handler
2:

		brad	r9
.endm


/*
  The MBZ_EXCEPT_SYSCALL macro contains code to call syscall handlers
*/

.macro MBZ_EXCEPT_SYSCALL
#ifdef CONFIG_HEXO_USERMODE
        // from user mode ?
        andi.   r11,     r10,      	0x00000800	//MBZ_MSR_USERMODE
        beq     1f

        // get tls
        lwi     r8,     r0,		mbz_context_regs
        sub     r8,     r12,    r8

        // use kernel context stack when from user mode
        lw      r1,    r8,		context_stack_end
        addiu   r1,    r1,      -CONFIG_HEXO_STACK_ALIGN
1:

#endif
        CONTEXT_LOCAL   lwz     cpu_syscall_handler, 9, 10
        li      3,      0               // arg0 number
        addi    4,      2,      CPU_MBZ_CONTEXT_GPR(0) // arg1 reg table
        mtctr   9
        bctrl
.endm


/*
  The MBZ_EXCEPT_IRQ macro contains code to call syscall handlers
*/

.macro MBZ_EXCEPT_IRQ number
#ifdef CONFIG_HEXO_USERMODE
        // from user mode ?
        andi.   10,     8,      MBZ_MSR_USERMODE
        beq     1f

         // get tls
        lwi     r8,     r0,		mbz_context_regs
        sub     r8,     r12,    r8

        // use kernel context stack when from user mode
        lw      r1,    r8,		context_stack_end
        addiu   r1,    r1,      -CONFIG_HEXO_STACK_ALIGN
1:
#endif
#ifndef CONFIG_RELEASE
        // ensure we do not use context local storage in irq handler
        li      0,              0
        mtspr   PPC_SPRG(4),    0
#endif

//        CPU_LOCAL       lwz     cpu_interrupt_handler, 9, 10
		lwi		r9,		r0,		cpu_interrupt_handler
        addi    r5,      \number         // arg0 number

		brad	r9
#ifndef CONFIG_RELEASE
        // restore cls
        li      0,              mbz_context_regs@l
        sub     0,      2,      0
        mtspr   PPC_SPRG(4),    0
#endif
.endm


/*
  The MBZ_EXCEPT_PREEMPT macro contains code to call preemption handler,
  finish current context registers saving and switch to preempted context.
*/

.macro  MBZ_EXCEPT_PREEMPT
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT

        // check if a preempt handler has been registered
		lwi		r9,		r0,		cpu_interrupt_handler
        and.    r9,      r9,	r9
        beq     2f

        // call preempt handler
		brad	r9

        // got pointer to context to switch to ?
        and.    r3,      r3,      r3
        beq     2f

        // lets deals with all not yet saved registers

# ifdef CONFIG_HEXO_FPU
        // test fpu enabled for previous context
        lwz     r8,      r12,	CPU_MBZ_CONTEXT_MSR
        andi.   r7,      r8,    MBZ_MSR_FPU_ENABLED
        beq     1f

#  ifdef CONFIG_HEXO_LAZY_SWITCH
        // keep track of last context using fpu on this processor
        CPU_LOCAL       stw     ppc_lazy_last, 2, 10
#  endif

        // save fpu registers 0 to 31
        SMD_FPU 2,      0,      31
        // save fpu status
        mffs    0
        stfd    0,              CPU_MBZ_CONTEXT_FPSCR(2)
1:
# endif

        // save callee saved registers r19 to r31
		swi		r19, 	r12, 	CPU_MBZ_CONTEXT_GPR(19)
		swi		r20, 	r12, 	CPU_MBZ_CONTEXT_GPR(20)
		swi		r21, 	r12, 	CPU_MBZ_CONTEXT_GPR(21)
		swi		r22, 	r12, 	CPU_MBZ_CONTEXT_GPR(22)
		swi		r23, 	r12, 	CPU_MBZ_CONTEXT_GPR(23)
		swi		r24, 	r12, 	CPU_MBZ_CONTEXT_GPR(24)
		swi		r25, 	r12, 	CPU_MBZ_CONTEXT_GPR(25)
		swi		r26, 	r12, 	CPU_MBZ_CONTEXT_GPR(26)
		swi		r27, 	r12, 	CPU_MBZ_CONTEXT_GPR(27)
		swi		r28, 	r12, 	CPU_MBZ_CONTEXT_GPR(28)
		swi		r29, 	r12, 	CPU_MBZ_CONTEXT_GPR(29)
		swi		r30, 	r12, 	CPU_MBZ_CONTEXT_GPR(30)
		swi		r31, 	r12, 	CPU_MBZ_CONTEXT_GPR(31)
		mfs 	r31,  	rmsr
		swi 	r31, 	r12, 	CPU_MBZ_CONTEXT_MSR

		addi	r31, 	r0,		CPU_MBZ_CONTEXT_RESTORE_CALLEE
		swi		r31, 	r12,	CPU_MBZ_CONTEXT_SAVE_MASK

        // set save mask
        lwi     r20,	r0,     CPU_MBZ_CONTEXT_RESTORE_CALLER | CPU_MBZ_CONTEXT_RESTORE_CALLEE
        swi     r20,	r0,     CPU_MBZ_CONTEXT_SAVE_MASK

        // switch to new context
        brad    cpu_context_jumpto
2:
#endif
.endm


/*
  The MBZ_EXCEPT_RESTORE macro contains code to restore "caller saved" registers
  and return to interrupted code.
*/

.macro  MBZ_EXCEPT_RESTORE

        // restore caller saved counter register

        // restore caller saved gp registers 0 - 13
		lwi		r3, 	r12, 	CPU_MBZ_CONTEXT_GPR(3)
		lwi		r4, 	r12, 	CPU_MBZ_CONTEXT_GPR(4)
		lwi		r5, 	r12, 	CPU_MBZ_CONTEXT_GPR(5)
		lwi		r6, 	r12, 	CPU_MBZ_CONTEXT_GPR(6)
		lwi		r7, 	r12, 	CPU_MBZ_CONTEXT_GPR(7)
		lwi		r8, 	r12, 	CPU_MBZ_CONTEXT_GPR(8)
		lwi		r9, 	r12, 	CPU_MBZ_CONTEXT_GPR(9)
		lwi		r10, 	r12, 	CPU_MBZ_CONTEXT_GPR(10)
		lwi		r11, 	r12, 	CPU_MBZ_CONTEXT_GPR(11)

		rtid	r14, 0

.endm

