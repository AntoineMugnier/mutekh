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

#include <hexo/asm.h>
#include <hexo/cpu.h>
#include <hexo/context.h>

.macro MBZ_IRQ_EXCEPT_SAVE
        // get context local storage
		lwi		r11, 	r0, 	__context_data_base

#ifdef MICROBLAZE_AREA_OPTIMIZED
		nop   					 nop if microblaze is area optimized, load take two cycles
#endif
		// get context registers save array
		addi	r12, 	r11,	mbz_context_regs
        // save "caller saved" registers yet (syscall args ragtable)
        swi		r1, 	r12, 	CPU_MBZ_CONTEXT_GPR(1)  // save stack pointer
       	swi		r3, 	r12, 	CPU_MBZ_CONTEXT_GPR(3)
		swi		r4, 	r12, 	CPU_MBZ_CONTEXT_GPR(4)
		swi		r5, 	r12, 	CPU_MBZ_CONTEXT_GPR(5)
		swi		r6, 	r12, 	CPU_MBZ_CONTEXT_GPR(6)
		swi		r7, 	r12, 	CPU_MBZ_CONTEXT_GPR(7)
		swi		r8, 	r12, 	CPU_MBZ_CONTEXT_GPR(8)
		swi		r9, 	r12, 	CPU_MBZ_CONTEXT_GPR(9)
		swi		r10, 	r12, 	CPU_MBZ_CONTEXT_GPR(10)
		swi		r11, 	r12, 	CPU_MBZ_CONTEXT_GPR(11)
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // save pc = r14 si interruption
        swi     r14,	r12,	CPU_MBZ_CONTEXT_PC
        swi     r14,	r12,	CPU_MBZ_CONTEXT_GPR(14)
#endif
       	swi		r15, 	r12, 	CPU_MBZ_CONTEXT_GPR(15)	// and return address
       	swi		r18, 	r12, 	CPU_MBZ_CONTEXT_GPR(18)
       	swi		r19, 	r12, 	CPU_MBZ_CONTEXT_GPR(19)

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // save status register
		mfs 	r10,  	rmsr
		swi 	r10, 	r12, 	CPU_MBZ_CONTEXT_MSR

#endif

.endm


/*
  The MBZ_IRQ_EXCEPT_PREEMPT_CLEAR macro contains code to unregister preemption handler
  so that we can check on handler return if a new handler has been registered.
*/

.macro  MBZ_IRQ_EXCEPT_PREEMPT_CLEAR
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // clear context preempt handler
        addi	r7,		r0,		cpu_preempt_handler
        swr 	r0,		r0,		r7
#endif
.endm


/*
  The MBZ_EXCEPT_FAULT macro contains code to call exception fault handlers
*/

.macro MBZ_EXCEPT_FAULT type

        // prepare handler call
        addi    r5,      r0,	\type           		// arg0 type
        mfs     r6,      rear 							// arg1 execptr
        mfs 	r7,		 redr							// arg2 data error
        addi    r8,      r12,	CPU_MBZ_CONTEXT_GPR(0)  // arg3 reg table
        addi    r9,      r0,	32     					// arg4 sp, skip redzone 32 oct =r5 to r10-r15-r19 A voir

        // find right handler
#ifdef CONFIG_HEXO_USERMODE
        // from user mode ?
        andi    r11,     r10,	MBZ_MSR_USERMODE
        beqid   r11,	 1f
		nop
        // get tls
        lwi     r8,     r0,		mbz_context_regs
        sub     r8,     r12,	r8

        // use kernel context stack when from user mode
        lwi     r1,    r8,		context_stack_end
        addi    r1,    r1,		-CONFIG_HEXO_STACK_ALIGN

        // use context local fault handler for user when available
        lwi		r10,	 r0,	cpu_user_exception_handler
        and     r10,      r10,	r10
        bneid   r10,      2f			// branch if cpu_user_exception_handler = r9 != 0
	    nop

1:
#endif
        // use cpu local fault handler
		lwi		r10,	r0,		cpu_exception_handler
2:
		addik	r1,	r1,-32
		brald 	r15,	r10
		nop
		addik	r1,	r1,32
.endm


/*
  The MBZ_EXCEPT_SYSCALL macro contains code to call syscall handlers
*/
.macro MBZ_EXCEPT_SYSCALL
#ifdef CONFIG_HEXO_USERMODE
        // from user mode ?
        andi   r11,     r10,	MBZ_MSR_USERMODE
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
  The MBZ_EXCEPT_IRQ macro contains code to call interrupt handlers
*/
.macro MBZ_EXCEPT_IRQ
#ifdef CONFIG_HEXO_USERMODE
        // from user mode ?
        andi    r10,     r10,	MBZ_MSR_USERMODE
        beq     1f

         // get tls
        lwi     r8,     r0,		mbz_context_regs
        sub     r8,     r12,    r8

        // use kernel context stack when from user mode
        lw      r1,    r8,		context_stack_end
        addiu   r1,    r1,      -CONFIG_HEXO_STACK_ALIGN
1:
#endif

		addik	r1, r1, -28			/* allocate enough space to store context onto stack */
		swi		r30, r1, 0
		swi		r29, r1, 4
		swi		r28, r1, 8
		swi		r27, r1, 12

		swi		r20, r1, 16
		swi		r5 , r1, 20
		swi		r19, r1, 24

		/* Identify Interrupt source */
		ori	r21, r0, XPAR_XPS_INTC_0_BASEADDR
		lwi	r19, r21, XIN_ISR_OFFSET 	/* load ISR INTC*/
		lwi	r20, r21, XIN_IER_OFFSET	/* load IER INTC */
		addi 	r5, r0, 0			/* r5 contains Interrupt vector */
		addi	r31, r0, 1			/* ISR mask */
		addi	r30, r0, 1 			/* IER mask */

$L1:
		and		r29, r31, r19			/* Check ISR */
		and 	r28, r30, r20			/* Check IER */

		and		r29, r29, r28			/* Check if interrupt is active  */
		bneid	r29, $L2			/* If yes goto $L2*/
		nop

		addi	r5,r5,1				/* If no, increment vector number */
		muli	r31,r31,2			/* ISR mask 1 left shift */
		muli	r30,r30,2			/* IER mask 1 left shift */

		addik	r27,r0,32			/* if Vector equal 31, end*/
		cmp		r27,r5,r27
		bneid	r27,$L1
		nop

		/*	r5 contains 32 if no bits are set in ISR */
		/*	r31 contains (1<<r5), suitable to acknowledge detected Interrupt */

$L2:
		lwi		r30, r1, 0
		lwi		r29, r1, 4
		lwi		r28, r1, 8
		lwi		r27, r1, 12

		lwi		r20, r1, 16
		lwi		r5 , r1, 20
		lwi		r19, r1, 24

		addik	r1, r1,  28

		/* Call cpu_interrupt_handler */
		/* r5 contains the interrupt vector number [0..31] */
		/* r31 contains mask for IT ack */
		/* r21 contains address of interrupt controller ack register */

		/* save context to call handler */
		/* allocate enough space to store context onto stack */
		addik	r1, r1, -12
		swi		r31, r1, 0
		swi		r21, r1, 4

#ifdef MICROBLAZE_AREA_OPTIMIZED
		nop   					/* nop if microblaze is area optimized, store takes two cycles */
#endif
/*********/

		lwi		r9,		r0,		cpu_interrupt_handler

		addik	r1,	r1,-32
		brald	r15,	r9
		nop
		addik	r1,	r1,32

		/* restore context to ack the it */
		/* the it is acked after calling the handler to be sure it source was cleared */

		lwi		r31, r1, 0
		lwi		r21, r1, 4
#ifdef MICROBLAZE_AREA_OPTIMIZED
		nop   								/* nop if microblaze is area optimized, store takes two cycles */
#endif
		addik	r1, r1, 12					/* allocate enough space to store context onto stack */
		swi		r31, r21, XIN_IAR_OFFSET	/* ack the it */

.endm




/*
  The MBZ_IRQ_EXCEPT_PREEMPT macro contains code to call preemption handler,
  finish current context registers saving and switch to preempted context.
*/
.macro  MBZ_IRQ_EXCEPT_PREEMPT
#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
        // check if a preempt handler has been registered
		lwi		r9,		 r0,		cpu_preempt_handler
        and     r9,      r9,		r9
        beqid   r9,      2f			// branch if cpu_preempt_handler = r9 = 0
	    nop

        // if r9 != 0 : call preempt handler
		addik	r1,	r1,-32
        brald	r15,	r9
		nop
		addik	r1,	r1,32

        // got pointer to context to switch to ?
        and     r3,      r3,      r3
        beqid   r3, 	 2f 		// branch if no context to switch to : r3 = 0
		nop
        // lets deals with all not yet saved registers

#ifdef CONFIG_HEXO_FPU
        // test fpu enabled for previous context:
        // For FPU registers = General registers for microblaze

#ifdef CONFIG_HEXO_LAZY_SWITCH
        // keep track of last context using fpu on this processor
        // CPU_LOCAL       stw     ppc_lazy_last, 2, 10
#endif


1:
#endif
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

        // set save mask to know we come from interrupt
        addik   r20,	r0,     CPU_MBZ_CONTEXT_RESTORE_CALLER | CPU_MBZ_CONTEXT_RESTORE_CALLEE
        swi     r20,	r12,    CPU_MBZ_CONTEXT_SAVE_MASK

		addk 	r5,		r0,		r3

        // switch to new context
        brid    cpu_context_jumpto
        nop

2:

#endif
.endm


/*
  The MBZ_IRQ_EXCEPT_RESTORE macro contains code to restore "caller saved" registers
  and return to interrupted code.
*/
.macro  MBZ_IRQ_EXCEPT_RESTORE

        // restore stack ptr
       	lwi		r1, 	r12, 	CPU_MBZ_CONTEXT_GPR(1)

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
        // restore return address
       	lwi		r15, 	r12, 	CPU_MBZ_CONTEXT_GPR(15)

 		rtid	r14, 	0
   		/* ATTENTION : prévoir rted si exception */
       	lwi		r19, 	r12, 	CPU_MBZ_CONTEXT_GPR(19)
       	lwi		r18, 	r12, 	CPU_MBZ_CONTEXT_GPR(18)

.endm



FUNC_START(.text,_hw_exception_handler)
bri _hw_exception_handler
#if 0
		MBZ_IRQ_EXCEPT_SAVE
		MBZ_IRQ_EXCEPT_PREEMPT_CLEAR

		LWI		R10,	R0,		_exc_from
		ADDIK	R1,		R1,		-32
		BRALD 	R15,	R10
		NOP
		ADDIK	R1,		R1,		 32

		MBZ_IRQ_EXCEPT_PREEMPT
		MBZ_IRQ_EXCEPT_RESTORE
#endif
FUNC_END(_hw_exception_handler)



FUNC_START(.text,_exception_handler)
bri _exception_handler
FUNC_END(_exception_handler)



FUNC_START(.text,_interrupt_handler)
		MBZ_IRQ_EXCEPT_SAVE
		MBZ_IRQ_EXCEPT_PREEMPT_CLEAR
#ifdef CONFIG_HEXO_IRQ
		MBZ_EXCEPT_IRQ
#endif
		MBZ_IRQ_EXCEPT_PREEMPT
		MBZ_IRQ_EXCEPT_RESTORE
FUNC_END(_interrupt_handler)

