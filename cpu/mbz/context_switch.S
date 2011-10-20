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
#include <hexo/cpu.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

/* Microblaze ABI
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
    Arguments: r5
 */


FUNC_START(.text,cpu_context_switch)
	// get context local storage
	lwi		r11, 	r0, 	__context_data_base

#ifdef MICROBLAZE_AREA_OPTIMIZED
	nop   					/* nop if microblaze is area optimized, load take two cycles */
#endif
	// get context registers save array
	addi	r12, 	r11,	mbz_context_regs
	// save stack pointer
	swi 	r1, 	r12, 	CPU_MBZ_CONTEXT_SP

	// save return address for Interrupt register
	swi		r14, 	r12, 	CPU_MBZ_CONTEXT_GPR(14)
	// save return address for Sub routine register
	swi		r15, 	r12, 	CPU_MBZ_CONTEXT_GPR(15)
	// resume directly at return address + 8
	addi	r15,	r15,	8							/* pour restit contexte car on sort par               */
														/* cpu_context_switch_jumpto donc r17 avec offset nul */
	swi 	r15, 	r12, 	CPU_MBZ_CONTEXT_PC		 	/* store @ return to pc interrupted task 	          */
	//save callee-save registers
	swi		r18, 	r12, 	CPU_MBZ_CONTEXT_GPR(18)
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

	swi		r5, 	r12, 	CPU_MBZ_CONTEXT_GPR(5)
	swi		r4, 	r12, 	CPU_MBZ_CONTEXT_GPR(4)
	swi		r3, 	r12, 	CPU_MBZ_CONTEXT_GPR(3)

	addi	r31, 	r0,		CPU_MBZ_CONTEXT_RESTORE_CALLEE
	swi		r31, 	r12,	CPU_MBZ_CONTEXT_SAVE_MASK


#ifdef MICROBLAZE_AREA_OPTIMIZED
	nop   					/* nop if microblaze is area optimized, store take two cycles */
#endif

	cpu_context_switch_jumpto:
	// restore tls pointer from context struct (r5 argument)
	lwi		r11, 	r5, 	0
	addi 	r12, 	r0, 	__context_data_base
	sw	 	r11,	r0,		r12

	addi 	r12, 	r11, 	mbz_context_regs
	// restore stack pointer
	lwi 	r1,  	r12, 	CPU_MBZ_CONTEXT_SP

	// restore return address for Interrupt register
	lwi		r14, 	r12, 	CPU_MBZ_CONTEXT_GPR(14)

	//restore callee-save registers
	lwi		r18, 	r12, 	CPU_MBZ_CONTEXT_GPR(18)
	lwi		r19, 	r12, 	CPU_MBZ_CONTEXT_GPR(19)
	lwi		r20, 	r12, 	CPU_MBZ_CONTEXT_GPR(20)
	lwi		r21, 	r12, 	CPU_MBZ_CONTEXT_GPR(21)
	lwi		r22, 	r12, 	CPU_MBZ_CONTEXT_GPR(22)
	lwi		r23, 	r12, 	CPU_MBZ_CONTEXT_GPR(23)
	lwi		r24, 	r12, 	CPU_MBZ_CONTEXT_GPR(24)
	lwi		r25, 	r12, 	CPU_MBZ_CONTEXT_GPR(25)
	lwi		r26, 	r12, 	CPU_MBZ_CONTEXT_GPR(26)
	lwi		r27, 	r12, 	CPU_MBZ_CONTEXT_GPR(27)
	lwi		r28, 	r12, 	CPU_MBZ_CONTEXT_GPR(28)
	lwi		r29, 	r12, 	CPU_MBZ_CONTEXT_GPR(29)
	lwi		r30, 	r12, 	CPU_MBZ_CONTEXT_GPR(30)
	lwi   	r31, 	r12, 	CPU_MBZ_CONTEXT_MSR			/* restitute status register MSR and return from*/
	mts		rmsr,  	r31									/* exception register r17 for operation mode    */
	lwi		r31, 	r12, 	CPU_MBZ_CONTEXT_GPR(31)		/* restoring : user/priv						*/
	lwi		r5, 	r12, 	CPU_MBZ_CONTEXT_GPR(5)		/* load passing argument 						*/
	lwi 	r17, 	r12, 	CPU_MBZ_CONTEXT_PC			/* load stored PC to RA for exception register	*/
	lwi 	r15, 	r12, 	CPU_MBZ_CONTEXT_GPR(15)		/* load stored RA to RA register 				*/

	lwi 	r3, 	r12,	CPU_MBZ_CONTEXT_SAVE_MASK
#ifdef MICROBLAZE_AREA_OPTIMIZED
	nop   						/* nop if microblaze is area optimized, load take two cycles 	*/
#endif
	andi	r4		r3,		CPU_MBZ_CONTEXT_RESTORE_CALLER
	beqid	r4,		1f		/* saut si r4=0 */
	nop

	// mettre ici les caller-save registers
	lwi		r3, 	r12, 	CPU_MBZ_CONTEXT_GPR(3)
	lwi		r4, 	r12, 	CPU_MBZ_CONTEXT_GPR(4)
	lwi		r5, 	r12, 	CPU_MBZ_CONTEXT_GPR(5)
	lwi		r6, 	r12, 	CPU_MBZ_CONTEXT_GPR(6)
	lwi		r7, 	r12, 	CPU_MBZ_CONTEXT_GPR(7)
	lwi		r8, 	r12, 	CPU_MBZ_CONTEXT_GPR(8)
	lwi		r9, 	r12, 	CPU_MBZ_CONTEXT_GPR(9)
	lwi		r10, 	r12, 	CPU_MBZ_CONTEXT_GPR(10)
	lwi		r11, 	r12, 	CPU_MBZ_CONTEXT_GPR(11)

   // For FPU registers = General registers for microblaze

#ifdef MICROBLAZE_AREA_OPTIMIZED
	nop   						/* nop if microblaze is area optimized, load take two cycles 	*/
#endif

	/* ici on sort en venant d'une IT car CPU_MBZ_CONTEXT_RESTORE_CALLER 	*/
	/* donc il faut un RTID qui remet le bit IE de MSR à 1 					*/
	rtid    r17,	0
 	/* ATTENTION : prévoir rted si exception */
	nop

1:

	lwi		r3, 	r12, 	CPU_MBZ_CONTEXT_GPR(3)
	lwi		r4, 	r12, 	CPU_MBZ_CONTEXT_GPR(4)
	// offset 0 car ici on se branche directement à l'adresse contenue dans r17

	/* ici on sort en venant d'un appel fonction en C, pas d'une IT */
	/* donc il ne faut pas un RTID qui remet le bit IE de MSR à 1 */
	rtsd    r17,	0
 	/* ATTENTION : prévoir rted si exception */
	nop
FUNC_END(cpu_context_switch)




FUNC_START(.text,cpu_context_jumpto)

	brid 	 cpu_context_switch_jumpto
	nop
FUNC_END(cpu_context_jumpto)




FUNC_START(.text, cpu_context_set)

	lwi		r11, 	r5, 	0							/* stack					  					*/
	addk	r11, 	r6, 	r0							/* + stack size 								*/
	addi    r1,     r11, 	-CONFIG_HEXO_STACK_ALIGN	/* put to stack    								*/
	brad	r7											/* jump to *jumpto 								*/
	nop
FUNC_END(cpu_context_set)


FUNC_START(.text, cpu_context_stack_use)

	// get tls pointer from context struct
	lwi 	r8,		r5, 	0
	// get context registers save array
	addi	r8,		r8,		OFFSET_MBZ_CONTEXT_REG

	addi 	r8,		r8,		CPU_MBZ_CONTEXT_GPR(1)
	lwi 	r1,		r8,		0

#ifndef CONFIG_RELEASE
	// prevent use of previous context local storage
	// CPU_LOCAL stw,   __context_data_base, zero, r1  version d' origine
#endif

	// private param
	add 	r5,		r7,		r0

	// call function
	bra		r6
	nop

FUNC_END(cpu_context_stack_use)

