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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

asm(
    ".section        .excep,\"ax\",@progbits			\n"

	".space 0x180    \n"

    ".globl mips_interrupt_entry				\n"
    "mips_interrupt_entry:					\n"
    ".set push							\n"
    ".set noat							\n"

    /* save registers */
    "	addu	$sp,	-4*32				\n"

    "	sw	$1,	 1*4($sp)		        \n" /* AT reg */

    "	sw	$2,	 2*4($sp)		        \n" /* Return value regs */
    "	sw	$3,	 3*4($sp)		        \n" /* Return value regs */

    "	mfc0	$1,	$13				\n" /* read Cause */

    "	sw	$4,	 4*4($sp)		        \n" /* Args regs */
    "	sw	$5,	 5*4($sp)		        \n" /* Args regs */
    "	sw	$6,	 6*4($sp)		        \n" /* Args regs */
    "	sw	$7,	 7*4($sp)		        \n" /* Args regs */

    "	sw	$8,	 8*4($sp)		        \n" /* Temp regs */
    "	sw	$9,	 9*4($sp)		        \n" /* Temp regs */
    "	sw	$10,	10*4($sp)		        \n" /* Temp regs */
    "	sw	$11,	11*4($sp)		        \n" /* Temp regs */

    "	mfc0	$8,	$14				\n" /* read EPC */

    "	sw	$12,	12*4($sp)		        \n" /* Temp regs */
    "	sw	$13,	13*4($sp)		        \n" /* Temp regs */
    "	sw	$14,	14*4($sp)		        \n" /* Temp regs */
    "	sw	$15,	15*4($sp)		        \n" /* Temp regs */
    "	sw	$24,	24*4($sp)		        \n" /* Temp regs */
    "	sw	$25,	25*4($sp)		        \n" /* Temp regs */

    "	sw	$31,	31*4($sp)		        \n" /* Return address regs */

    "	sw	$8,	0*4($sp)		        \n" /* EPC reg */

    "	addiu	$11,	$sp,	4*32			\n" /* stack pointer */
    "	sw	$11,	29*4($sp)			\n"

    "	andi	$9,	$1,	0x3c			\n" /* extract cause */

    "	li	$10,	32				\n"
    "	beq	$9,	$10,	interrupt_sys		\n"

    /* save extra register for int/except only */
    "	sw	$16,	16*4($sp)		        \n"
    "	sw	$17,	17*4($sp)		        \n"
    "	sw	$18,	18*4($sp)		        \n"
    "	sw	$19,	19*4($sp)		        \n"
    "	sw	$20,	20*4($sp)		        \n"
    "	sw	$21,	21*4($sp)		        \n"
    "	sw	$22,	22*4($sp)		        \n"
    "	sw	$23,	23*4($sp)		        \n"
    "	sw	$26,	26*4($sp)		        \n"
    "	sw	$27,	27*4($sp)		        \n"
    "	sw	$28,	28*4($sp)		        \n"
    "	sw	$30,	30*4($sp)		        \n"

    "	beq	$9,	$0,	interrupt_hw		\n"

    /*************************************************************
		exception handling
    **************************************************************/

    "interrupt_ex:					\n"

    /* exception function arguments */
    "	srl	$4,	$9,	2			\n" /* adjust cause arg */
    "	move	$5,	$8				\n" /* execution pointer */
    "	mfc0	$6,	$8				\n" /* bad address if any */
    "	addiu	$7,	$sp,	0			\n" /* register table on stack */

    "	addiu	$sp,	$sp,	-5*4			\n"
    "	sw	$11,	4*4($sp)			\n"
    "	lw	$1,	cpu_exception_handler($27)	\n"
    "	jalr	$1					\n"
    "	addiu	$sp,	$sp,	5*4			\n"

    "	j	return					\n"

    /*************************************************************
		syscall handling
    **************************************************************/
    "interrupt_sys:					\n"

    "	addiu	$sp,	$sp,	-4*4			\n"
    "	lw	$11,	__cpu_context_data_base($27)	\n"
    "	lw	$1,	    cpu_syscall_handler($11)	\n"
    "	jalr	$1					\n"
    "	addiu	$sp,	$sp,	4*4			\n"
    "	lw	$8,	    0*4($sp)		        \n" /* EPC reg */
    "   addiu $8, 4                 \n" /* increment epc for not doing the syscall again */
    "	sw	$8,	    0*4($sp)		        \n"

    "	j	return_val				\n"

    /*************************************************************
		hardware interrupts handling
    **************************************************************/
    "interrupt_hw:					\n"

    "	srl	$4,	$1,	8			\n"
    "	andi	$4,	$4,	0xff			\n"

    "	addiu	$sp,	$sp,	-4*4			\n"
    "	lw	$1,	cpu_interrupt_handler($27)	\n"
    "	jalr	$1					\n"
    "	addiu	$sp,	$sp,	4*4			\n"

    /************************************************************/

    /* restore registers */
    "return:						\n"
    "	lw	$16,	 16*4($sp)		        \n"
    "	lw	$17,	 17*4($sp)		        \n"
    "	lw	$18,	 18*4($sp)		        \n"
    "	lw	$19,	 19*4($sp)		        \n"
    "	lw	$20,	 20*4($sp)		        \n"
    "	lw	$21,	 21*4($sp)		        \n"
    "	lw	$22,	 22*4($sp)		        \n"
    "	lw	$23,	 23*4($sp)		        \n"
    "	lw	$26,	 26*4($sp)		        \n"
    "	lw	$27,	 27*4($sp)		        \n"
    "	lw	$28,	 28*4($sp)		        \n"
    "	lw	$30,	 30*4($sp)		        \n"

    "	lw	$2,	 2*4($sp)		        \n"
    "	lw	$3,	 3*4($sp)		        \n"

    "return_val:					\n"
    "	lw	$1,	 1*4($sp)		        \n"

    "	lw	$26,	 0*4($sp)		        \n" /* EPC reg */

    "	lw	$4,	 4*4($sp)		        \n"
    "	lw	$5,	 5*4($sp)		        \n"
    "	lw	$6,	 6*4($sp)		        \n"
    "	lw	$7,	 7*4($sp)		        \n"

    "	lw	$8,	 8*4($sp)		        \n"
    "	lw	$9,	 9*4($sp)		        \n"
    "	lw	$10,	10*4($sp)		        \n"
    "	lw	$11,	11*4($sp)		        \n"
    "	lw	$12,	12*4($sp)		        \n"
    "	lw	$13,	13*4($sp)		        \n"
    "	lw	$14,	14*4($sp)		        \n"
    "	lw	$15,	15*4($sp)		        \n"

    "	lw	$24,	24*4($sp)		        \n"
    "	lw	$25,	25*4($sp)		        \n"

    "	lw	$31,	31*4($sp)		        \n"

    "	lw	$sp,	29*4($sp)		        \n"
//    "	addu	$sp,	4*32				\n"

    ".set noreorder					\n"
    "	jr	$26					\n"
    "	rfe						\n"

    ".set pop						\n"
    );

