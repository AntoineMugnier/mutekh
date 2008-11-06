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
    ".section        .excep,\"ax\",@progbits		\n"

    "b	1f						\n"
    ".space 0x17c					\n"

    ".globl mips_interrupt_entry			\n"
    "mips_interrupt_entry:				\n"
    ".set push						\n"
    ".set noat						\n"

    "1:							\n"

    /* restore cpu local storage */
#if defined(CONFIG_CPU_USER)
# if defined(CONFIG_SMP)
#  if __mips >= 32 
    "	mfc0	$26,	$15,	1			\n"
#  else
    "	mfc0	$26,	$15				\n"
#  endif
    "	andi	$26,	$26,	0x3ff			\n"

    "	sll	$26,	$26,	2			\n"
    "	lw	$27,	cpu_local_storage($26)		\n"
# endif

    /* event from user mode ? */
    "	mfc0	$26,	$12				\n"
    "	andi	$26,	$26,	0x8			\n"

    ".set noreorder					\n"
    "	blez	$26,	1f				\n"
    "	move	$26,	$sp				\n"
    ".set reorder					\n"

    /* restore kernel stack ! */

# ifdef CONFIG_SMP
    "	lw	$sp,	__cpu_context_data_base($27)	\n"
# else
    "	lw	$sp,	__cpu_context_data_base		\n"
# endif
    "	addiu	$sp,	%lo(context_kstack)		\n"
    "	lw	$sp,	($sp)				\n"

#else  /* !defined(CONFIG_CPU_USER) */

    "	move	$26,	$sp				\n"

#endif

    /* save registers usefull to syscall */
    "1:							\n"
    "	addu	$sp,	-4*32				\n"
    "	sw	$26,	29*4($sp)			\n"

    "	sw	$4,	 4*4($sp)		        \n" /* Args regs */
    "	sw	$5,	 5*4($sp)		        \n" /* Args regs */
    "	sw	$6,	 6*4($sp)		        \n" /* Args regs */
    "	sw	$7,	 7*4($sp)		        \n" /* Args regs */

    "	sw	$gp,	28*4($sp)		        \n" /* Global pointer */
    "   la	$gp,   _gp				\n" /* restore kernel $gp */

    "	sw	$31,	31*4($sp)		        \n" /* Return address regs */

    /* read and extract cause */
    "	mfc0	$4,	$13				\n"
    "	andi	$6,	$4,	0x3c			\n"

    /* read & save EPC */
    "	mfc0	$5,	$14				\n"
    "	sw	$5,	0*4($sp)		        \n"

    "	li	$7,	32				\n"
    "	beq	$6,	$7,	interrupt_sys		\n"

    /* save extra registers for hw interrupts and exceptions only */

    "	sw	$1,	 1*4($sp)		        \n" /* AT reg */

    "	sw	$2,	 2*4($sp)		        \n" /* Return value regs */
    "	sw	$3,	 3*4($sp)		        \n" /* Return value regs */

    "	sw	$8,	 8*4($sp)		        \n" /* Temp regs */
    "	sw	$9,	 9*4($sp)		        \n" /* Temp regs */
    "	sw	$10,	10*4($sp)		        \n" /* Temp regs */
    "	sw	$11,	11*4($sp)		        \n" /* Temp regs */
    "	sw	$12,	12*4($sp)		        \n" /* Temp regs */
    "	sw	$13,	13*4($sp)		        \n" /* Temp regs */
    "	sw	$14,	14*4($sp)		        \n" /* Temp regs */
    "	sw	$15,	15*4($sp)		        \n" /* Temp regs */

    "	sw	$16,	16*4($sp)		        \n" /* Callee saved */
    "	sw	$17,	17*4($sp)		        \n" /* Callee saved */
    "	sw	$18,	18*4($sp)		        \n" /* Callee saved */
    "	sw	$19,	19*4($sp)		        \n" /* Callee saved */
    "	sw	$20,	20*4($sp)		        \n" /* Callee saved */
    "	sw	$21,	21*4($sp)		        \n" /* Callee saved */
    "	sw	$22,	22*4($sp)		        \n" /* Callee saved */
    "	sw	$23,	23*4($sp)		        \n" /* Callee saved */

    "	sw	$30,	30*4($sp)		        \n" /* Callee saved */

    "	sw	$24,	24*4($sp)		        \n" /* Temp regs */
    "	sw	$25,	25*4($sp)		        \n" /* Temp regs */

    "	beq	$6,	$0,	interrupt_hw		\n"

    /*************************************************************
		exception handling
    **************************************************************/

    "interrupt_ex:					\n"

    /* exception function arguments */
    "	srl	$4,	$6,	2			\n" /* adjust cause arg */
//  "	move	$5,	$5				\n" /* execution pointer */
    "	mfc0	$6,	$8				\n" /* bad address if any */
    "	addiu	$7,	$sp,	0			\n" /* register table on stack */

    "	addiu	$sp,	$sp,	-5*4			\n"
    "	sw	$26,	4*4($sp)			\n"
#ifdef CONFIG_SMP
    "	lw	$1,	cpu_exception_handler($27)	\n"
#else
    "	lw	$1,	cpu_exception_handler		\n"
#endif
    "	jalr	$1					\n"
    "	addiu	$sp,	$sp,	5*4			\n"

    "	lw	$26,	 0*4($sp)		        \n" /* get EPC value */

    "	j	return					\n"

    /*************************************************************
		syscall handling
    **************************************************************/
    "interrupt_sys:					\n"

    "	move	$4,	$0				\n" /* single trap on mips: id = 0 */
    "	addiu	$5,	$sp,	0			\n" /* register table on stack */
    "	addiu	$sp,	$sp,	-4*4			\n"
#ifdef CONFIG_SMP
    "	lw	$11,	__cpu_context_data_base($27)	\n"
#else
    "	lw	$11,	__cpu_context_data_base		\n"
#endif
    "	lw	$1,	    cpu_syscall_handler($11)	\n"
    "	jalr	$1					\n"
    "	addiu	$sp,	$sp,	4*4			\n"

    "	lw	$26,	0*4($sp)		        \n" /* get EPC value */
    "   addiu	$26,	4				\n" /* increment epc for not doing the syscall again */

#ifdef CONFIG_SYSCALL_CLEAN_REGS
    /* FIXME cleanup all caller saved registers here */
#endif

    "	j	return_val				\n"

    /*************************************************************
		hardware interrupts handling
    **************************************************************/
    "interrupt_hw:					\n"

    "	srl	$4,	$4,	8			\n" /* hw interrupt line id */
    "	andi	$4,	$4,	0xff			\n"

    "	addiu	$sp,	$sp,	-4*4			\n"
#ifdef CONFIG_SMP
    "	lw	$1,	cpu_interrupt_handler($27)	\n"
#else
    "	lw	$1,	cpu_interrupt_handler		\n"
#endif
    "	jalr	$1					\n"
    "	addiu	$sp,	$sp,	4*4			\n"

    "	lw	$26,	 0*4($sp)		        \n" /* get EPC value */

    /************************************************************/

    /* restore registers */
    "return:						\n"

    "	lw	$1,	 1*4($sp)		        \n"

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

    "	lw	$16,	 16*4($sp)		        \n"
    "	lw	$17,	 17*4($sp)		        \n"
    "	lw	$18,	 18*4($sp)		        \n"
    "	lw	$19,	 19*4($sp)		        \n"
    "	lw	$20,	 20*4($sp)		        \n"
    "	lw	$21,	 21*4($sp)		        \n"
    "	lw	$22,	 22*4($sp)		        \n"
    "	lw	$23,	 23*4($sp)		        \n"

    "	lw	$30,	 30*4($sp)		        \n"

    "	lw	$24,	24*4($sp)		        \n"
    "	lw	$25,	25*4($sp)		        \n"

    "return_val:					\n"

    "	lw	$2,	 2*4($sp)		        \n" /* Syscall return value */
    "	lw	$3,	 3*4($sp)		        \n" /* Syscall return value */

    "	lw	$28,	 28*4($sp)		        \n" /* restore user GP */
    "	lw	$31,	31*4($sp)		        \n" /* restore return address */

    "	lw	$sp,	29*4($sp)		        \n" /* restore user stack */

    ".set noreorder					\n"
    "	jr	$26					\n"
    "	rfe						\n"

    ".set pop						\n"
    );

