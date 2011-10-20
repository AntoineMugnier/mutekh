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

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif


	.globl _start

/*	Set the exception and interrupt address vectors    */
/*	to jump to the appropriate handlers                */

	.align 2
	.ent _start
	_start:
        bri     _start1                 # 0x00
        nop                             # 0x04
        nop                             # 0x08          # Reserve space for software exception vector
        nop                             # 0x0c
        nop                             # 0x10          # Reserve space for interrupt vector
        nop                             # 0x14
        nop                             # 0x18          # Reserve space for breakpoint vector
        nop                             # 0x1c
        nop                             # 0x20          # Reserve space for hw exception vector
        nop                             # 0x24
        nop                             # 0x28          # Reserve space for Xilinx future use
        nop                             # 0x2c
        nop                             # 0x30          # Reserve space for Xilinx future use
        nop                             # 0x34
        nop                             # 0x38          # Reserve space for Xilinx future use
        nop                             # 0x3c
        nop                             # 0x40          # Reserve space for Xilinx future use
        nop                             # 0x44
        nop                             # 0x48          # Reserve space for Xilinx future use
        nop                             # 0x4c

        _start1:
/*	Set r13 and r2 small data area and stack pointer  */
	la	r13, r0, _SDA_BASE_		# r13 = read/write small data area anchor
	la	r2, r0, _SDA2_BASE_		# r2  = read only small data area
	la	r1, r0, __initial_stack-16   /*origine : _stack	modif le 03/03/2011 TODO*/# 16 bytes (4 words are needed by
					# crt for args and link reg )

/*      Set the opcodes brai and imm for handlers         */
	la	r6,r0,0xb8080000	# [opcode for brai ]
	swi	r6,r0,0x00000004			# [brai opcode for reset]
	swi	r6,r0,0x0000000c			# [brai opcode for exception]
	swi	r6,r0,0x00000014			# [brai opcode for interrupt]
	swi	r6,r0,0x00000024			# [brai opcode for hw exceptions]

	la	r6,r0,0xb0000000	# [opcode for imm ]
	swi	r6,r0,0x00000000			# [imm opcode for reset]
	swi	r6,r0,0x00000008			# [imm opcode for exception]
	swi	r6,r0,0x00000010			# [imm opocde for interrupt]
	swi	r6,r0,0x00000020			# [imm opocde for hw exceptions]

/* 	Set Reset vector        */
	la	r6,r0,_start1
	sw	r6,r1,r0
	nop
	lhu	r7,r1,r0
	nop
	shi	r7,r0, 0x00000002  		# [imm for reset]
	nop
	shi	r6,r0, 0x00000006			# [lower half for reset]
	nop
/* 	Set Software Exception Handler */
	la	r6,r0,_exception_handler
	sw	r6,r1,r0
	lhu	r7,r1,r0
	shi	r7,r0, 0x0000000a  		# [imm for exception]
	shi	r6,r0, 0x0000000e			# [lower half for exception ]

/* 	Set Interrupt Handler */
	la	r6,r0,_interrupt_handler
	sw	r6,r1,r0
	lhu	r7,r1,r0
	shi	r7,r0, 0x00000012 		# [imm for interrupt]
	shi	r6,r0, 0x00000016			# [lower half for intterupt ]

/*      Set HW Exception Handler */
    la      r6,r0,_hw_exception_handler
    sw      r6,r1,r0
    lhu     r7,r1,r0
    shi     r7,r0, 0x00000022      # [imm for exception]
    shi     r6,r0, 0x00000026      # [lower half for hw exception]
                
/* 	Adjust the stack pointer 			  */
	addi	r1,r1,16

	addi	r5,r1,0
	bri 	arch_init /*origine : arch_init modif 03/03/2011 boot_from_reset_vectorTODO*/	# branchement sur arch_init
	nop

/*      Fall through to exit                              */
        .end _start__
                
/* 	Use this exit function 	                          */
        .globl exit_                  # exit library call
        .ent exit_

exit_:
	bri	exit_
	.end exit_




