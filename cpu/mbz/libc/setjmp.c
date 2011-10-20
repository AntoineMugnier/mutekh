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

#include <setjmp.h>

reg_t setjmp(jmp_buf env);
asm(
		".type setjmp, %function 			\n"
		".globl setjmp			 			\n\t"
		".section .text          			\n\t"
		".align 2                			\n\t"
		".ent setjmp             			\n\t"
		"setjmp:                 			\n\t"
		"swi     r1,  r5, 0       			\n\t"
		"swi     r13, r5, 4                	\n\t"
		"swi     r14, r5, 8                	\n\t"
		"swi     r15, r5, 12                \n\t"
		"swi     r16, r5, 16                \n\t"
		"swi     r17, r5, 20                \n\t"
		"swi     r18, r5, 24                \n\t"
		"swi     r19, r5, 28                \n\t"
		"swi     r20, r5, 32                \n\t"
		"swi     r21, r5, 36                \n\t"
		"swi     r22, r5, 40                \n\t"
		"swi     r23, r5, 44                \n\t"
		"swi     r24, r5, 48                \n\t"
		"swi     r25, r5, 52                \n\t"
		"swi     r26, r5, 56                \n\t"
		"swi     r27, r5, 60                \n\t"
		"swi     r28, r5, 64                \n\t"
		"swi     r29, r5, 68                \n\t"
		"swi     r30, r5, 72                \n\t"
		"swi     r31, r5, 76                \n\t"
		"rtsd    r15, 8                		\n\t"
		"or      r3, r0, r0                	\n\t"
		".end setjmp                		\n\t"
	);


void longjmp(jmp_buf env, reg_t val);
asm(
		".globl longjmp			 			\n"
		".section .text			 			\n\t"
		".align 2  			 				\n\t"
		".ent longjmp    			 		\n\t"
		"longjmp:			 				\n\t"
		    "lwi     r1, r5, 0			 	\n\t"
		    "lwi     r13, r5, 4			 	\n\t"
		    "lwi     r14, r5, 8			 	\n\t"
		    "lwi     r15, r5, 12			\n\t"
		    "lwi     r16, r5, 16			\n\t"
		    "lwi     r17, r5, 20			\n\t"
		    "lwi     r18, r5, 24			\n\t"
		    "lwi     r19, r5, 28			\n\t"
		    "lwi     r20, r5, 32			\n\t"
		    "lwi     r21, r5, 36			\n\t"
		    "lwi     r22, r5, 40			\n\t"
		    "lwi     r23, r5, 44			\n\t"
		    "lwi     r24, r5, 48			\n\t"
		    "lwi     r25, r5, 52			\n\t"
		    "lwi     r26, r5, 56			\n\t"
		    "lwi     r27, r5, 60			\n\t"
		    "lwi     r28, r5, 64			\n\t"
		    "lwi     r29, r5, 68			\n\t"
		    "lwi     r30, r5, 72			\n\t"
		    "lwi     r31, r5, 76			\n\t"
		    "rtsd    r15, 8					\n\t"
		    "or      r3, r0, r6				\n\t"
		".end longjmp						\n\t"
	);
