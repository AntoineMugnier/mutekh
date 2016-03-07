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

    Copyright (c) Nicolas Pouillon, <nipo@ssji.net>, 2009

*/

#include <setjmp.h>

reg_t setjmp(jmp_buf env);
asm(
        ".thumb                  \n\t"
        ".thumb_func             \n\t"
	".type setjmp, %function \n"
	".globl setjmp          \n\t"
	"setjmp:                \n\t"
	"stmia   r0!, {r4, r5, r6, r7} \n\t"
        "mov     r2, r9         \n\t"
        "mov     r3, r10        \n\t"
        "mov     r4, r11        \n\t"
        "mov     r5, r12        \n\t"
        "mov     r6, r13        \n\t"
        "mov     r7, lr         \n\t"
	"stmia   r0!, {r2, r3, r4, r5, r6, r7} \n\t"
	"movs    r0, #0         \n\t"
	"bx      lr             \n\t"
	".size setjmp, .-setjmp \n\t"
	);


void longjmp(jmp_buf env, reg_t val);
asm(
        ".thumb                  \n\t"
        ".thumb_func             \n\t"
	".type longjmp, %function \n"
	".globl longjmp          \n\t"
	"longjmp:                \n\t"
	"mov    r2, r0           \n\t"

	"mov    r0, r1           \n\t"
	"cmp    r0, #0           \n\t"
        "bne    1f               \n\t"
	"mov    r0, #1           \n\t"
	"1:                      \n\t"

	"mov    r1, r2           \n\t"
	"ldmia   r1!, {r2, r3, r4, r5, r6, r7} \n\t"
        "mov     r9, r2         \n\t"
        "mov     r10, r3        \n\t"
        "mov     r11, r4        \n\t"
        "mov     r12, r5        \n\t"
        "mov     r13, r6        \n\t"
        "mov     lr, r7         \n\t"
	"ldmia   r1!, {r4, r5, r6, r7} \n\t"
	"bx      lr             \n\t"
	".size longjmp, .-longjmp \n\t"
	);

