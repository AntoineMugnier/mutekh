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

    Copyright (c) Nicolas Pouillon, <nipo@ssji.net>, 2009

*/

#include <setjmp.h>

reg_t setjmp(jmp_buf env);
asm(
        ".thumb                  \n"
        ".thumb_func             \n"
	".type setjmp, %function \n"
	".globl setjmp          \n\t"
	"setjmp:                \n\t"
      	"stmia   r0, {r4, r5, r6, r7, r8, r9, r10, r11, r12}\n\t"
        "str     r13, [r2, #-40]\n\t"
        "str     lr,  [r2, #-44]\n\t"
	"movs    r0, #0         \n\t"
	"bx      lr             \n\t"
	".size setjmp, .-setjmp \n\t"
	);


void longjmp(jmp_buf env, reg_t val);
asm(
        ".thumb                  \n"
        ".thumb_func             \n"
	".type longjmp, %function \n"
	".globl longjmp          \n\t"
	"longjmp:                \n\t"
	"mov    r2, r0           \n\t"
	"cmp    r1, #0           \n\t"
	"ITE    NE               \n\t"
	"movne  r0, r1           \n\t"
	"moveq  r0, #1           \n\t"
	"ldmia  r2, {r4, r5, r6, r7, r8, r9, r10, r11, r12}\n\t"
        "ldr     r13, [r2, #-40]\n\t"
        "ldr     pc,  [r2, #-44]\n\t"
	".size longjmp, .-longjmp \n\t"
	);
