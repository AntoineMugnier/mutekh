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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
    Copyright Joel Porquet <joel.porquet@lip6.fr> (c) 2009

*/

#include <setjmp.h>

reg_t setjmp(jmp_buf env);
asm(
	    ".type setjmp, %function \n"
	    ".globl setjmp          \n\t"
            "setjmp:                \n\t"
            ".set push				\n"
            ".set noat				\n"

            /* save gp registers */
            "sw     $16,    16*4($4)\n"
            "sw     $17,    17*4($4)\n"
            "sw     $18,    18*4($4)\n"
            "sw     $19,    19*4($4)\n"
            "sw     $20,    20*4($4)\n"
            "sw     $21,    21*4($4)\n"
            "sw     $22,    22*4($4)\n"
            "sw     $23,    23*4($4)\n"

            "sw     $24,    24*4($4)\n"
            "sw     $25,    25*4($4)\n"

            "sw     $gp,    28*4($4)\n"
            "sw     $sp,    29*4($4)\n"
            "sw     $fp,    30*4($4)\n"
            "sw     $ra,    31*4($4)\n"

            /* return is 0 */
            "move   $2,      $0     \n"
            "jr     $ra             \n"

            ".set pop		    \n"
            ".size setjmp, .-setjmp \n\t"
    );

void longjmp(jmp_buf env, reg_t val);
asm(
	    ".type longjmp, %function \n"
	    ".globl longjmp          \n\t"
            "longjmp:                \n\t"
            ".set push				\n"
            ".set noat				\n"

            "bne    $0,     $5,      1f\n"
            "nop                       \n"
            "li     $5,     1          \n"
            "1:                        \n"

            /* restore gp registers */
            "lw     $16,    16*4($4)\n"
            "lw     $17,    17*4($4)\n"
            "lw     $18,    18*4($4)\n"
            "lw     $19,    19*4($4)\n"
            "lw     $20,    20*4($4)\n"
            "lw     $21,    21*4($4)\n"
            "lw     $22,    22*4($4)\n"
            "lw     $23,    23*4($4)\n"

            "lw     $24,    24*4($4)\n"
            "lw     $25,    25*4($4)\n"

            "lw     $gp,    28*4($4)\n"
            "lw     $sp,    29*4($4)\n"
            "lw     $fp,    30*4($4)\n"
            "lw     $ra,    31*4($4)\n"

            /* set return value: */
            "move   $2,     $5      \n"
            "jr     $ra             \n"
            ".set pop               \n"
            ".size longjmp, .-longjmp \n\t"
    );

