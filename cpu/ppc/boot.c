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
    ".section        .boot,\"ax\",@progbits		\n"

    ".globl cpu_boot					\n"
    "cpu_boot:						\n"

    /* get CPU id and adjust stack */
    "lis	9, __system_uncached_heap_end - 8@ha    \n"
    "la		1, __system_uncached_heap_end - 8@l(9)  \n"
    "mfdcr	29,0					\n"

#ifndef CONFIG_SMP
    /* only first CPU is allowed to boot */
    "cmpwi	cr0, 29, 0				\n"
    "1:							\n"
    "bne	cr0, 1b					\n"
#endif

    "rlwinm	3,29,12,0,19				\n"
    "sub	1,1,3					\n"

    //    "sll	$8,	$8,	10			\n"
    //    "subu	$sp,	$8,	$sp			\n"
    "							\n"
    /* setup global data pointer */
    "lis	13, _gp@ha				\n"
    "la		13, _gp@l(13)				\n"

    /* jumpto arch_init function */

    "lis	3, arch_init@ha				\n"
    "la         3, arch_init@l(3)			\n"
    "mtctr      3					\n"
    "bctr						\n"

    ".org 60						\n"
    "b		cpu_boot				\n"
    );

