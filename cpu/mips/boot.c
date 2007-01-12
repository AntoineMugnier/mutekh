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

    ".set push						\n"
    ".set noreorder					\n"
    "							\n"
    /* set up IT disable and kernel mode */
    "li        $8,   0x0000FF00  			\n"
    "mtc0      $8,	$12         			\n"
    "							\n"
    /* get CPU id and adjust stack */
    "							\n"
    "mfc0	$8,	$15				\n"
    "la         $sp,	__system_uncached_heap_end - 16	\n"
    "andi	$8,	$8,	0x000003ff		\n"

    "1: bne	$0,	$8,	1b			\n"
    "nop						\n"

    //    "sll	$8,	$8,	10			\n"
    //    "subu	$sp,	$8,	$sp			\n"
    "							\n"
    /* setup global data pointer */
    "la	   $gp,   _gp					\n"
    "							\n"
    /* jumpto arch_init function */
    "							\n"
    "la         $8,   arch_init				\n"
    "j          $8					\n"
    "nop						\n"
    "							\n"
    ".set pop						\n"
    );

