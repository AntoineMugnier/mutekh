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

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

asm(
    ".section        .boot,\"ax\",@progbits		\n"

    ".globl cpu_boot					\n"
    "cpu_boot:						\n"

    ".set push						\n"
    ".set noreorder					\n"

    /* set up IT disable and kernel mode */
#if __mips >= 32
    "mfc0      $8,	$12         			\n"
//    "ori       $8,   0x00000000  			\n"
    "andi      $8,   0x0000ffff  			\n"
    "mtc0      $8,	$12         			\n"
#else
    "li        $8,   0x00000000  			\n"
    "mtc0      $8,	$12         			\n"
#endif

    /* get CPU id and adjust stack */

#if __mips >= 32
    "mfc0	$9,	$15, 1			\n"
#else
    "mfc0	$9,	$15				\n"
#endif
    "la         $sp,	__system_uncached_heap_end - 16	\n"
    "andi	$9,	$9,	0x000003ff		\n"

#ifndef CONFIG_SMP
    "1: bne	$0,	$9,	1b			\n"
    "nop						\n"
#else
    "sll	$8,	$9,	10			\n"
    "subu	$sp,	$sp,	$8			\n"
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
    ".set push			\n"
    ".set noat			\n"
    "addiu	$8,	$0,	1024		\n"
    "li		$1,	" ASM_STR(SOCLIB_MC_MAGIC_VAL) " \n"
    "sw		$1,	" ASM_STR(SOCLIB_MC_MAGIC) "($0) \n"

    "sw		$8,	" ASM_STR(SOCLIB_MC_R2) "($0) \n"
    "subu	$8,	$sp,	$8			\n"
    "addiu	$8,	$8,		16			\n"
    "sw		$8,	" ASM_STR(SOCLIB_MC_R1) "($0) \n"
    "sw		$9,	" ASM_STR(SOCLIB_MC_CTX_CREATE) "($0) \n"
    "sw		$9,	" ASM_STR(SOCLIB_MC_CTX_SET) "($0) \n"
    "ori	$1,	$0,	" ASM_STR(SOCLIB_MC_CHECK_SPFP+SOCLIB_MC_CHECK_INIT) " \n"
    "sw		$1,	" ASM_STR(SOCLIB_MC_ENABLE) "($0) \n"

    "sw		$0,	" ASM_STR(SOCLIB_MC_MAGIC) "($0) \n"
    ".set pop						\n"
#endif

    /* setup global data pointer */
    "la	   $gp,   _gp					\n"

    /* jumpto arch_init function */

    "la         $8,   arch_init				\n"
    "j          $8					\n"
    "nop						\n"

    ".set pop						\n"
    );

