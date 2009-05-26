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
    ".section        .text,\"ax\",@progbits		\n"

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

	"li    3, 0                      \n"
	"mtmsr 3                         \n"

#ifdef CONFIG_SOCLIB_MEMCHECK
    "addi	2,	0,	1024		\n"
/*     "lis	0, hi(" ASM_STR(SOCLIB_MC_MAGIC_VAL) ") \n" */
/*     "ori	0, 0, lo(" ASM_STR(SOCLIB_MC_MAGIC_VAL) ") \n" */
	"lis	0, (" ASM_STR(SOCLIB_MC_MAGIC_VAL) ")@h  \n"
	"ori 0,	0, (" ASM_STR(SOCLIB_MC_MAGIC_VAL) ")@l  \n"
    "stw	0,	" ASM_STR(SOCLIB_MC_MAGIC) "(0) \n"

    "stw	2,	" ASM_STR(SOCLIB_MC_R2) "(0) \n"
    "subf	2,	2,	1			\n"
    "addi	2,	2,		8			\n"
    "stw	2,	" ASM_STR(SOCLIB_MC_R1) "(0) \n"
    "stw	29,	" ASM_STR(SOCLIB_MC_CTX_CREATE) "(0) \n"
    "stw	29,	" ASM_STR(SOCLIB_MC_CTX_SET) "(0) \n"
    "addi	0,	0,	" ASM_STR(SOCLIB_MC_CHECK_SPFP+SOCLIB_MC_CHECK_INIT) " \n"
    "stw	0,	" ASM_STR(SOCLIB_MC_ENABLE) "(0) \n"

    "stw	3,	" ASM_STR(SOCLIB_MC_MAGIC) "(0) \n"
#endif

    /* setup global data pointer */
    "lis	13, _gp@ha				\n"
    "la		13, _gp@l(13)				\n"

    /* jumpto arch_init function */

    "lis	3, arch_init@ha				\n"
    "la         3, arch_init@l(3)			\n"
    "mtctr      3					\n"
    "bctr						\n"
    );

asm(
    ".section        .boot,\"ax\",@progbits     \n"

    ".globl cpu_boot_pointer    \n\t"
    "cpu_boot_pointer:          \n\t"
    ".org 0x80-4*5              \n\t"
	"1:                         \n\t"
    "lis    3, cpu_boot@ha      \n\t"
    "la     3, cpu_boot@l(3)    \n\t"
    "mtctr  3                   \n\t"
    "bctr                       \n\t"
    "b      1b                  \n\t"
    );

