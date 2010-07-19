/*
 *   This file is part of MutekH.
 *
 *   MutekH is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   MutekH is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with MutekH; if not, write to the Free Software Foundation,
 *   Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 *   Copyright Francois Charot <charot@irisa.fr>  (c) 2008
 *   INRIA Rennes Bretagne Atlantique
 *
 */

#include <hexo/asm.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

asm(
    ".section .boot,\"ax\",@progbits                         \n"
    ".globl cpu_boot                                         \n"
    ".func cpu_boot                                        \n\t"
    ".type   cpu_boot, %function                           \n\t"
    ".set noat                                               \n"

    "cpu_boot:                                               \n"

    /* set up IT disable    */
    "  	movia   r16, 0x00000000                              \n"
    "  	wrctl   status, r16                                  \n"

    /* get CPU id and adjust stack */
    "	rdctl   r16, cpuid                                   \n"
    "	andi    r16, r16, 0x000003ff                         \n"
    "	movia   sp, __initial_stack                          \n"

    "1:                                                      \n"
#ifndef CONFIG_ARCH_SMP
    "   bne     r16, r16, 1b                                 \n"
#else
    "   movia   r17,  " ASM_STR(CONFIG_CPU_MAXCOUNT) "       \n"
    "   cmplt   r17, r16, r17                                \n"
    "   beq     zero, r17, 1b                                \n"

    "   slli    r17, r16, 10                                 \n"
    "   sub     sp,  sp,  r17                                \n"
#endif

#if defined(CONFIG_COMPILE_FRAMEPTR) && !defined(__OPTIMIZE__)
    "   or      fp, sp, zero                                 \n"
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
    ".set noat                                               \n"
    "   addi    r17, zero, 1024                              \n"

    /* enter memchecker command mode */
    "   movia   r1,  " ASM_STR(SOCLIB_MC_MAGIC_VAL) "        \n"
    "   stw     r1,  " ASM_STR(SOCLIB_MC_MAGIC) "(zero)      \n"

    /* create a new initial memchecker context using cpuid as context id */
    "   stw     r17,  " ASM_STR(SOCLIB_MC_R2) "(zero)        \n"
    "   sub     r17,  sp, r17                                \n"
    "   stw     r17,  " ASM_STR(SOCLIB_MC_R1) "(zero)        \n"
    "   stw     r16,  " ASM_STR(SOCLIB_MC_CTX_CREATE) "(zero)\n"

    /* switch to new memchecker context */
    "   stw     r16,  " ASM_STR(SOCLIB_MC_CTX_SET) "(zero)   \n"

    /* enable all memchecker checks */
    "   ori     r1,   zero, " ASM_STR(SOCLIB_MC_CHECK_ALL) " \n"
    "   stw     r1,   " ASM_STR(SOCLIB_MC_ENABLE) "(zero)    \n"
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
    "   stw     r0,   " ASM_STR(SOCLIB_MC_MAGIC) "(zero)     \n"
#endif

    "   movia   r4, __exception_base_ptr                     \n"
    "   wrctl   ctl17, r4                                    \n"


    /* Get the device tree and put it in first arg */
#ifdef CONFIG_ARCH_DEVICE_TREE
    "   movia   r4, dt_blob_start                            \n"
#else
    "   mov     r4, r0                                       \n"
#endif
    /* Put a 0 in second arg */
    "   mov     r5, zero                                     \n"

    /* jumpto arch_init function */
    "	movia   r16, arch_init                               \n"
    "	jmp     r16                                          \n"
    ".size   cpu_boot, .-cpu_boot                          \n\t"
    ".endfunc                                              \n\t"
    );

