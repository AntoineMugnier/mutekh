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

    Copyright (c) 2010, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef __ARM_ASM_H_
#define __ARM_ASM_H_

# define ASM_SECTION(name) \
        .section name,"ax"

# define CPU_ASM_FUNC_END .ltorg

asm(
".macro CPU_ID reg \n"
# if defined(CONFIG_CPU_ARM32_SOCLIB)
"        mrc    p15, 0, \\reg, c0, c0, 5 \n"
# elif defined(CONFIG_ARCH_SMP_CAPABLE)
"        .emsg No CPUID \n"
# else
"        mov    \\reg, #0 \n"
# endif
".endm \n"

/* get a variable using a cp15 register as base */
".macro GET_CP15_REL name, rd, rt, op2 \n"
"     mrc   p15,0, \\rt, c13, c0, \\op2 \n"
"     ldr   \\rd, =\\name \n"
"     ldr   \\rd, [\\rd, \\rt] \n"
".endm \n"

".macro SET_CP15_REL name, tmp, tmp2, val, op2 \n"
"     mrc   p15,0, \\tmp, c13, c0, \\op2 \n"
"     ldr   \\tmp2, =\\name \n"
"     str   \\val, [\\tmp, \\tmp2] \n"
".endm \n"

/* get a global variable */
".macro GET_GLOBAL name, rd \n"
"     ldr   \\rd, =\\name \n"
"     ldr   \\rd, [\\rd] \n"
".endm \n"

".macro SET_GLOBAL name, rval, tmp \n"
"     ldr   \\tmp, =\\name \n"
"     str   \\rval, [\\tmp] \n"
".endm \n"

/* get a variable using another global as base */
".macro GET_GLOBAL_REL name, rd, rt, var \n"
"     ldr   \\rt, =\\var \n"
"     ldr   \\rt, [\\rt] \n"
"     ldr   \\rd, =\\name \n"
"     ldr   \\rd, [\\rt, \\rd] \n"
".endm \n"

".macro GET_GLOBAL_REL_ADDR name, rd, rt, var \n"
"     ldr   \\rt, =\\var \n"
"     ldr   \\rt, [\\rt] \n"
"     ldr   \\rd, =\\name \n"
"     add   \\rd, \\rt, \\rd \n"
".endm \n"

/* TLS stuff */

#if CONFIG_CPU_ARM32_ARCH_VERSION >= 6

".macro CONTEXT_LOCAL_ld name, rd, rt \n"
"     GET_CP15_REL \\name, \\rd, \\rt, 4 \n"
".endm \n"

".macro CONTEXT_LOCAL_addr name, rd, rt \n"
"     mrc   p15,0, \\rt, c13, c0, 4 \n"
"     ldr   \\rd, =\\name \n"
"     add   \\rd, \\rd, \\rt \n"
".endm \n"

".macro TLS_BASE_SET reg, tmp \n"
"    mcr   p15, 0, \\reg, c13, c0, 4 \n"
".endm \n"

#else

".macro CONTEXT_LOCAL_addr name, rd, rt \n"
"     GET_GLOBAL_REL_ADDR \\name, \\rd, \\rt, __context_data_base \n"
".endm \n"

".macro CONTEXT_LOCAL_ld name, rd, rt \n"
"     GET_GLOBAL_REL \\name, \\rd, \\rt, __context_data_base \n"
".endm \n"

".macro TLS_BASE_SET reg, tmp \n"
"     SET_GLOBAL __context_data_base, \\reg, \\tmp \n"
".endm \n"

#endif

/* CLS stuff */

# ifdef CONFIG_ARCH_SMP
".macro CPU_LOCAL_ld name, rd, rt \n"
"     GET_CP15_REL \\name, \\rd, \\rt, 3 \n"
".endm \n"

".macro CPU_LOCAL_st name, tmp, tmp2, val \n"
"     SET_CP15_REL \\name, \\tmp, \\tmp2, \\val, 3 \n"
".endm \n"

# else

".macro CPU_LOCAL_ld name, rd, rt \n"
"     GET_GLOBAL \\name, \\rd \n"
".endm \n"

".macro CPU_LOCAL_st name, tmp, tmp2, val \n"
"     SET_GLOBAL \\name, \\val, \\tmp \n"
".endm \n"
# endif
);

#if defined(CONFIG_CPU_ARM32_THUMB)
# define THUMB_TMP_VAR uint32_t thumb_tmp
# define THUMB_TO_ARM                 \
    ".align 2               \n\t"     \
    "mov  %[adr], pc        \n\t"     \
    "add  %[adr], %[adr], #4\n\t"     \
    "bx   %[adr]            \n\t"     \
    "nop                    \n\t"     \
    ".arm                   \n\t"
# define ARM_TO_THUMB               \
    "add  %[adr], pc, #1  \n\t"     \
    "bx   %[adr]          \n\t"
# define THUMB_OUT(x...) x [adr] "=&l" (thumb_tmp)
#else
# define THUMB_TMP_VAR
# define THUMB_TO_ARM
# define ARM_TO_THUMB
# define THUMB_OUT(x, ...) x
#endif

#endif

