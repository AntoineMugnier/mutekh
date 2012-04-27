/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2010
 */

#ifndef ASM_H_
#define ASM_H_

/* The cpu/hexo/asm.h file contains GNU as .macro definition which
   must go in the assembly source file. Assembly is this file use the
   _ASM macro foreach line of code so that appropriate escaping is
   used. The p* macros are used for as .macro parameters in order to
   properly escape backslash. */

#ifdef __MUTEK_ASM__     /* We are included from a .S file */

# define p0 \_0
# define p1 \_1
# define p2 \_2
# define p3 \_3
# define p4 \_4
# define p5 \_5
# define _ASM(...) __VA_ARGS__
# define __ASM

#else                    /* We are included from a .c file */

# define p0 \\_0
# define p1 \\_1
# define p2 \\_2
# define p3 \\_3
# define p4 \\_4
# define p5 \\_5
# define _ASM(...) __ASM(__VA_ARGS__)
# define __ASM(...) #__VA_ARGS__ "\n"

asm(
#endif
#include <cpu/hexo/asm.h>
#ifndef __MUTEK_ASM__
);
#endif

#undef p0
#undef p1
#undef p2
#undef p3
#undef p4
#undef p5
#undef _ASM
#undef __ASM

/****************************************************/

# define ASM_STR_(x) #x
# define ASM_STR(x) ASM_STR_(x)

#ifdef __MUTEK_ASM__     /* We are included from a .S file */

# ifndef ASM_SECTION
#  define ASM_SECTION(name) \
    .section name,"ax",@progbits
# endif

# ifndef CPU_ASM_FUNC_END
#  define CPU_ASM_FUNC_END
# endif

# define FUNC_START(sec, x)              \
        ASM_SECTION(sec.x)                    ; \
        .globl x                              ; \
        .func x                               ; \
        .type x , %function                   ; \
        x:

# define FUNC_START_ORG(x, o)              \
        .org o                                ; \
        .func x                               ; \
        .type x , %function                   ; \
        x:

# define FUNC_END(x)                            \
        CPU_ASM_FUNC_END                      ; \
        .globl x##_end                        ; \
        x##_end:                                \
        .endfunc                              ; \
        .size x, .-x

#else                    /* We are included from a .c file */

# define ASM_SECTION(name)              \
        ".section " name ",\"ax\",@progbits \n\t"

#endif

#endif
