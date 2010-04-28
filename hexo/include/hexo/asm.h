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

#include <cpu/hexo/asm.h>

#ifndef __MUTEK_ASM__

#ifndef ASM_SECTION
# error Your CPU must define ASM_SECTION
#endif

# define ASM_STR_(x) #x
# define ASM_STR(x) ASM_STR_(x)

# define FUNC_START(section, x)                 \
        ASM_SECTION(#section "." #x)            \
        "\t.globl " #x "            \n"         \
        "\t.func " #x "             \n"         \
        "\t.type " #x ", %function  \n"         \
        #x ":                     \n"

# define FUNC_START_ORG(section, x, o)                         \
        "\t.org " #o "              \n"         \
        "\t.globl " #x "            \n"         \
        "\t.func " #x "             \n"         \
        "\t.type " #x ", %function  \n"         \
        #x ":                     \n"

# define FUNC_END(x)                            \
    "\t.endfunc               \n"               \
    "\t.size " #x ", .-" #x " \n"

#else

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
        .endfunc                              ; \
        .size x, .-x

#endif

#endif
