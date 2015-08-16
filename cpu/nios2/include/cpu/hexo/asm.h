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

*/

#ifndef __HEXO_NIOS2_ASM_H_
#define __HEXO_NIOS2_ASM_H_

asm(
".macro GLOBAL_ACCESS _0, _1, _2, _3                    \n"
"         movia  \\_3,     \\_1                         \n"
"         \\_0    \\_2,     (\\_3)                      \n"
".endm                                                  \n"

#ifdef CONFIG_ARCH_SMP
".macro CPU_LOCAL_op _0, _1, _2, _3                     \n"
"        \\_0 \\_2, \\_1(r26)                           \n"
".endm                                                  \n"
#else
".macro CPU_LOCAL_op _0, _1, _2, _3                     \n"
"	GLOBAL_ACCESS \\_0, \\_1, \\_2, \\_3            \n"
".endm                                                  \n"
#endif
);

#endif

