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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#ifndef CPU_ASM_H_
#define CPU_ASM_H_

#define PPC_SPRG(n) (0x110 + n)

asm(
".macro LI32 reg value \n"
"	lis	\\reg,		\\value@h \n"
"	ori	\\reg,	\\reg,	\\value@l \n"
".endm \n"

".macro LA32 reg symbol \n"
"	lis	\\reg,		\\symbol@ha \n"
"	la	\\reg,		\\symbol@l(\\reg) \n"
".endm \n"

/* access a variable using a SPR as base */
".macro SPRREL_ACCESS op, name, rd, rt, spr \n"
"        mfspr  \\rt,      \\spr \n"
"        \\op    \\rd,      \\name (\\rt) \n"
"    .if \\rt == 0 \n"
"        .fail 0 \n"	// rt can not be zero here
"    .endif \n"
".endm \n"

/* access a global variable */
".macro GLOBAL_ACCESS op, name, rd, rt \n"
"        lis   \\rt, \\name@h \n"
"        ori   \\rt, \\rt, \\name@l \n"
"        \\op   \\rd, 0(\\rt) \n"
"    .if \\rt == 0 \n"
"        .fail 0 \n"	// rt can not be zero here
"    .endif \n"
".endm \n"

#ifdef CONFIG_ARCH_SMP
".macro CPU_LOCAL_op op, name, rd, rt \n"
"        SPRREL_ACCESS \\op, \\name, \\rd, \\rt, 0x115 \n"
".endm \n"
#else
".macro CPU_LOCAL_op op, name, rd, rt \n"
"        GLOBAL_ACCESS \\op, \\name, \\rd, \\rt \n"
".endm \n"
#endif

".macro CONTEXT_LOCAL_op op, name, rd, rt \n"
"        SPRREL_ACCESS \\op, \\name, \\rd, \\rt, 0x114 \n"
".endm \n"

// restore multiple gp regs
".macro LMW_GP array i j \n"
"	lwz	\\i,		(_offsetof(cpu_context_s, gpr) + 4 * \\i)(\\array) \n"
"    .if \\i-\\j \n"
"	LMW_GP \\array, (\\i+1), \\j \n"
"    .endif \n"
".endm \n"

// save multiple gp regs
".macro SMW_GP array i j \n"
"	stw	\\i,		(_offsetof(cpu_context_s, gpr) + 4 * \\i)(\\array) \n"
"    .if \\i-\\j \n"
"	SMW_GP \\array, (\\i+1), \\j \n"
"    .endif \n"
".endm \n"

#ifdef CONFIG_HEXO_FPU
// restore multiple fpu regs
".macro LMD_FPU array i j \n"
"	lfd	\\i,		(_offsetof(cpu_context_s, fpr) + 8 * \\i)(\\array) \n"
"    .if \\i-\\j \n"
"	LMD_FPU \\array, (\\i+1), \\j \n"
"    .endif \n"
".endm \n"

// save multiple fpu regs
".macro SMD_FPU array i j \n"
"	stfd	\\i,		(_offsetof(cpu_context_s, fpr) + 8 * \\i)(\\array) \n"
"    .if \\i-\\j \n"
"	SMD_FPU \\array, (\\i+1), \\j \n"
"    .endif \n"
".endm \n"
#endif

);

#endif

