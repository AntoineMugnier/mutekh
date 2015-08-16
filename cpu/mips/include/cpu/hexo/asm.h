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

#ifndef __MIPS_ASM_H_
#define __MIPS_ASM_H_

asm(

// rd and rt must be different for store
".macro GLOBAL_ACCESS op, name, rd, rt \n"
"	lui  \\rt, %hi(\\name) \n"
"        addiu \\rt, %lo(\\name) \n"
"	\\op  \\rd, 0(\\rt) \n"
".endm \n"

#ifdef CONFIG_ARCH_SMP
".macro CPU_LOCAL_op op, name, rd, rt \n"
"        \\op \\rd, %lo(\\name) ($27) \n"
".endm \n"
#else
".macro CPU_LOCAL_op op, name, rd, rt \n"
"	GLOBAL_ACCESS \\op, \\name, \\rd, \\rt \n"
".endm \n"
#endif

".macro CONTEXT_LOCAL_op op, name, rd, rt \n"
"        CPU_LOCAL lw, __context_data_base, \\rt, \\rt \n"
"        \\op \\name, \\rd (\\rt) \n"
".endm \n"

".macro MTC0_ reg, creg \n"
"        mtc0   \\reg,    $\\creg \n"
#if CONFIG_CPU_MIPS_VERSION > 32
"	ehb \n"
#else
"	nop \n"
"	nop \n"
#endif
".endm \n"

".macro CPU_ID reg \n"
#ifdef CONFIG_ARCH_SMP_CAPABLE
# if CONFIG_CPU_MIPS_VERSION >= 32
"        mfc0    \\reg,    $15,    1 \n"
"        andi    \\reg,    \\reg,    0x3ff \n"
# else
"        mfc0    \\reg,    $15 \n"
"        andi    \\reg,    \\reg,    0x3ff \n"
# endif
#else
"        or     \\reg,  $0,      $0 \n"
#endif
".endm \n"

// restore multiple fpu regs
".macro LxC1 i j \n"
# if CONFIG_CPU_MIPS_FPU == 32
"    .ifeq \\i & 1 \n"
"	ldc1	$\\i,		\\j \n"
"    .endif \n"
# elif CONFIG_CPU_MIPS_FPU == 64
"	ldc1	$\\i,		\\j \n"
#endif
".endm \n"

".macro SxC1 i j \n"
# if CONFIG_CPU_MIPS_FPU == 32
"    .ifeq \\i & 1 \n"
"	sdc1	$\\i,		\\j \n"
"    .endif \n"
# elif CONFIG_CPU_MIPS_FPU == 64
"	sdc1	$\\i,		\\j \n"
#endif
".endm \n"

/** mtf0 instruction wait cycles (FIXME should depend on MIPS version) */
".macro MTC0_WAIT	\n"
"nop			\n"
"nop			\n"
".endm			\n"
     );

#endif

