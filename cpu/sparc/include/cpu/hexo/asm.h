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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech
*/

#ifndef CPU_ASM_H_
#define CPU_ASM_H_

asm(
    " .macro CPU_ID _0 \n"
# ifdef CONFIG_ARCH_SMP_CAPABLE
#  if defined (CONFIG_CPU_SPARC_SOCLIB) || defined (CONFIG_CPU_SPARC_LEON3)
    "        rd    %asr17, \\_0 \n"
    "        srl   \\_0, 28, \\_0 \n"
#  else
#   error missing CPUID macro for your architecture
#  endif
# else
    "        mov     %g0, \\_0 \n"
# endif
    " .endm\n"

    " .macro CPU_LOCAL_ld _0, _1 \n"
# ifdef CONFIG_ARCH_SMP
    "          ld     [%g6 + %lo(\\_0)], \\_1 \n"
# else
    "          set    \\_0,   \\_1 \n"
    "          ld     [\\_1], \\_1 \n"
# endif
    " .endm \n"



    " .macro CPU_LOCAL_st _0, _1, _2 \n"
# ifdef CONFIG_ARCH_SMP
    "          st     \\_1, [%g6 + %lo(\\_0)] \n"
# else
    "          set    \\_0,   \\_2 \n"
    "          st     \\_1,   [\\_2] \n"
# endif
    " .endm \n"



    " .macro WAIT \n"
# if defined (CONFIG_CPU_SPARC_LEON3) || defined (CONFIG_CPU_SPARC_SOCLIB)
    "         wr %g0, %asr19 \n"
    "         nop \n"
    "         nop \n"
# elif defined(CONFIG_CPU_WAIT_IRQ)
#  error No wait opcode defined for selected sparc processor
# endif
    " .endm \n"
);

#endif

