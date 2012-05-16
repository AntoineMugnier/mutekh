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

_ASM( .macro CPU_ID _0                                                        )
# ifdef CONFIG_ARCH_SMP
#  if defined (CONFIG_CPU_SPARC_SOCLIB) || defined (CONFIG_CPU_SPARC_LEON3)
_ASM(        rd    %asr17, p0                                                 )
_ASM(        srl   p0, 28, p0                                                 )
#  else
#   error missing CPUID macro for your architecture
#  endif
# else
_ASM(        mov     %g0,                    p0                               )
# endif
_ASM( .endm                                                                   )



_ASM( .macro CPU_LOCAL_ld _0, _1                                              )
# ifdef CONFIG_ARCH_SMP
_ASM(          ld     [%g6 + %lo(p0)], p1                                     )
# else
_ASM(          set    p0,   p1                                                )
_ASM(          ld     [p1], p1                                                )
# endif
_ASM( .endm                                                                   )



_ASM( .macro CPU_LOCAL_st _0, _1, _2                                          )
# ifdef CONFIG_ARCH_SMP
_ASM(          st     p1, [%g6 + %lo(p0)]                                     )
# else
_ASM(          set    p0,   p2                                                )
_ASM(          st     p1,   [p2]                                              )
# endif
_ASM( .endm                                                                   )



_ASM( .macro WAIT                                                             )
# if defined (CONFIG_CPU_SPARC_LEON3) || defined (CONFIG_CPU_SPARC_SOCLIB)
_ASM(         wr %g0, %asr19                                                  )
# elif defined(CONFIG_CPU_WAIT_IRQ)
#  error No wait opcode defined for selected sparc processor
# endif
_ASM( .endm                                                                   )

#endif

