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

_ASM(.macro GLOBAL_ACCESS _0, _1, _2, _3                    )
_ASM(         movia  p3,     p1                             )
_ASM(         p0    p2,     (p3)                            )
_ASM(.endm                                                  )

#ifdef CONFIG_ARCH_SMP
_ASM(.macro CPU_LOCAL _0, _1, _2, _3                        )
_ASM(        p0 p2, p1(CPU_NIOS2_CLS_REG)                   )
_ASM(.endm                                                  )
#else
_ASM(.macro CPU_LOCAL _0, _1, _2, _3                        )
_ASM(	GLOBAL_ACCESS p0, p1, p2, p3                        )
_ASM(.endm                                                  )
#endif

#endif

