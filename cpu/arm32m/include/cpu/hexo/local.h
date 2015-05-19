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

    Copyright (c) Nicolas Poullon <nipo@ssji.net>, 2009
	              Alexandre Becoulet <alexandre.becoulet@free.fr>, 2009

*/


/**
   @file Task local and CPU local variables access
  */

#if !defined(LOCAL_H_) || defined(CPU_LOCAL_H_)
#error This file can not be included directly
#else

#define CPU_LOCAL_H_

# define CPU_LOCAL

/** context local storage type attribute */
# define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

# ifndef __MUTEK_ASM__
extern CONTEXT_LOCAL struct cpu_context_s arm_context_regs;
# endif

/** get address of cpu local object */
# define CONTEXT_GET_TLS()                                      \
  ({                                                            \
    uintptr_t _ptr_;                                            \
                                                                \
    asm (                                                       \
         "mrs %0, psp\n\t"                                      \
         : [ptr] "=r" (_ptr_)                                   \
        );                                                      \
                                                                \
    _ptr_;                                                      \
  })

#endif

