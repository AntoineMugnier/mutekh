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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


/**
   @file Task local and CPU local variables access
  */

#if !defined(LOCAL_H_) || defined(CPU_LOCAL_H_)
#error This file can not be included directly
#else

#define CPU_LOCAL_H_

extern void *cpu_context_reg;

/************************************************************************/

/** context local storage type attribute */
#define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

/** context local storage variable assignement from different context */
#define CONTEXT_LOCAL_FOREIGN_SET(tls, n, v)	({ *(typeof(n)*)((uintptr_t)(tls) + (uintptr_t)&(n)) = (v); })

/** context local storage variable assignement */
#define CONTEXT_LOCAL_SET(n, v)	({ *(typeof(n)*)((uintptr_t)(cpu_context_reg) + (uintptr_t)&(n)) = (v); })

/** context local storage variable read access */
#define CONTEXT_LOCAL_GET(n) 	({ *(typeof(n)*)((uintptr_t)(cpu_context_reg) + (uintptr_t)&(n)); })

/** get address of context local object */
#define CONTEXT_LOCAL_ADDR(n)	({ (void*)((uintptr_t)(cpu_context_reg) + (uintptr_t)&(n)); })

/************************************************************************/

/** cpu local storage type attribute */
#ifdef CONFIG_SMP
# define CPU_LOCAL	__thread
#else
# define CPU_LOCAL
#endif

/** cpu local storage variable assignement */
# define CPU_LOCAL_SET(n, v)  (n) = (v)

/** cpu local storage variable read access */
# define CPU_LOCAL_GET(n)    (n)

/** get address of cpu local object */
# define CPU_LOCAL_ADDR(n)   (&(n))

#endif

