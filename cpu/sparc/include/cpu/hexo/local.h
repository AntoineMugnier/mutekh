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


/**
   @file Task local and CPU local variables access
  */

#if !defined(LOCAL_H_) || defined(CPU_LOCAL_H_)
#error This file can not be included directly
#else

#define CPU_LOCAL_H_

/************************************************************************/

#ifdef CONFIG_ARCH_SMP

# undef CPU_LOCAL
# define CPU_LOCAL	__attribute__((section (".cpudata")))

#define CPU_GET_CLS()					\
({							\
  uintptr_t _ptr_;					\
							\
  asm (							\
	   "	mov	%%g6, %0		\n"	\
	   : "=&r" (_ptr_)				\
  /* prevent optimize if memory has been reloaded (possible context switch), g6 may have changed too */ \
           : "m" (*(reg_t*)4)                           \
	   );						\
							\
  _ptr_;						\
})

#else /* CONFIG_ARCH_SMP */

# define CPU_LOCAL

#endif /* !CONFIG_ARCH_SMP */

/************************************************************************/

/** context local storage type attribute */
#define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

/** get address of cpu local object */
# define CONTEXT_GET_TLS()				\
({							\
  uintptr_t _ptr_;					\
							\
  asm (							\
	   "	mov	%%g7, %0		\n"	\
	   : "=&r" (_ptr_)				\
	   );						\
							\
  _ptr_;						\
})

#endif

