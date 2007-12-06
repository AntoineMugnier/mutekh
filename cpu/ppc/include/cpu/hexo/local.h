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

/************************************************************************/

#ifdef CONFIG_SMP

# define CPU_LOCAL	__attribute__((section (".cpudata")))

/* SPRG5 register is cls */

/** cpu local storage variable assignement */
# define CPU_LOCAL_SET(n, v)				\
{							\
  reg_t tmp;						\
							\
  volatile asm (					\
	   "	mfspr	%0, 0x105		\n"	\
	   "	stwx	%2, %1, %0		\n"	\
	   : "=&r" (tmp)				\
	   : "b" (&n)					\
	   , "r" ((typeof(n))v)				\
	   : "memory"					\
	   );						\
}

/** cpu local storage variable read access */
# define CPU_LOCAL_GET(n)				\
({							\
  reg_t tmp;						\
  typeof(n) _val_;					\
							\
  asm (							\
	   "	mfspr	%0, 0x105		\n"	\
	   "	lwzx	%1, %2, %0		\n"	\
	   : "=&r" (tmp)				\
	   , "=&b" (_val_)				\
	   : "r" (&n)					\
	   );						\
							\
  _val_;						\
})

/** get address of cpu local object */
# define CPU_LOCAL_ADDR(n)				\
({							\
  typeof(n) *_ptr_;					\
							\
  asm (							\
	   "	mfspr	%0, 0x105		\n"	\
	   "	add	%0, %0, %1		\n"	\
	   : "=&r" (_ptr_)				\
	   : "r" (&n)					\
	   );						\
							\
  _ptr_;						\
})

#else

# define CPU_LOCAL

/** cpu local storage variable assignement */
# define CPU_LOCAL_SET(n, v)  (n) = (v)

/** cpu local storage variable read access */
# define CPU_LOCAL_GET(n)    (n)

/** get address of cpu local object */
# define CPU_LOCAL_ADDR(n)   (&(n))

#endif /* !CONFIG_SMP */

/************************************************************************/

/* SPRG4 register is tls */

/** context local storage type attribute */
#define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

/** context local storage variable assignement from different context */
#define CONTEXT_LOCAL_FOREIGN_SET(tls, n, v)	({ *(typeof(n)*)((uintptr_t)(tls) + (uintptr_t)&(n)) = (v); })

/** cpu local storage variable assignement */
# define CONTEXT_LOCAL_SET(n, v)			\
{							\
  reg_t tmp;						\
							\
  asm volatile (					\
	   "	mfspr	%0, 0x104		\n"	\
	   "	stwx	%2, %1, %0		\n"	\
	   : "=&r" (tmp)				\
	   : "r" (&n)					\
	   , "r" ((typeof(n))v)				\
	   : "memory"					\
	   );						\
}

/** cpu local storage variable read access */
# define CONTEXT_LOCAL_GET(n)				\
({							\
  reg_t tmp;						\
  typeof(n) _val_;					\
							\
  asm (							\
	   "	mfspr	%0, 0x104		\n"	\
	   "	lwzx	%1, %2, %0		\n"	\
	   : "=&r" (tmp)				\
	   , "=&b" (_val_)				\
	   : "r" (&n)					\
	   );						\
							\
  _val_;						\
})

/** get address of cpu local object */
# define CONTEXT_LOCAL_ADDR(n)				\
({							\
  typeof(n) *_ptr_;					\
							\
  asm (							\
	   "	mfspr	%0, 0x104		\n"	\
	   "	add	%0, %0, %1		\n"	\
	   : "=&r" (_ptr_)				\
	   : "r" (&n)					\
	   );						\
							\
  _ptr_;						\
})

#endif

