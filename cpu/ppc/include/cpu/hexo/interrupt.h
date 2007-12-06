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
   @file

   CPU specific interrupt handling
*/

#if !defined(INTERRUPT_H_) || defined(CPU_INTERRUPT_H_)
#error This file can not be included directly
#else

#define CPU_INTERRUPT_H_

#include "hexo/local.h"

extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;
extern CPU_LOCAL cpu_exception_handler_t  *cpu_exception_handler;

void mips_interrupt_entry(void);

static inline void
cpu_interrupt_sethandler(cpu_interrupt_handler_t *hndl)
{
  CPU_LOCAL_SET(cpu_interrupt_handler, hndl);
}

static inline void
cpu_exception_sethandler(cpu_exception_handler_t *hndl)
{
  CPU_LOCAL_SET(cpu_exception_handler, hndl);
}

static inline void
cpu_interrupt_disable(void)
{
  reg_t tmp;

  asm volatile (
		"mfmsr %0		\n\t"
		"and %0, %0, %1		\n\t"
		"mtmsr %0		\n\t"
		: "=r" (tmp)
		: "r" (~0x8000)
		);
}

static inline void
cpu_interrupt_enable(void)
{
  reg_t tmp;

  asm volatile (
		"mfmsr %0		\n\t"
		"ori %0, %0, 0x8000	\n\t"
		"mtmsr %0		\n\t"
		: "=r" (tmp)
		);
}

static inline void
cpu_interrupt_process(void)
{
  cpu_interrupt_enable();
  __asm__ volatile ("nop"
		    :
		    :
		    : "memory"
		    );
  cpu_interrupt_disable();
}

static inline void
cpu_interrupt_savestate(reg_t *state)
{
  __asm__ volatile (
		    "mfmsr	%0\n"
		    : "=r" (*state)
		    );
}

static inline void
cpu_interrupt_savestate_disable(reg_t *state)
{
  reg_t tmp;

  __asm__ volatile (
		    "mfmsr	%0		\n\t"
		    "mr		%1, %0		\n\t"
		    "and	%1, %1, %2	\n\t"
		    "mtmsr	%1		\n\t"
		    : "=&r" (*state)
		    , "=&r" (tmp)
		    : "r" (~0x8000)
		    );
}

static inline void
cpu_interrupt_restorestate(const reg_t *state)
{
  __asm__ volatile (
		    "mtmsr	%0"
		    :
		    : "r" (*state)
		    );
}

static inline bool_t
cpu_interrupt_getstate(void)
{
  reg_t		state;

  __asm__ volatile (
		    "mfmsr	%0"
		    : "=r" (state)
		    );

  return state & 0x8000 ? 1 : 0;
}

static inline void
cpu_interrupt_wait(void)
{
  //  __asm__ volatile ("");
}

#endif

