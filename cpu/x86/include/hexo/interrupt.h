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

/** first interrupt vector used for exceptions */
#define CPU_EXCEPT_VECTOR		0
/** exceptions vector count */
#define CPU_EXCEPT_VECTOR_COUNT		32

/** first interrupt vector used for hardware interrupts */
#define CPU_HWINT_VECTOR		32
/** hardware interrupts vector count */
#define CPU_HWINT_VECTOR_COUNT		96

/** first interrupt vector used for system calls */
#define CPU_SYSCALL_VECTOR		128
/** syscall vector count */
#define CPU_SYSCALL_VECTOR_COUNT	128

/** max interrupt line handled by the CPU */
#define CPU_MAX_INTERRUPTS		256

/** interrupts entry tampline code size  */
#define CPU_INTERRUPT_ENTRY_ALIGN	16

extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
extern CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

void x86_interrupt_hw_entry(void);
void x86_interrupt_ex_entry(void);
void x86_interrupt_sys_entry(void);

static inline void
cpu_interrupt_hw_sethandler(cpu_interrupt_handler_t *hndl)
{
  CPU_LOCAL_SET(cpu_interrupt_hw_handler, hndl);
}

static inline void
cpu_interrupt_ex_sethandler(cpu_exception_handler_t *hndl)
{
  CPU_LOCAL_SET(cpu_interrupt_ex_handler, hndl);
}

static inline void
cpu_interrupt_sys_sethandler(cpu_interrupt_handler_t *hndl)
{
  CPU_LOCAL_SET(cpu_interrupt_sys_handler, hndl);
}

static inline void
cpu_interrupt_disable(void)
{
  __asm__ volatile (
		    "cli\n"
		    );
}

static inline void
cpu_interrupt_enable(void)
{
  __asm__ volatile (
		    "sti\n"
		    );
}

static inline void
cpu_interrupt_savestate(reg_t *state)
{
  __asm__ volatile (
		    "pushfl	\n"
		    "popl	%0\n"
		    : "=m,r" (*state)
		    );
}

static inline void
cpu_interrupt_savestate_disable(reg_t *state)
{
  cpu_interrupt_savestate(state);
  cpu_interrupt_disable();
}

static inline void
cpu_interrupt_restorestate(const reg_t *state)
{
  __asm__ volatile (
		    "pushl	%0\n"
		    "popfl	\n"
		    :
		    : "m,r" (*state)
		    );
}

static inline bool_t
cpu_interrupt_getstate(void)
{
  reg_t		flags;

  __asm__ volatile (
		    "pushfl	\n"
		    "popl	%0\n"
		    : "=r" (flags)
		    );

  return flags & 0x200 ? 1 : 0;
}

static inline void
cpu_interrupt_wait(void)
{
  __asm__ volatile ("hlt");
}

#endif

