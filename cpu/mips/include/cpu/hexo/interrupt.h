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

#include "cpu/hexo/specific.h"

extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;
extern CPU_LOCAL cpu_exception_handler_t  *cpu_exception_handler;

#ifdef CONFIG_DRIVER_ICU_MIPS
struct device_s;
extern CPU_LOCAL struct device_s cpu_icu_dev;
#endif

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
  __asm__ volatile (
		    ".set push			\n"
		    ".set noat			\n"
		    ".set reorder		\n"
#if (CONFIG_CPU_MIPS_VERSION >= 322)
		    "di				\n"
		    "ehb			\n"
#else
		    "mfc0	$1,	$12	\n"
		    "ori	$1,	0x1	\n"
		    "addiu	$1,	-1	\n"
		    ".set noreorder		\n"
		    "mtc0	$1,	$12	\n"
#endif
		    "MTC0_WAIT			\n"
		    ".set pop			\n"
		    );
}

static inline void
cpu_interrupt_enable(void)
{
  __asm__ volatile (
		    ".set push			\n"
		    ".set noat			\n"
		    ".set reorder		\n"
#if (CONFIG_CPU_MIPS_VERSION >= 322)
		    "ei				\n"
		    "ehb			\n"
#else
		    "mfc0	$1,	$12	\n"
		    "ori	$1,	1	\n"
		    "mtc0	$1,	$12	\n"
#endif
		    ".set pop			\n"
		    );
}

static inline void
cpu_interrupt_savestate(reg_t *state)
{
  __asm__ volatile (
		    "mfc0	%0,	$12	\n"
		    : "=r" (*state)
		    );
}

static inline void
cpu_interrupt_savestate_disable(reg_t *state)
{
  __asm__ volatile (
#if (CONFIG_CPU_MIPS_VERSION >= 322)
		    "di	%0			\n"
		    "ehb			\n"
#else
		    ".set push				\n"
		    ".set noat				\n"
		    ".set reorder			\n"
		    "mfc0	%0,	$12		\n"
		    "ori	$1,	%0,	1	\n"
		    "addiu	$1,	-1		\n"
		    ".set noreorder			\n"
		    "mtc0	$1,	$12		\n"
		    "MTC0_WAIT				\n"
		    ".set pop				\n"
#endif
		    : "=r" (*state)
		    );
}

static inline void
cpu_interrupt_restorestate(const reg_t *state)
{
  __asm__ volatile (
		    "mtc0	%0,	$12		\n"
#if (CONFIG_CPU_MIPS_VERSION >= 322)
		    "ehb			\n"
#endif
		    :
		    : "r" (*state)
		    );
}

static inline void
cpu_interrupt_process(void)
{
  reg_t state;
  cpu_interrupt_savestate(&state);
  cpu_interrupt_enable();
  __asm__ volatile (
		    "nop"
		    :
		    :
    /* memory clobber is important here as cpu_interrupt_process()
       will let pending intterupts change global variables checked in
       a function loop (scheduler root queue for instance) */
		    : "memory"
		    );
  cpu_interrupt_restorestate(&state);
}

static inline bool_t
cpu_interrupt_getstate(void)
{
  reg_t		state;

  __asm__ volatile (
		    "mfc0	%0,	$12		\n"
		    : "=r" (state)
		    );

  return state & 0x01;
}

static inline bool_t
cpu_is_interruptible(void)
{
  reg_t		state;

  __asm__ volatile (
		    "mfc0	%0,	$12		\n"
		    : "=r" (state)
		    );

  // erl and exl masks interrupts
  return ( ! (state & 0x6)
		   && (state & 0x1) );
}

static inline void
cpu_interrupt_wait(void)
{
#if (CONFIG_CPU_MIPS_VERSION >= 322)
  __asm__ volatile ("wait");
#endif
}

#endif

