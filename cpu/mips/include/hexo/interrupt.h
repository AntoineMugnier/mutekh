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

extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
extern CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

void mips_interrupt_entry(void);

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
		    ".set push			\n"
		    ".set noat			\n"
		    ".set reorder		\n"
#if (CONFIG_MIPS_VERSION >= 332)
		    "di				\n"
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
#if (CONFIG_MIPS_VERSION >= 332)
		    "ei				\n"
#else
		    "mfc0	$1,	$12	\n"
		    "ori	$1,	1	\n"
		    "mtc0	$1,	$12	\n"
#endif
		    ".set pop			\n"
		    );
}

static inline void
cpu_interrupt_savestate(__reg_t *state)
{
  __asm__ volatile (
		    "mfc0	%0,	$12	\n"
		    : "=r" (*state)
		    );
}

static inline void
cpu_interrupt_savestate_disable(__reg_t *state)
{
  __asm__ volatile (
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
		    : "=r" (*state)
		    );
}

static inline void
cpu_interrupt_restorestate(const __reg_t *state)
{
  __asm__ volatile (
		    "mtc0	%0,	$12		\n"
		    :
		    : "r" (*state)
		    );
}

#endif

