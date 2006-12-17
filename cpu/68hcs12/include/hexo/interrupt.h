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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#if !defined(INTERRUPT_H_) || defined(CPU_INTERRUPT_H_)
#error This file can not be included directly
#else

#define CPU_INTERRUPT_H_

#include "hexo/local.h"

extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
extern CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

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
  __asm__ volatile ("sei");
}

static inline void
cpu_interrupt_enable(void)
{
  __asm__ volatile ("cli");
}

static inline void
cpu_interrupt_process(void)
{
  __asm__ volatile ("nop"
		    :
		    :
		    : "memory");
}

static inline void
cpu_interrupt_savestate(reg_t *state)
{
  reg_t	r;

  __asm__ volatile ("tpa"
		    : "=a" (r));
  *state = r & 0x10;
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
  if (!state)
    cpu_interrupt_enable();
  else
    cpu_interrupt_disable();
}

static inline bool_t
cpu_interrupt_getstate(void)
{
  reg_t	r;

  __asm__ volatile ("tpa"
		    : "=a" (r));

  return !!(r & 0x10);
}

static inline void
cpu_interrupt_wait(void)
{
  __asm__ volatile ("wai"
		    :
		    :
		    : "memory");
}

#endif
