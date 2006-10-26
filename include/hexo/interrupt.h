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


/*

    %config CONFIG_HEXO_IPI
    desc Inter processor interrupts support
    depend CONFIG_SMP
    %config end

*/

#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "types.h"

/** CPU interrupt handler function template */
#define CPU_INTERRUPT_HANDLER(n) void (n) (uint_fast8_t irq)

/**
   CPU interrupt handler function type.

   @param irq interrupt line number
*/
typedef CPU_INTERRUPT_HANDLER(cpu_interrupt_handler_t);


/** CPU exception handler function template */
#define CPU_EXCEPTION_HANDLER(n) void (n) (uint_fast8_t type, uintptr_t execptr, \
					   uintptr_t dataptr, reg_t *regtable)
/**
   CPU exception handler function type.

   @param type exception ID
   @param execptr faulty instruction pointer
   @param dataptr faulty memory access pointer
*/
typedef CPU_EXCEPTION_HANDLER(cpu_exception_handler_t);


/** Set hardware interrupt handler for the current cpu */
static void cpu_interrupt_hw_sethandler(cpu_interrupt_handler_t *hndl);
/** Set exception interrupt handler for the current cpu */
static void cpu_interrupt_ex_sethandler(cpu_exception_handler_t *hndl);
/** Set exception interrupt handler for the current cpu */
static void cpu_interrupt_sys_sethandler(cpu_interrupt_handler_t *hndl);

/** Disable all maskable interrupts for the current cpu */
static void cpu_interrupt_disable(void);
/** Enable all maskable interrupts for the current cpu */
static void cpu_interrupt_enable(void);
/** Save interrupts enable state (may use stack) */
static void cpu_interrupt_savestate(reg_t *state);
/** Save interrupts enable state end disable interrupts */
static void cpu_interrupt_savestate_disable(reg_t *state);
/** Restore interrupts enable state (may use stack) */
static void cpu_interrupt_restorestate(const reg_t *state);
/** read current interrupts state as boolean */
static bool_t cpu_interrupt_getstate(void);

/** enter interrupt wait state if supported, may return imediatly if
    unsupported */
static void cpu_interrupt_wait(void);

/** Save interrupts enable state end disable interrupts. This macro
    must be matched with the CPU_INTERRUPT_RESTORESTATE macro. */
#define CPU_INTERRUPT_SAVESTATE_DISABLE				\
{								\
  reg_t	__interrupt_state;				\
  cpu_interrupt_savestate_disable(&__interrupt_state);

/** Restore interrupts enable state. This macro must be matched with
    the CPU_INTERRUPT_SAVESTATE_DISABLE macro. */
#define CPU_INTERRUPT_RESTORESTATE				\
  cpu_interrupt_restorestate(&__interrupt_state);		\
}

#include "cpu/hexo/interrupt.h"

#endif

