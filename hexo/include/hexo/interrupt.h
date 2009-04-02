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
 * @file
 * @module{Hexo}
 * @short Interrupts, exceptions and syscall events management
 */

#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "local.h"
#include "types.h"

/************************************************************ hw irq */

/** CPU interrupt handler function template */
#define CPU_INTERRUPT_HANDLER(n) void (n) (uint_fast8_t irq)

/**
   CPU interrupt handler function type.

   @param irq interrupt line number
*/
typedef CPU_INTERRUPT_HANDLER(cpu_interrupt_handler_t);

/** Set hardware interrupt handler for the current cpu */
static inline void cpu_interrupt_sethandler(cpu_interrupt_handler_t *hndl);

/** Disable all maskable interrupts for the current cpu */
static inline void cpu_interrupt_disable(void);
/** Enable all maskable interrupts for the current cpu */
static inline void cpu_interrupt_enable(void);
/** Save interrupts enable state (may use stack) */
static inline void cpu_interrupt_savestate(reg_t *state);
/** Save interrupts enable state end disable interrupts */
static inline void cpu_interrupt_savestate_disable(reg_t *state);
/** Restore interrupts enable state (may use stack) */
static inline void cpu_interrupt_restorestate(const reg_t *state);
/** read current interrupts state as boolean */
static inline bool_t cpu_interrupt_getstate(void);
/** tell whether cpu is interruptible */
static inline bool_t cpu_is_interruptible(void);

/** enable interrupts and give a change to pending requests to
    execute. This function must be used to avoid the "sti; cli"
    syndrome which makes interrupts execution impossible on some
    procesors. Memory is marked as clobbered by this function 
    to force global variable reload after interrupts processing. */
static inline void cpu_interrupt_process(void);

/** enter interrupt wait state if supported, may return imediatly if
    unsupported */
static inline void cpu_interrupt_wait(void);

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


/************************************************************ exceptions */

/** CPU exception handler function template */
#define CPU_EXCEPTION_HANDLER(n) void (n) (uint_fast8_t type, uintptr_t execptr, \
					   uintptr_t dataptr, reg_t *regtable, \
					   uintptr_t stackptr)
/**
   CPU exception handler function type.

   @param type exception ID
   @param execptr faulty instruction pointer
   @param dataptr faulty memory access pointer
   @param regtable register table
   @param stackptr value of stack pointer
*/
typedef CPU_EXCEPTION_HANDLER(cpu_exception_handler_t);


/** Set exception interrupt handler for the current cpu */
static void cpu_exception_sethandler(cpu_exception_handler_t *hndl);



/************************************************************ syscalls */

#include <hexo/context.h>

/** CPU syscall handler function template */
#define CPU_SYSCALL_HANDLER(n) void (n) (uint_fast8_t number, reg_t *regtable)

/**
   CPU syscall handler function type.

   @param irq interrupt line number
*/
typedef CPU_SYSCALL_HANDLER(cpu_syscall_handler_t);

extern CONTEXT_LOCAL cpu_syscall_handler_t  *cpu_syscall_handler;

/** Set syscall interrupt handler for the current _context_ */
static inline void
cpu_syscall_sethandler(cpu_syscall_handler_t *hndl)
{
  CONTEXT_LOCAL_SET(cpu_syscall_handler, hndl);
}

struct context_s;

/** Set syscall interrupt handler for a given context */
void
cpu_syscall_sethandler_ctx(struct context_s *context,
			   cpu_syscall_handler_t *hndl);

/************************************************************/

#include "cpu/hexo/interrupt.h"

#endif

