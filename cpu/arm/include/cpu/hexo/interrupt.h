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

#ifdef CONFIG_DRIVER_ICU_ARM
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
	uint32_t tmp;

	asm volatile(
		"mrs  %0, cpsr        \n\t"
		"orr  %0, %0, #0x80   \n\t"
		"msr  cpsr, %0        \n\t"
		: "=r" (tmp) );
}

static inline void
cpu_interrupt_enable(void)
{
	uint32_t tmp;

	asm volatile(
		"mrs  %0, cpsr        \n\t"
		"bic  %0, %0, #0x80   \n\t"
		"msr  cpsr, %0        \n\t"
		: "=r" (tmp) );
}

static inline void
cpu_interrupt_savestate(reg_t *state)
{
	uint32_t tmp;

	asm volatile(
		"mrs  %0, cpsr        \n\t"
		: "=r" (tmp) );

	*state = tmp;
}

static inline void
cpu_interrupt_savestate_disable(reg_t *state)
{
	uint32_t tmp, result;

	asm volatile(
		"mrs  %1, cpsr        \n\t"
		"orr  %0, %1, #0x80   \n\t"
		"msr  cpsr, %0        \n\t"
		: "=r" (tmp), "=r" (result) );

	*state = result;
}

static inline void
cpu_interrupt_restorestate(const reg_t *state)
{
	asm volatile(
		"msr  cpsr, %0        \n\t"
		:: "r" (*state) );
}

static inline void
cpu_interrupt_process(void)
{
	reg_t state;
	cpu_interrupt_savestate(&state);
	cpu_interrupt_enable();
    /* memory clobber is important here as cpu_interrupt_process()
       will let pending intterupts change global variables checked in
       a function loop (scheduler root queue for instance) */
	__asm__ volatile ("nop":::"memory");
	cpu_interrupt_restorestate(&state);
}

static inline bool_t
cpu_interrupt_getstate(void)
{
	reg_t		state;

	asm volatile(
		"mrs  %0, cpsr        \n\t"
		: "=r" (state) );

	return !(state & 0x80);
}

static inline bool_t
cpu_is_interruptible(void)
{
	return cpu_interrupt_getstate();
}

static inline void
cpu_interrupt_wait(void)
{
#if defined(__ARM_ARCH_6K__)
	asm volatile(
		"mcr p15, 0, %0, c7, c0, 4"
		:: "r" (0) );
#else
	/**/
#endif
}

#endif

