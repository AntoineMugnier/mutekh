/*
 *   This file is part of MutekH.
 *
 *   MutekH is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   MutekH is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with MutekH; if not, write to the Free Software Foundation,
 *   Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 *   Copyright Francois Charot <charot@irisa.fr>  (c) 2008
 *   INRIA Rennes Bretagne Atlantique
 *
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

#ifdef CONFIG_DRIVER_ICU_NIOS
struct device_s;
extern CPU_LOCAL struct device_s cpu_icu_dev;
#endif

void nios2_interrupt_entry(void);

static inline void
cpu_interrupt_disable(void)
{
  __asm__ volatile (
		    ".set noat 					\n"
		    "	rdctl	r1, status		\n"
		    "	ori	    r1, r1, 0x1     \n"
		    "	addi	r1, r1, -1      \n"
		    "	wrctl	status, r1		\n"

                    :
                    :
                    : "memory"     /* compiler memory barrier */
		    );
}

static inline void
cpu_interrupt_enable(void)
{
  __asm__ volatile (
		    ".set noat 					\n"
		    "	rdctl	r1, status		\n"
		    "	ori	    r1, r1, 0x1     \n"
		    "	wrctl	status, r1		\n"

                    :
                    :
                    : "memory"     /* compiler memory barrier */
		    );
}

static inline void
cpu_interrupt_process(void)
{
  cpu_interrupt_enable();
//  reg_t state;
//  cpu_interrupt_savestate(&state);
//  cpu_interrupt_enable();
  __asm__ volatile (
		    "nop"
		    :
		    :
		    /* memory clobber is important here as cpu_interrupt_process()
		       will let pending interrupts change global variables checked in
		       a function loop (scheduler root queue for instance) */
		    : "memory"
		    );
  cpu_interrupt_disable();
//  cpu_interrupt_restorestate(&state);
}

static inline void
cpu_interrupt_savestate(reg_t *state)
{
  __asm__ volatile (
		    "	rdctl	%0, status\n"
		    : "=r" (*state)
                    :
                    : "memory"     /* compiler memory barrier */
		    );
}

static inline void
cpu_interrupt_savestate_disable(reg_t *state)
{
  __asm__ volatile (
		    ".set noat 					\n"
		    "	rdctl	%0, status      \n"
		    "	ori	    r1, %0, 0x1     \n"
		    "	addi	r1, r1, -1      \n"
		    "	wrctl	status, r1		\n"
		  : "=r" (*state)
                    :
                    : "memory"     /* compiler memory barrier */
		    );
}

static inline void
cpu_interrupt_restorestate(const reg_t *state)
{
  __asm__ volatile (
		    "	wrctl	status, %0		\n"
		    :
		    : "r" (*state)
                    : "memory"     /* compiler memory barrier */
		    );
}

static inline bool_t
cpu_interrupt_getstate(void)
{
  reg_t state;

  __asm__ volatile (
		    "	rdctl	%0, status		\n"
		    : "=r" (state)
		    );

  return state & 0x1;
}

static inline bool_t
cpu_is_interruptible(void)
{
	return cpu_interrupt_getstate();

//	reg_t		state;
//
//  __asm__ volatile (
//		    "	rdctl	%0, status		\n"
//		    : "=r" (state)
//		    );
//  return ( (state & 0x1) );
}

#ifdef CONFIG_CPU_WAIT_IRQ
static inline void
cpu_interrupt_wait(void)
{
# ifdef CONFIG_HEXO_IRQ
  //  __asm__ volatile ("");
#endif
}
#endif

#endif

