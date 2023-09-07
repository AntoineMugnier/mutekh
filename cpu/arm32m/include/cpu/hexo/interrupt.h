/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

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

# define CPU_EXCEPTION_NMI          2
# define CPU_EXCEPTION_HARDFAULT    3
# define CPU_EXCEPTION_MEMMANAGE    4
# define CPU_EXCEPTION_BUSFAULT     5
# define CPU_EXCEPTION_USAGEFAULT   6
# define CPU_FAULT_COUNT 7

# define CPU_FAULT_NAMES {			\
      "", "",                                   \
      "NMI",                                    \
      "HardFault",                              \
      "MemManage",                              \
      "BusFault",                               \
      "UsageFault",                             \
      }

void arm_interrupt_restore(void);
void arm_interrupt_entry(void);

typedef reg_t cpu_irq_state_t;

ALWAYS_INLINE void
cpu_interrupt_disable(void)
{
# ifdef CONFIG_HEXO_IRQ
	asm volatile ("cpsid i" ::: "memory");
# endif
}

ALWAYS_INLINE void
cpu_interrupt_enable(void)
{
# ifdef CONFIG_HEXO_IRQ
	asm volatile ("cpsie i" ::: "memory");
# endif
}

ALWAYS_INLINE void
cpu_interrupt_savestate_disable(cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ
	uint32_t result;

	asm volatile (
                      "mrs  %[result], primask     \n\t"
                      "cpsid i                     \n\t"
		      : [result] "=l" (result)
                      :: "memory"     /* compiler memory barrier */
                     );

	*state = result;
# endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_restorestate(const cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ
        reg_t s = *state;
#  ifdef CONFIG_HEXO_CONTEXT_IRQEN
        if (!s)
          arm_interrupt_restore();
#  endif
	asm volatile (
                      "msr  primask, %[state]        \n\t"
                      : : [state] "l" (s)
                      : "memory"     /* compiler memory barrier */
                     );
	return ~*state & 1;
# else
	return 0;
# endif
}

ALWAYS_INLINE void
cpu_interrupt_process(void)
{
# ifdef CONFIG_HEXO_IRQ
	cpu_interrupt_enable();
    /* memory clobber is important here as cpu_interrupt_process()
       will let pending intterupts change global variables checked in
       a function loop (scheduler root queue for instance) */
	__asm__ volatile ("nop":::"memory");
# endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_getstate(void)
{
# ifdef CONFIG_HEXO_IRQ
	reg_t		state;

	asm volatile (
                      "mrs  %[state], primask        \n\t"
                      : [state] "=l" (state)
                     );
	return ~state & 1;
# else
	return 0;
# endif
}

ALWAYS_INLINE bool_t
cpu_is_interruptible(void)
{
# ifdef CONFIG_HEXO_IRQ
	reg_t		primask, ipsr;
	asm volatile (
                      "mrs  %0, primask        \n\t"
                      "mrs  %1, ipsr           \n\t"
                      : "=l" (primask), "=l" (ipsr)
                     );
	return !(primask & 1) && !(ipsr & 0xff);
# else
	return 0;
# endif
}


ALWAYS_INLINE void cpu_interrupt_wait(void)
{
# ifdef CONFIG_HEXO_IRQ

        cpu_interrupt_enable();
	asm volatile (/* wait for event */
#if 0
                      "wfi \n\t"
                      "sev \n\t"
                      "wfe \n\t"
#else
                      "wfe \n\t"
                      /* force clear of the event flag */
                      "sev \n\t"
                      "wfe \n\t"
#endif
                      ::: "memory");

# endif
}

#endif

