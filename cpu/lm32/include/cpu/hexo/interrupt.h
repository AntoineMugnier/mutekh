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

*/

#if !defined(INTERRUPT_H_) || defined(CPU_INTERRUPT_H_)
#error This file can not be included directly
#else

#define CPU_INTERRUPT_H_

# define CPU_FAULT_BREAKPOINT 0
# define CPU_FAULT_INS_BERR   1
# define CPU_FAULT_WATCHPOINT 2
# define CPU_FAULT_DATA_BERR  3
# define CPU_FAULT_DIV_BY_0   4
# define CPU_FAULT_SYSCALL    5
# define CPU_FAULT_LOST_IRQ   6

# define CPU_FAULT_COUNT 7

# define CPU_FAULT_NAMES {                      \
    "Break point",                              \
      "Ins bus error",                          \
      "Watch point",                            \
      "Data bus error",                         \
      "Divide by zero",                         \
      "Syscall",                                \
      "Lost irq",                               \
      }


# include "hexo/local.h"

typedef reg_t cpu_irq_state_t;

ALWAYS_INLINE void
cpu_interrupt_disable(void)
{
# ifdef CONFIG_HEXO_IRQ

  asm volatile ("wcsr    IE, r0"
                ::: "memory"     /* compiler memory barrier */
	  );
# endif
}

ALWAYS_INLINE void
cpu_interrupt_enable(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t tmp = 1;

  asm volatile ("wcsr   IE, %0          \n\t"
                :
                : "r" (tmp)
                : "memory"     /* compiler memory barrier */
                );
# endif
}

ALWAYS_INLINE void
cpu_interrupt_process(void)
{
# ifdef CONFIG_HEXO_IRQ
  cpu_interrupt_enable();
  __asm__ volatile ("nop"
		    :
		    :
    /* memory clobber is important here as cpu_interrupt_process()
       will let pending intterupts change global variables checked in
       a function loop (scheduler root queue for instance) */
		    : "memory"
		    );
  cpu_interrupt_disable();
# endif
}

ALWAYS_INLINE void
cpu_interrupt_savestate_disable(cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ

  asm volatile (	
                "rcsr   %0, IE		\n\t"
		"wcsr   IE, r0          \n\t"
                : "=r" (*state)
                :
                : "memory"     /* compiler memory barrier */
	  );
# endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_restorestate(const cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
                    "wcsr IE, %0           \n\t"
		    :
		    : "r" (*state)
                    : "memory"     /* compiler memory barrier */
		    );

  return *state & 1;
# else
  return 0;
# endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_getstate(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t		state;

  __asm__ volatile (
                    "rcsr   %0, IE		\n\t"
		    : "=r" (state)
		    );

  return state & 1;
# else
  return 0;
# endif
}

ALWAYS_INLINE bool_t
cpu_is_interruptible(void)
{
# ifdef CONFIG_HEXO_IRQ
	return cpu_interrupt_getstate();
# else
	return 0;
# endif
}

#endif

