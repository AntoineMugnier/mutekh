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

#if !defined(INTERRUPT_H_) || defined(CPU_INTERRUPT_H_)
#error This file can not be included directly
#else

#define CPU_INTERRUPT_H_

#define CPU_EXCEPTION_ILLEGAL_INS  0
#define CPU_EXCEPTION_DATA_ERROR   1
#define CPU_EXCEPTION_INS_ERROR    2
#define CPU_EXCEPTION_DATA_ALIGN   3
#define CPU_EXCEPTION_IRQ          4
#define CPU_EXCEPTION_SYSCALL      5

#define CPU_FAULT_COUNT 6

# define CPU_FAULT_NAMES {       \
"Illegal instruction",          \
"Data storage",                 \
"Instruction storage",          \
"Alignment",                    \
"Irq",                          \
"Syscall",                      \
}

# include "hexo/local.h"

typedef reg_t cpu_irq_state_t;

ALWAYS_INLINE void
cpu_interrupt_disable(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t tmp;

  asm volatile (
		"mfmsr %0		\n\t"
                "ori %0, %0, 0x8000     \n\t"
                "xori %0, %0, 0x8000    \n\t"
		"mtmsr %0		\n\t"
		: "=r" (tmp)
                :: "memory"     /* compiler memory barrier */
	  );
# endif
}

ALWAYS_INLINE void
cpu_interrupt_enable(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t tmp;

  asm volatile (
		"mfmsr %0		\n\t"
                "ori %0, %0, 0x8000     \n\t"
		"mtmsr %0		\n\t"
		: "=r" (tmp)
                :
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
  reg_t tmp;

  asm volatile (
		"mfmsr %1		\n\t"
                "ori %0, %1, 0x8000     \n\t"
                "xori %0, %0, 0x8000    \n\t"
		"mtmsr %0		\n\t"
		: "=r" (tmp),
                  "=r" (*state)
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
		    "mtmsr	%0"
		    :
		    : "r" (*state)
                    : "memory"     /* compiler memory barrier */
		    );
  return (*state >> 15) & 1;
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
		    "mfmsr	%0"
		    : "=r" (state)
		    );

  return (state >> 15) & 1;
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

# ifdef CONFIG_CPU_WAIT_IRQ
ALWAYS_INLINE void cpu_interrupt_wait(void)
{
#  ifdef CONFIG_HEXO_IRQ
  reg_t tmp;
#   ifdef CONFIG_CPU_PPC_WAIT_OPCODE
  __asm__ volatile (
                    "mfmsr %0		\n"
                    "ori %0, %0, 0x8000     \n"
                    "mtmsr %0		\n"
                    "wait\n"	/* Power ISA 2.0 */
		    : "=r" (tmp)
		    :: "memory"
                    );
#   elif defined(CONFIG_CPU_PPC_WAIT_MSRWE)
  __asm__ volatile (
                    "mfmsr %0\n"
                    "or %0, %0, %1\n"
                    "mtmsr %0\n"
		    : "=&r" (tmp)
                    : "r" (0x48000) /* WE & int enable bits */
                    : "memory"
                    );
#   else
#    error CONFIG_CPU_WAIT_IRQ shall not be defined
#   endif
#  endif
}
# endif

#endif

