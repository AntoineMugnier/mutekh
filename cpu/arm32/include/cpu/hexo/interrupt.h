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

#include "asm.h"

# define CPU_EXCEPTION_ILLEGAL_INS  0
# define CPU_EXCEPTION_DATA_ERROR   1
# define CPU_EXCEPTION_INS_ERROR    2
# define CPU_FAULT_COUNT 3

# define CPU_FAULT_NAMES {			\
      "Illegal instruction",                    \
      "Data abort",				\
      "Ins abort",                              \
      }

void arm_interrupt_entry(void);

typedef reg_t cpu_irq_state_t;

ALWAYS_INLINE void
cpu_interrupt_disable(void)
{
# ifdef CONFIG_HEXO_IRQ

#  if defined(CONFIG_CPU_ARM32_THUMB) && (CONFIG_CPU_ARM32_ARCH_VERSION >= 7)
	asm volatile ("cpsid i" ::: "memory");
#  else
	uint32_t tmp;
        THUMB_TMP_VAR;
	asm volatile (
                      THUMB_TO_ARM
                      "mrs  %[tmp], cpsr            \n\t"
                      "orr  %[tmp], %[tmp], # 0x80   \n\t"
                      "msr  cpsr, %[tmp]            \n\t"
                      ARM_TO_THUMB
                      : [tmp] "=r" (tmp) /*,*/ THUMB_OUT(,)
                      :: "memory"     /* compiler memory barrier */
                     );
#  endif
# endif
}

ALWAYS_INLINE void
cpu_interrupt_enable(void)
{
# ifdef CONFIG_HEXO_IRQ

#  if defined(CONFIG_CPU_ARM32_THUMB) && (CONFIG_CPU_ARM32_ARCH_VERSION >= 7)
	asm volatile ("cpsie i" ::: "memory");
#  else
	uint32_t tmp;
        THUMB_TMP_VAR;
	asm volatile (
                      THUMB_TO_ARM
                      "mrs  %[tmp], cpsr            \n\t"
                      "bic  %[tmp], %[tmp], #0x80   \n\t"
                      "msr  cpsr, %[tmp]            \n\t"
                      ARM_TO_THUMB
                      : [tmp] "=r" (tmp) /*,*/ THUMB_OUT(,)
                      :: "memory"     /* compiler memory barrier */
                     );
#  endif
# endif
}

ALWAYS_INLINE void
cpu_interrupt_savestate_disable(cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ
	uint32_t result;

	uint32_t tmp;
        THUMB_TMP_VAR;
	asm volatile (
                      THUMB_TO_ARM
		      "mrs  %[result], cpsr            \n\t"
		      "orr  %[tmp], %[result], #0x80   \n\t"
		      "msr  cpsr, %[tmp]               \n\t"
                      ARM_TO_THUMB
		      : [tmp] "=r" (tmp), [result] "=r" (result) /*,*/ THUMB_OUT(,)
                      :: "memory"     /* compiler memory barrier */
                     );

	*state = result;
# endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_restorestate(const cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ

        THUMB_TMP_VAR;
	asm volatile (
                      THUMB_TO_ARM
                      "msr  cpsr, %[state]        \n\t"
                      ARM_TO_THUMB
                      /* : */ THUMB_OUT(:)
                      : [state] "r" (*state)
                      : "memory"     /* compiler memory barrier */
                     );

	return !(*state & 0x80);
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

        THUMB_TMP_VAR;
	asm volatile (
                      THUMB_TO_ARM
                      "mrs  %[state], cpsr        \n\t"
                      ARM_TO_THUMB
                      : [state] "=r" (state) /*,*/ THUMB_OUT(,)
                     );
	return !(state & 0x80);
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


ALWAYS_INLINE void cpu_interrupt_wait(void)
{
# if defined(CONFIG_HEXO_IRQ) && (CONFIG_CPU_ARM32_ARCH_VERSION >= 6)
        THUMB_TMP_VAR;
	asm volatile (
                      THUMB_TO_ARM
                      /* work with interrupts disabled in order to avoid race */
		      "mcr p15, 0, %[zero], c7, c0, 4  \n\t"
                      ARM_TO_THUMB
                      /* : */  THUMB_OUT(:)
                      : [zero] "r" (0)
                      : "memory"
                     );
        /* need to reenable interrupts so that the handler is executed */
        cpu_interrupt_enable();
# endif
}

#endif

