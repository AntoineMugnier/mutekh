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

#include <hexo/local.h>
#include <hexo/ordering.h>

typedef reg_t cpu_irq_state_t;

#ifdef CONFIG_HEXO_IRQ
void emu_interrupts_wait(void);
void emu_interrupts_init(void);
void emu_interrupts_set(bool_t state);
bool_t emu_interrupts_get(void);
void emu_interrupts_post(cpu_id_t cpu, uint_fast8_t irq);
#endif

ALWAYS_INLINE void
cpu_interrupt_disable(void)
{
#ifdef CONFIG_HEXO_IRQ
  emu_interrupts_set(0);
  order_compiler_mem();
#endif
}

ALWAYS_INLINE void
cpu_interrupt_enable(void)
{
#ifdef CONFIG_HEXO_IRQ
  order_compiler_mem();
  emu_interrupts_set(1);
#endif
}

ALWAYS_INLINE void
cpu_interrupt_process(void)
{
  cpu_interrupt_enable();
}

ALWAYS_INLINE void
cpu_interrupt_savestate_disable(cpu_irq_state_t *state)
{
#ifdef CONFIG_HEXO_IRQ
  *state = emu_interrupts_get();
  cpu_interrupt_disable();
  order_compiler_mem();
#endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_restorestate(const cpu_irq_state_t *state)
{
#ifdef CONFIG_HEXO_IRQ
  order_compiler_mem();
  emu_interrupts_set(*state);
  return *state;
#else
  return 0;
#endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_getstate(void)
{
#ifdef CONFIG_HEXO_IRQ
  return emu_interrupts_get();
#else
  return 0;
#endif
}

ALWAYS_INLINE bool_t
cpu_is_interruptible(void)
{
#ifdef CONFIG_HEXO_IRQ
  return cpu_interrupt_getstate();
#else
  return 0;
#endif
}

#ifdef CONFIG_CPU_WAIT_IRQ
ALWAYS_INLINE void
cpu_interrupt_wait(void)
{
# ifdef CONFIG_HEXO_IRQ
  emu_interrupts_wait();
# endif
}
#endif

#endif

