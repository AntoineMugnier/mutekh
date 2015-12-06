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

/** @multiple x86 cpu exception id */
#define CPU_EXCEPTION_DIVIDE_ERROR          0
#define CPU_EXCEPTION_DEBUG                 1
#define CPU_EXCEPTION_NMI_INTERRUPT         2
#define CPU_EXCEPTION_BREAKPOINT            3
#define CPU_EXCEPTION_OVERFLOW              4
#define CPU_EXCEPTION_BOUND_RANGE           5
#define CPU_EXCEPTION_INVALID_OPCODE        6
#define CPU_EXCEPTION_NO_FPU                7
#define CPU_EXCEPTION_DOUBLE_FAULT          8
#define CPU_EXCEPTION_COPROCESSOR_SEGMENT   9
#define CPU_EXCEPTION_INVALID_TSS           10
#define CPU_EXCEPTION_NO_SEGMENT            11
#define CPU_EXCEPTION_STACK_SEGMENT         12
#define CPU_EXCEPTION_GENERAL_PROTECTION    13
#define CPU_EXCEPTION_PAGE_FAULT            14
#define CPU_EXCEPTION_INTEL_RESERVED        15
#define CPU_EXCEPTION_X87_FPU_ERROR         16
#define CPU_EXCEPTION_ALIGNMENT_CHECK       17
#define CPU_EXCEPTION_MACHINE_CHECK         18
#define CPU_EXCEPTION_SIMD_FP_EXCEPTION     19

/** first interrupt vector used for exceptions */
#define CPU_EXCEPT_VECTOR		0
/** exceptions vector count */
#define CPU_EXCEPT_VECTOR_COUNT		32

/** first interrupt vector used for hardware interrupts */
#define CPU_HWINT_VECTOR		(CPU_EXCEPT_VECTOR_COUNT)
/** hardware interrupts vector count */
#define CPU_HWINT_VECTOR_COUNT		96

/** first interrupt vector used for system calls */
#define CPU_SYSCALL_VECTOR		(CPU_EXCEPT_VECTOR_COUNT + CPU_HWINT_VECTOR_COUNT)
/** syscall vector count */
#define CPU_SYSCALL_VECTOR_COUNT	128

/** max interrupt line handled by the CPU */
#define CPU_MAX_INTERRUPTS		256

/** interrupts entry trampoline size  */
#define CPU_INTERRUPT_ENTRY_ALIGN	16

/** direct iret entry, used for cpu wakeup ipis */
#define CPU_HWINT_VECTOR_IRET           0x5f


# include <hexo/local.h>

void x86_interrupt_hw_entry(void);
void x86_interrupt_ex_entry(void);
void x86_interrupt_sys_entry(void);
void x86_interrupt_sys_enter(void);

typedef reg_t cpu_irq_state_t;

ALWAYS_INLINE void
cpu_interrupt_disable(void)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
                    "cli"
                    :
                    :
                    : "memory" /* compiler memory barrier */
		    );
# endif
}

ALWAYS_INLINE void
cpu_interrupt_enable(void)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
                    "sti"
                    :
                    :
                    : "memory" /* compiler memory barrier */
		    );
# endif
}

ALWAYS_INLINE void
cpu_interrupt_process(void)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
		    "sti\n"
    /* nop is required here to let enough time for pending interrupts
       to execute on some processors */
		    "nop\n"
		    "nop\n"
		    :
		    :
    /* memory clobber is important here as cpu_interrupt_process()
       will let pending intterupts change global variables checked in
       a function loop (scheduler root queue for instance) */
		    : "memory"
		    );
# endif
}

ALWAYS_INLINE void
cpu_interrupt_savestate_disable(cpu_irq_state_t *state)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
		    "pushfl	\n"
		    "popl	%0\n"
                    "cli        \n"
		    : "=m,r" (*state)
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
		    "pushl	%0\n"
		    "popfl	\n"
		    :
		    : "m,r" (*state)
                    : "memory"     /* compiler memory barrier */
		    );

  return (*state >> 9) & 1;
# else
  return 0;
# endif
}

ALWAYS_INLINE bool_t
cpu_interrupt_getstate(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t		flags;

  __asm__ volatile (
		    "pushfl	\n"
		    "popl	%0\n"
		    : "=r" (flags)
		    );

  return (flags >> 9) & 1;
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
  /* sti ; hlt is guaranteed to be atomic */
  __asm__ volatile ("sti; hlt \n"
		    ::: "memory"
		    );
#  endif
}
# endif

#endif

