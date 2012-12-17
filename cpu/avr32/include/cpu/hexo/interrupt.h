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

# define CPU_FAULT_UNRECOV_EXCEPT 0
# define CPU_FAULT_TLB_MULTI_HITS 1
# define CPU_FAULT_BERR_DATA      2
# define CPU_FAULT_BERR_INS       3
# define CPU_FAULT_INS_ADDR       4
# define CPU_FAULT_ITLB_MISS      5
# define CPU_FAULT_ITLB_PROT      6 
# define CPU_FAULT_BREAKPOINT     7 
# define CPU_FAULT_ILL_OPCODE     8 
# define CPU_FAULT_UNIMP_INS      9 
# define CPU_FAULT_PRIVILEGE      10
# define CPU_FAULT_FPU            11
# define CPU_FAULT_MISSING_COPROC 12
# define CPU_FAULT_DATA_ADDR_RD   13
# define CPU_FAULT_DATA_ADDR_WR   14
# define CPU_FAULT_DTLB_MISS_RD   15
# define CPU_FAULT_DTLB_MISS_WR   16
# define CPU_FAULT_DTLB_PROT_RD   17
# define CPU_FAULT_DTLB_PROT_WR   18
# define CPU_FAULT_DTLB_MODIFY    19

# define CPU_FAULT_COUNT          20

#ifndef __MUTEK_ASM__

# define CPU_FAULT_NAMES {                      \
    "Unrecoverable exception",                  \
    "TLB multiple hit",                         \
    "Bus error data fetch",                     \
    "Bus error instruction fetch",              \
    "Instruction Address",                      \
    "ITLB Miss",                                \
    "ITLB Protection",                          \
    "Break point",                              \
    "Illegal Opcode",                           \
    "Unimplemented instruction",                \
    "Privilege violation",                      \
    "Floating-point",                           \
    "Coprocessor absent",                       \
    "Data Address (Read)",                      \
    "Data Address (Write)",                     \
    "DTLB Miss (Read)",                         \
    "DTLB Miss (Write)",                        \
    "DTLB Protection (Read)",                   \
    "DTLB Protection (Write)",                  \
    "DTLB Modified"                             \
    }

# ifdef CONFIG_DRIVER_ICU_AVR32
struct device_s;
extern CPU_LOCAL struct device_s cpu_icu_dev;
# endif

# include "hexo/local.h"

static inline void
cpu_interrupt_disable(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t tmp;

  asm volatile ("mustr   %0\n"
                "sbr     %0, 0\n"
                "musfr   %0\n"
                : "=r" (tmp)
                :
                : "memory"     /* compiler memory barrier */
	  );
# endif
}

static inline void
cpu_interrupt_enable(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t tmp = 1;

  asm volatile ("mustr   %0\n"
                "cbr     %0, 0\n"
                "musfr   %0\n"
                :
                : "r" (tmp)
                : "memory"     /* compiler memory barrier */
	  );
# endif
}

static inline void
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

static inline void
cpu_interrupt_savestate(reg_t *state)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
                    "mustr %0		\n"
		    : "=r" (*state)
		    );
# endif
}

static inline void
cpu_interrupt_savestate_disable(reg_t *state)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t tmp;

  asm volatile (	
                "mustr   %1     \n"
                "mov     %0, %1 \n"
                "sbr     %1, 0  \n"
                "musfr   %1     \n"
                : "=r" (*state),  "=r" (tmp)
                :
                : "memory"     /* compiler memory barrier */
	  );
# endif
}

static inline void
cpu_interrupt_restorestate(const reg_t *state)
{
# ifdef CONFIG_HEXO_IRQ
  __asm__ volatile (
                    "musfr %0  \n"
		    :
		    : "r" (*state)
                    : "memory"     /* compiler memory barrier */
		    );
# endif
}

static inline bool_t
cpu_interrupt_getstate(void)
{
# ifdef CONFIG_HEXO_IRQ
  reg_t		state;

  __asm__ volatile (
                    "mustr %0		\n"
		    : "=r" (state)
		    );

  return !(state & 1);
# else
  return 0;
# endif
}

static inline bool_t
cpu_is_interruptible(void)
{
# ifdef CONFIG_HEXO_IRQ
	return cpu_interrupt_getstate();
# else
	return 0;
# endif
}

#endif  /* __MUTEK_ASM__ */

#endif

