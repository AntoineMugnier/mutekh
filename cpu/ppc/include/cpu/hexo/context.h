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

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

#include <hexo/cpu.h>

struct cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  void	*unused1, *unused2;

  asm volatile (
		"	addi	1, 1, -7*4		\n"
#ifndef CONFIG_COMPILE_PIC
#warning To be tested
		/* save execution pointer based on current PC */
		"	bl	1f			\n"
		"	b	2f			\n"
		"1:	mflr	0			\n"
#else
		/* save execution pointer based on static label address */
		"	lis	0, 2f@ha		\n"
		"	la	0, 2f@l(0)		\n"
#endif
		"	stw	0, 6*4(1)		\n"
		/* save r30, r31, r14, r15 */
		"	stw	14, 5*4(1)		\n"
		"	stw	15, 4*4(1)		\n"
		"	stw	31, 3*4(1)		\n"
		"	stw	30, 2*4(1)		\n"
		/* save status */
		"	mfmsr	0			\n"
		"	stw	0, 1*4(1)		\n"

		/* disable interrupts */
		"	andi.	2, 0, 0x8000		\n"
		"	xor	0, 2, 0			\n"
		"	mtmsr	0			\n"
		/* save context local storage on stack */
		"	mfspr	0, 0x114		\n"
		"	stw	0, 0*4(1)		\n"
		/* switch stack pointer */
		"	stw	1, 0(%0)		\n"
		"	lwz	1, 0(%1)			\n"
		/* restore status & tls */
		"	lwz	0, 0*4(1)		\n"
		"	mtspr	0x114, 0		\n" /* SPRG4 is tls */
		"	lwz	0, 1*4(1)		\n"
		"	mtmsr	0			\n"
		/* restore r30, r31, r14, r15 */
		"	lwz	30, 2*4(1)		\n"
		"	lwz	31, 3*4(1)		\n"
		"	lwz	15, 4*4(1)		\n"
		"	lwz	14, 5*4(1)		\n"
		/* Restore execution pointer */
		"	lwz	0, 6*4(1)		\n"
		"	addi	1, 1, 7*4		\n"
		"	mtctr	0			\n"
		"	bctrl				\n"
		"2:					\n"
		: "=r" (unused1)
		, "=r" (unused2)

		: "0" (&old->stack_ptr)
		, "1" (&new->stack_ptr)

		/* These registers will be saved by the compiler */
		: "r0", "r2", "r3", "r4", "r5", "r6", "r7"
		, "r8", "r9", "r10", "r11", "r12", "r13" /*, "r14", "r15" */
	        , "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23"
		, "r24", "r25", "r26", "r27", "r28", "r29" /*, "r30", "r31" */
		, "cr0", "cr1", "cr2", "cr3", "cr4", "cr5", "cr6", "cr7"
		, "lr", "ctr"
		, "memory"
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  asm volatile (
		"	mr	1, %0			\n"
		/* restore status & tls */
		"	lwz	0, 0*4(1)		\n"
		"	mtspr	0x114, 0		\n" /* SPRG4 is tls */
		"	lwz	0, 1*4(1)		\n"
		"	mtmsr	0			\n"
		/* restore r13, r14, r15 */
		"	lwz	30, 2*4(1)		\n"
		"	lwz	31, 3*4(1)		\n"
		"	lwz	15, 4*4(1)		\n"
		"	lwz	14, 5*4(1)		\n"
		/* Restore execution pointer */
		"	lwz	0, 6*4(1)		\n"
		"	addi	1, 1, 7*4		\n"
		"	mtctr	0			\n"
		"	bctrl				\n"
		:
		: "r" (new->stack_ptr)
		);
  while (1);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set(uintptr_t stack, void *jumpto)
{
  asm volatile (
		"	mr	1, %0			\n"
		"	mtctr	%1			\n"
		"	bctrl				\n"
		:
		: "r" (stack)
		, "r" (jumpto)
		);
  while (1);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack, uintptr_t jumpto)
{
  cpu_trap();			/* FIXME not supported */
}

#endif

