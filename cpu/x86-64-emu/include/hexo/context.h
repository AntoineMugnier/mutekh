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

#include <hexo/local.h>

struct cpu_context_s
{
};

__asm__ (
	 ".macro PUSHA64	\n"
	 "push %rax		\n"
	 "push %rbx		\n"
	 "push %rcx		\n"
	 "push %rdx		\n"
	 "push %rsi		\n"
	 "push %rdi		\n"
	 "push %rbp		\n"
	 "push %r8		\n"
	 "push %r9		\n"
	 "push %r10		\n"
	 "push %r11		\n"
	 "push %r12		\n"
	 "push %r13		\n"
	 "push %r14		\n"
	 "push %r15		\n"
	 ".endm			\n"
	 );

__asm__ (
	 ".macro POPA64		\n"
	 "pop %r15		\n"
	 "pop %r14		\n"
	 "pop %r13		\n"
	 "pop %r12		\n"
	 "pop %r11		\n"
	 "pop %r10		\n"
	 "pop %r9		\n"
	 "pop %r8		\n"
	 "pop %rbp		\n"
	 "pop %rdi		\n"
	 "pop %rsi		\n"
	 "pop %rdx		\n"
	 "pop %rcx		\n"
	 "pop %rbx		\n"
	 "pop %rax		\n"
	 ".endm			\n"
	 );

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  asm volatile (
		/* save execution pointer */
#ifdef CONFIG_COMPILE_PIC
		"	callq	1f		\n"
		"	jmp	2f		\n"
		"1:				\n"
#else
		"	pushq	2f		\n"
#endif
		/* save flags */
		"	pushf			\n"
//		"	cli			\n" /* FIXME */
		/* save general purpose registers on stack */
		"	PUSHA64			\n"
		/* save context local storage on stack */
		"	push	(%2)		\n"
		/* switch stack pointer */
		"	movq	%%rsp, %0	\n"
		"	movq	%1, %%rsp	\n"
		/* restore tls */
		"	pop	(%2)		\n"
		/* restore general purpose registers */
		"	POPA64			\n"
		/* restore flags */
		"	popf			\n"
		/* restore execution pointer */
		"	retq			\n"
		"2:				\n"
		: "=m,m" (old->stack_ptr)
		: "r,m" (new->stack_ptr)
		, "r,r" (CPU_LOCAL_ADDR(__cpu_context_data_base))
		: "memory"
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  asm volatile (
		"	movq	%0, %%rsp	\n"
		/* restore tls */
		"	pop	(%1)		\n"
		/* restore general purpose registers */
		"	POPA64			\n"
 		/* restore flags */
		"	popf			\n"
		/* restore execution pointer */
		"	retq			\n"
		:
		: "r" (new->stack_ptr)
		, "r" (CPU_LOCAL_ADDR(__cpu_context_data_base))
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set_stack(uintptr_t stack, void *jumpto)
{
  asm volatile (
		"	movq	%0, %%rsp	\n"
		"	jmpq	*%1		\n"
		:
		: "r,m" (stack), "r,r" (jumpto)
		);
}

#endif

