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

struct cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  asm volatile (
		/* save execution pointer */
#ifdef CONFIG_COMPILE_PIC
		"	call	1f		\n"
		"	jmp	2f		\n"
		"1:				\n"
#else
		"	pushl	2f		\n"
#endif
		/* save flags */
		"	pushf			\n"
		"	cli			\n"
		/* save general purpose registers on stack */
		"	pusha			\n"
		/* save context local storage on stack */
		"	push	%%gs		\n"
		/* switch stack pointer */
		"	movl	%%esp, %0	\n"
		"	movl	%1, %%esp	\n"
		/* restore tls */
		"	pop	%%gs		\n"
		/* restore general purpose registers */
		"	popa			\n"
		/* restore flags */
		"	popf			\n"
		/* restore execution pointer */
		"	ret			\n"
		"2:				\n"
		: "=m,m" (old->stack_ptr)
		:  "r,m" (new->stack_ptr)
		: "memory"
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  asm volatile (
		"	movl	%0, %%esp	\n"
		/* restore tls */
		"	pop	%%gs		\n"
		/* restore general purpose registers */
		"	popa			\n"
		/* restore flags */
		"	popf			\n"
		/* restore execution pointer */
		"	ret			\n"
		:
		: "r" (new->stack_ptr)
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set_stack(uintptr_t stack, void *jumpto)
{
  asm volatile (
		"	movl	%0, %%esp	\n"
		"	jmpl	*%1		\n"
		:
		: "r,m" (stack), "r,r" (jumpto)
		);
}

#endif

