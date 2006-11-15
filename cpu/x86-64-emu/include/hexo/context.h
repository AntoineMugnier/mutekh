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

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  register reg_t tmp0 asm("rbx");
  register reg_t tmp1 asm("r12");
  register reg_t tmp2 asm("r13");

  /* Note: gcc save and restore registers for us because all registers
     are marked clobbered in the asm statement. This will allow gcc to
     decide which registers must be saved so that we don't need to
     save _all_ registers ourself */

  asm volatile (
		/* save execution pointer */
#ifdef CONFIG_COMPILE_PIC
		"	callq	1f		\n"
		"	jmp	2f		\n"
		"1:				\n"
#else
		"	pushl	2f		\n"
#endif
		/* save flags */
		"	pushf			\n"
//		"	cli			\n" /* FIXME */
		/* save context local storage on stack */
		"	push	(%2)		\n"
		/* switch stack pointer */
		"	movq	%%rsp, (%0)	\n"
		"	movq	(%1), %%rsp	\n"
		/* restore tls */
		"	pop	(%2)		\n"
		/* restore flags */
		"	popf			\n"
		/* restore execution pointer */
		"	retq			\n"
		"2:				\n"

		/* these input registers will be clobbered */
		: "=r" (tmp0)
		, "=r" (tmp1)
		, "=r" (tmp2)

		/* input args */
		: "0" (&old->stack_ptr)
		, "1" (&new->stack_ptr)
		, "2" (CPU_LOCAL_ADDR(__cpu_context_data_base))

		/* remaining registers will be clobbered too */
		: "memory"
		, "%rax", /* "%rbx", */ "%rcx", "%rdx", "%rbp", "%rsi", "%rdi"
		, "%r8", "%r9", "%r10", "%r11", /* "%r12", */ /*"%r13", */ "%r14", "%r15"
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

