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

#include <arch/hexo/specific.h>

struct cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  reg_t	tmp0, tmp1;

  /* Note: gcc save and restore registers for us because all registers
     are marked clobbered in the asm statement. This will allow gcc to
     decide which registers must be saved so that we don't need to
     save _all_ registers ourself */

  asm volatile (
		/* save execution pointer */
#ifdef CONFIG_COMPILE_PIC
		"	call	1f		\n"
		"	jmp	2f		\n"
		"1:				\n"
#else
		"	pushl	2f		\n"
#endif
#ifdef CONFIG_COMPILE_FRAMEPTR
		/* save frame pointer */
		"	push	%%ebp		\n"
#endif
		/* save flags */
		"	pushf			\n"
		"	cli			\n"
#if 0
		/* save page directory pointer */
		"	movl	%%cr3, %%eax	\n"
		"	pushl	%%eax		\n"
#endif
		/* save context local storage on stack */
		"	push	%%gs		\n"
		/* switch stack pointer */
		"	movl	%%esp, (%0)	\n"
		"	movl	(%1), %%esp	\n"
		/* restore tls */
		"	pop	%%gs		\n"
#if 0
		/* restore page directory pointer */
		"	popl	%%edx		\n"
		"	cmpl	%%edx, %%eax	\n" /* avoid useless TLB flush */
		"	jz	3f		\n"
		"	movl	%%edx, %%cr3	\n"
		"3:				\n"
#endif
		/* restore flags */
		"	popf			\n"
#ifdef CONFIG_COMPILE_FRAMEPTR
		/* restore frame pointer */
		"	pop	%%ebp		\n"
#endif
		/* restore execution pointer */
		"	ret			\n"
		"2:				\n"

		/* these input registers will be clobbered */
		: "=b" (tmp0)
		, "=S" (tmp1)

		/* input args */
		: "0" (&old->stack_ptr)
		, "1" (&new->stack_ptr)

		/* remaining registers will be clobbered too */
		: "memory"
		, "%eax", /* "%ebx", */ "%ecx", "%edx"
		, /* "%esi", */ "%edi", "%ebp"
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
#if 0
		/* restore page directory pointer */
		"	popl	%%edx		\n"
		"	movl	%%edx, %%cr3	\n"
#endif
		/* restore flags */
		"	popf			\n"
#ifdef CONFIG_COMPILE_FRAMEPTR
		/* restore frame pointer */
		"	pop	%%ebp		\n"
#endif
		/* restore execution pointer */
		"	ret			\n"
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
		"	movl	%0, %%esp	\n"
#ifdef CONFIG_COMPILE_FRAMEPTR
		"	xorl	%%ebp, %%ebp	\n"
#endif
		"	jmpl	*%1		\n"
		:
		: "r,m" (stack), "r,r" (jumpto)
		);

  while (1);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack, uintptr_t jumpto)
{
  asm volatile (
#ifdef CONFIG_CPU_X86_SYSENTER
		"sysexit		\n"
		:
		: "c" (ustack)
		, "d" (jumpto)
#else
		"pushl %0		\n" /* SS */
		"pushl %1		\n" /* ESP */
		"pushl %2		\n" /* CS */
		"pushl %3		\n" /* EIP */
		"lret			\n"
		:
		: "r" ((ARCH_GDT_USER_DATA_INDEX << 3) | 3)
		, "r" (ustack)
		, "r" ((ARCH_GDT_USER_CODE_INDEX << 3) | 3)
		, "r" (jumpto)
#endif
		);

  while (1);
}

#endif

