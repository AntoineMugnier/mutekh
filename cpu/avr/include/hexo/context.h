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
  register void	*tmp0, *tmp1;

  asm volatile (
		/* save execution pointer based on current PC */
		"	rcall	1f				\n"
		"	rjmp	2f				\n"
		"1:						\n"
		/* save flags */
		"	in	r0, 0x3f			\n"
		"	push	r0				\n"
		"	cli					\n"
		/* save context local storage on stack */
		"	lds	r0, __cpu_context_data_base + 1	\n"
		"	push	r0				\n"
		"	lds	r0, __cpu_context_data_base	\n"
		"	push	r0				\n"
		/* switch stack pointer */
		"	in	r0, 0x3d			\n"
		"	st	Z, r0				\n"
		"	in	r0, 0x3e			\n"
		"	std	Z + 1, r0			\n"
		"	out	0x3d, %A1			\n"
		"	out	0x3e, %B1			\n"
		/* restore tls */
		"	pop	r0				\n"
		"	sts	__cpu_context_data_base, r0	\n"
		"	pop	r0				\n"
		"	sts	__cpu_context_data_base + 1, r0	\n"
		/* restore flags */
		"	pop	r0				\n"
		"	out	0x3f, r0			\n"
		"	ret					\n"
		"2:						\n"

		: "=z" (tmp0)
		, "=y" (tmp1)

		: "0" (&old->stack_ptr)
		, "1" (new->stack_ptr)
		/* These GP registers will be saved by the compiler */
		: "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"
		, "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17"
		, "r18", "r19", "r20", "r21"
		, "r22", "r23"
#if 0
		, "r24", "r25"
		, "r26", "r27" /* "r28", "r29", "r30", "r31" used as input */
#endif
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  asm volatile (
		"	out	0x3d, %A0			\n"
		"	out	0x3e, %B0			\n"
		/* restore tls */
		"	pop	r0				\n"
		"	lds	__cpu_context_data_base, r0	\n"
		"	pop	r0				\n"
		"	lds	__cpu_context_data_base + 1, r0	\n"
		/* restore flags */
		"	pop	r0				\n"
		"	out	0x3f, r0			\n"
		"	ret					\n"
		"2:						\n"
		: 
		: "z" (new->stack_ptr)

		/* These GP registers will be saved by the compiler */
		: "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"
		, "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17"
		, "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25"
		, "r26", "r27", "r28", "r29" /* "r30", "r31" used as input */
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set_stack(uintptr_t stack, void *jumpto)
{
  asm volatile (
		"	out	0x3d, %A0			\n"
		"	out	0x3e, %B0			\n"
		"	ijmp					\n"
		:
		: "e" (stack)
		, "z" (jumpto)
		);
}

#endif

