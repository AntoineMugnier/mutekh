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

#include "cpu/hexo/specific.h"

struct cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  void	*unused1, *unused2, *unused3;

  asm volatile (
		".set push			\n"
		".set noat			\n"
		//		".set noreorder			\n"
#ifdef CONFIG_COMPILE_PIC
		/* save execution pointer based on current PC */
		"	addiu	$sp, 	-3*4		\n"
		"	bal	1f			\n"
		"	b	2f			\n"
		"1:	sw	$31,	2*4($sp)	\n"
#else
		/* save execution pointer based on static label address */
		"	addiu	$sp, 	-5*4		\n"
		"	la	$1,	2f		\n"
		"	sw	$1,	2*4($sp)	\n"
#endif
		"	lw	$15,	(%2)		\n"
		/* save status */
		"	mfc0	$1,	$12		\n"
		"	sw	$1,	1*4($sp)	\n"
		/* disable interrupts */
		"	ori	$1,	0x1		\n"
		"	addiu	$1,	-1		\n"
		"	mtc0	$1,	$12		\n"
		"	MTC0_WAIT			\n"
		/* save context local storage on stack */
		"	sw	$15,	0*4($sp)	\n"
		/* switch stack pointer */
		"	sw	$sp,	(%0)		\n"
		"	lw	$sp,	(%1)		\n"
		/* restore status & tls */
		"	lw	$1,	1*4($sp)	\n"
		"	lw	$15,	0*4($sp)	\n"
		"	mtc0	$1,	$12		\n"
		/* restore execution pointer */
		"	lw	$1,	2*4($sp)	\n"
		"	sw	$15,	(%2)		\n"
		"	addiu	$sp,	3*4		\n"
		"	jr	$1			\n"
		"2:					\n"
		".set pop				\n"
		: "=r" (unused1)
		, "=r" (unused2)
		, "=r" (unused3)

		: "0" (&old->stack_ptr)
		, "1" (&new->stack_ptr)
		, "2" (CPU_LOCAL_ADDR(__cpu_context_data_base))

#if defined(CONFIG_CPU_MIPS_ABI_O32) || defined(CONFIG_CPU_MIPS_ABI_O64)
		/* These GP registers will be saved by the compiler */
		: "$2", "$3"	/* return values */
		, /*"$4", "$5",  "$6",*/  "$7" /* arguments. r4 to r6 are left for asm input */
		, "$8",  "$9",  "$10",  "$11", "$12",  "$13",  "$14",  "$15" /* temp */
		, "$16", "$17", "$18", "$19", "$20", "$21", "$22", "$23", "$30" /* saved accros function call */
		, "$24", "$25"	/* temp */
		, "$31"		/* return value */
#elif defined(CONFIG_CPU_MIPS_ABI_N32) || defined(CONFIG_CPU_MIPS_ABI_N64)
		/* These GP registers will be saved by the compiler */
		: "$2", "$3",	/* FIXME add more registers */
		, /* "$4", "$5", "$6",*/  "$7",  "$8",  "$9",  "$10",  "$11", /* arguments. r4 to r6 are left for asm input */
		, "$12",  "$13",  "$14",  "$15" /* temp */
		, "$16", "$17", "$18", "$19", "$20", "$21", "$22", "$23", "$30" /* saved accros function call */
		, "$24", "$25"	/* temp */
		, "$31"		/* return value */
#else
# error Mips ABI support missing in context.h
#endif
		, "memory"
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  asm volatile (
		".set push			\n"
		".set noat			\n"
		"	move	$sp,	%0		\n"
		/* restore status & tls */
		"	lw	$1,	1*4($sp)	\n"
		"	lw	$15,	0*4($sp)	\n"
		"	mtc0	$1,	$12		\n"
		/* restore execution pointer */
		"	lw	$1,	2*4($sp)	\n"
		"	sw	$15,	(%1)		\n"
		"	addiu	$sp,	3*4		\n"
		"	jr	$1			\n"
		".set pop				\n"
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
		"	move	$sp,	%0	\n"
		"	j	%1		\n"
		:
		: "r" (stack)
		, "r" (jumpto)
		);
}

#endif

