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
#include "cpu/hexo/specific.h"

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

struct cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
	void *old_addr = &old->stack_ptr;
	void *new_addr = &new->stack_ptr;

	/*
	  stack should be
	   * entry point (r12)
	   * fp (r11)
	   * cpsr (r3)
	   * tls (r2)
	 */

	asm volatile (
		"adr  r12, 1f                      \n\t"
		"mrs  r3, cpsr                     \n\t"
		"mrc  p15,0,r2,c13,c0,4            \n\t"
		"push {r2, r3, r11, r12}           \n\t"
		"str  sp, [%0]                     \n\t"
#ifdef CONFIG_SOCLIB_MEMCHECK
			/* let memchecker know about context switch */
		"ldr  r2, =" ASM_STR(SOCLIB_MC_MAGIC_VAL) "  \n\t"
		"ldr  r3, =" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) " \n\t"
		"str  r2, [r3, #(" ASM_STR(SOCLIB_MC_MAGIC) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
		"str  %1, [r3, #(" ASM_STR(SOCLIB_MC_CTX_SET) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
		"mov  r2, #" ASM_STR(SOCLIB_MC_CHECK_SPFP) " \n\t"
		"str  r2, [r3, #(" ASM_STR(SOCLIB_MC_ENABLE) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
#endif
		"ldr  sp, [%1]                     \n\t"
#ifdef CONFIG_SOCLIB_MEMCHECK
		"mov  r2, #0 \n\t"
		"str  r2, [r3, #(" ASM_STR(SOCLIB_MC_MAGIC) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
#endif
		"pop  {r2, r3, r11, r12}           \n\t"
		"mcr  p15,0,r2,c13,c0,4            \n\t"
		"msr  cpsr, r3                     \n\t"
		"bx   r12                          \n\t"
		"1:                                \n\t"
		: 
		: "r"(old_addr), "r"(new_addr)
		/* These registers will be saved by the compiler */
		: /* "r0",  "r1",*/  "r2", "r3", "r4", "r5", "r6", "r7"
		, "r8", "r9", "r10"
# if !defined(CONFIG_COMPILE_FRAMEPTR) || defined(__OPTIMIZE__)
		, "r11"
# endif
		, "r12", /* "sp", */ "r14"
		, "memory"
		);
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
	void *new_addr = &new->stack_ptr;

	asm volatile (
#ifdef CONFIG_SOCLIB_MEMCHECK
		"ldr  r2, =" ASM_STR(SOCLIB_MC_MAGIC_VAL) "  \n\t"
		"ldr  r3, =" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) " \n\t"
		"str  r2, [r3, #(" ASM_STR(SOCLIB_MC_MAGIC) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
		"str  %0, [r3, #(" ASM_STR(SOCLIB_MC_CTX_SET) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
		"mov  r2, #" ASM_STR(SOCLIB_MC_CHECK_SPFP) " \n\t"
		"str  r2, [r3, #(" ASM_STR(SOCLIB_MC_ENABLE) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
#endif
		"ldr  sp, [%0]                     \n\t"
#ifdef CONFIG_SOCLIB_MEMCHECK
		"mov  r2, #0 \n\t"
		"str  r2, [r3, #(" ASM_STR(SOCLIB_MC_MAGIC) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
#endif
		"pop  {r2, r3, r11, r12}           \n\t"
		"mcr  p15,0,r2,c13,c0,4            \n\t"
		"msr  cpsr, r3                     \n\t"
		"bx   r12                          \n\t"
		:
		: "r"(new_addr)
		: "r2", "r3"
		);

	while(1)
		;
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set(uintptr_t stack, void *jumpto)
{
	asm volatile (
		"	mov	 sp, %0			\n"
		"	mov  ip, %1			\n"
		"	bx   ip				\n"
		:
		: "r" (stack)
		, "r" (jumpto)
		);
	while (1);
}

# if defined(CONFIG_CPU_USER)

/** kernel stack pointer value on user entry */
extern CONTEXT_LOCAL uintptr_t context_kstack;

void
__attribute__((noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack,
		     user_entry_t *entry, void *param);

# endif

#endif

