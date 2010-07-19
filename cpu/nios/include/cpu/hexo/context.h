/*
 *   This file is part of MutekH.
 *
 *   MutekH is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   MutekH is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with MutekH; if not, write to the Free Software Foundation,
 *   Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 *   Copyright Francois Charot <charot@irisa.fr>  (c) 2008
 *   INRIA Rennes Bretagne Atlantique
 *
 */


#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

#include <hexo/cpu.h>


#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

struct cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  void	*unused1, *unused2, *unused3;

  __asm__ volatile (
		    ".set   noat                                \n"
#ifdef CONFIG_COMPILE_PIC_
		    /* save execution pointer based on current PC */
		    "	addi    sp, sp, -4*4                    \n"
		    "	stw     ra, 3*4(sp)                     \n"
		    "	nextpc  r31                             \n"
		    "	jmpi    1f                              \n"
		    "	jmpi    2f                              \n"
		    "1:	nop                                     \n"
#else
		    /* save execution pointer based on static label address */
		    "	addi    sp, sp, -4*4                    \n"
		    "	movia   r1, 2f                          \n"
		    "	stw     r1, 3*4(sp)                     \n"
#endif
		    "	ldw     r16, (%2)                       \n"
		    /* save frame pointer */
		    "	stw     r28, 2*4(sp)                    \n"
		    /* save status */
		    "	rdctl   r1, status                      \n"
		    "	stw     r1, 1*4(sp)                     \n"
		    /* disable interrupt */
		    "	ori     r1, r1, 0x1                     \n"
		    "	addi    r1, r1, -1                      \n"
		    "   wrctl   status, r1                      \n"
		    /* save context local storage on stack */
		    "	stw     r16, 0*4(sp)                    \n"

#ifdef CONFIG_SOCLIB_MEMCHECK
		    /* enter memchecker command mode */
		    "	movia   r1, "  ASM_STR(SOCLIB_MC_MAGIC_VAL) "               \n"
		    "	stw     r1, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)             \n"
		    /* switch to associated memchecker context */
		    "	stw     %1, "  ASM_STR(SOCLIB_MC_CTX_SET) "(zero)           \n"
#endif

		    /* switch stack pointer */
		    "	stw     sp, 0(%0)                       \n"
		    "	ldw     r16, 0(%1)                      \n"

		    "	mov     sp, r16                         \n"

#ifdef CONFIG_SOCLIB_MEMCHECK
		    /* leave memchecker command mode */
		    "	stw     zero,  "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)          \n"
#endif

		    /* restore status & tls */
		    "   ldw     r1, 1*4(sp)                     \n"
		    "	ldw     r16, 0*4(sp)                    \n"
		    "	wrctl	status, r1                      \n"
		    /* restore frame pointer */
		    "	ldw     r28, 2*4(sp)                    \n"
		    /* Restore execution pointer */
		    "	ldw     r1, 3*4(sp)                     \n"
		    "	stw     r16, 0(%2)                      \n"
		    "	addi    sp, sp, 4*4                     \n"
		    "	jmp     r1                              \n"
		    "2:						\n"
		    : "=r" (unused1)
		    , "=r" (unused2)
		    , "=r" (unused3)

		    : "0" (&old->stack_ptr)
		    , "1" (&new->stack_ptr)
		    , "2" (CPU_LOCAL_ADDR(__context_data_base))

		      /* These registers will be saved by the compiler */
		    : "r2", "r3" /*"r4", "r5", "r6" */  ,"r7"/* leave for __asm__ input */
		    , "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"
		    , "r16", "r17", "r18", "r19", "r20", "r21", "r23"
		    , "r24", "r26", "r27"
#if !defined(CONFIG_COMPILE_FRAMEPTR) || defined(__OPTIMIZE__)
		    , "r28"
#endif
		    , "r31"
		    , "memory"
		    );
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  __asm__ volatile (
		    ".set noat                                	\n"
		    "	ldw r16, 0(%0)                          \n"

#ifdef CONFIG_SOCLIB_MEMCHECK
		    /* enter memchecker command mode */
		    "   movia   r1, "  ASM_STR(SOCLIB_MC_MAGIC_VAL) "     \n"
		    "   stw     r1, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)   \n"

		    /* mark current memchecker context as invalid */
		    "   movia   r1, "  ASM_STR(SOCLIB_MC_CTX_ID_CURRENT) "          \n"
		    "   stw     %1, "  ASM_STR(SOCLIB_MC_CTX_INVALIDATE) "(zero)    \n"

		    /* switch to associated memchecker context */
		    "   stw     %0, "  ASM_STR(SOCLIB_MC_CTX_SET) "(zero)  \n"
#endif

		    /* switch stack pointer */
		    "	mov     sp, r16                          \n"

#ifdef CONFIG_SOCLIB_MEMCHECK
		    "	stw	zero, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero) \n"
#endif

		    /* restore status & tls */
		    "	ldw     r1, 1*4(sp)                     \n"
		    "	ldw     r16, 0*4(sp)                    \n"
		    "	wrctl   status, r1                      \n"
		    /* restore frame pointer	 */
		    "	ldw     r28, 2*4(sp)                    \n"
		    /* Restore execution pointer */
		    "	ldw     r1, 3*4(sp)                     \n"
		    "	stw     r16, 0(%1)                      \n"
		    "	addi    sp, sp, 4*4                     \n"
		    "   jmp     r1                          	\n"
		    :
		    : "r" (&new->stack_ptr)
		    , "r" (CPU_LOCAL_ADDR(__context_data_base))
		    );
  while (1);
}

/*static inline */void
__attribute__((always_inline, noreturn))
cpu_context_set(uintptr_t stack, size_t stack_size, void *jumpto)
{
  __asm__ volatile (
		    ".set   noat                                \n"

#ifdef CONFIG_SOCLIB_MEMCHECK
		/* enter memchecker command mode */
		    "   movia   r1, "  ASM_STR(SOCLIB_MC_MAGIC_VAL) "               \n"
		    "   stw     r1, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)             \n"

		/* mark current memchecker context as invalid */
		    "   movia   r1, "  ASM_STR(SOCLIB_MC_CTX_ID_CURRENT) "          \n"
		    "   stw     %1, "  ASM_STR(SOCLIB_MC_CTX_INVALIDATE) "(zero)    \n"

		/* create a new temporary memchecker context using passed stack */
		    "	stw	%0,	" ASM_STR(SOCLIB_MC_R1) "(zero)             \n"
		    "	stw	%1,	" ASM_STR(SOCLIB_MC_R2) "(zero)             \n"
		    "	stw	%0,	" ASM_STR(SOCLIB_MC_CTX_CREATE_TMP) "(zero) \n"

		/* switch to new temporary memchecker context */
		    "   stw     %0, "  ASM_STR(SOCLIB_MC_CTX_SET) "(zero)           \n"
#endif
		    /* set stack pointer  */
		    "	add    %0, %0, %1                     	\n"
		    "	addi   sp, %0, -8                       \n"
		    "	jmp    %2                               \n"
		    :
		    : "r" (stack)
		    , "r" (stack_size)
		    , "r" (jumpto)
		    );
  while (1);
}

# if defined(CONFIG_HEXO_USERMODE)

/** kernel stack pointer value on user entry */
extern CONTEXT_LOCAL uintptr_t context_kstack;

void
__attribute__((noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack,
		     user_entry_t *entry, void *param);

# endif

#endif

