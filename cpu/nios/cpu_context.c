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


#include <stdlib.h>
#include <hexo/error.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>

error_t
cpu_context_bootstrap(struct context_s *context)
{
  /* set context local storage base pointer */
  CPU_LOCAL_SET(__context_data_base, context->tls);

  return 0;
}

/* fake context entry point, pop entry function param and entry function
   address from stack and perform jump to real entry function.  We
   need to do this to pass an argument to the context entry function. */
void __nios2_context_entry(void);

asm(
    ".set noat                                  \n"
	".type __nios2_context_entry, @function \n"
    "__nios2_context_entry:                     \n"
    "	ldw     r4, 0(sp)                       \n" /* entry function param */
    "	ldw     r1, 4(sp)                       \n" /* entry function address */
    "	addi    sp, sp, 2*4                     \n"
    "	jmp     r1                              \n"
    ".size __nios2_context_entry, .-__nios2_context_entry \n"
    );



/* context init function */

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{

  /* NiosII ABI requires 4 free words in the stack. */
  context->stack_ptr = (reg_t*)((uintptr_t)context->stack_end - CONFIG_HEXO_STACK_ALIGN);

  /* push entry function address and param arg */
  *--context->stack_ptr = (uintptr_t)entry;
  *--context->stack_ptr = (uintptr_t)param;

  /* fake entry point */
  *--context->stack_ptr = (uintptr_t)&__nios2_context_entry;

  /* frame pointer */
  *--context->stack_ptr = 0;

  /* status register, interrupts are disabled */
  *--context->stack_ptr = 0x0;

  /* context local storage address */
  *--context->stack_ptr = (uintptr_t)context->tls;

  return 0;
}



void
cpu_context_destroy(struct context_s *context)
{
#if 0
  reg_t	*stack = (reg_t*)context->stack_ptr;
#endif
}

#if defined(CONFIG_HEXO_USERMODE)
void __attribute__((noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack,
		     user_entry_t *entry, void *param)
{
  cpu_interrupt_disable();

  CONTEXT_LOCAL_SET(context_kstack, kstack);

  __asm__ volatile (
		    ".set noat                     \n"
	        ".set noreorder                \n"
		    /* set stack */
		    "   mov    sp,    %[ustack]    \n"
		    /* set arg */
		    "   mov    r4,    %[param]     \n"
		    "   addi   sp,    -4*4         \n"
		    "   mov    r16,   %[entry]     \n"
		    "   jmp    r16                 \n"
		    :
		    : [ustack]  "r" (ustack)
		      , [entry]   "r" (entry)
		      , [param]   "r" (param)
		    );
}
#endif
