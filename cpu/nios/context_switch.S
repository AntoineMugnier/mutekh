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
 *   Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2011
 */

#include <hexo/context.h>
#include <hexo/cpu.h>
#include <hexo/asm.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

/*
        Nios2 ABI

        * r1-r15 and r24-r31 are caller saved
        * r16-r21 are callee saved
        * r22-r23,r28 are either callee saved gp regs or special purpose

        * r0 is 0
        * r1 is asm temp
        * r2-r3 are ret value
        * r4-r7 are func args
        * r22 may be used as GOT ptr on GNU/Linux (caller saved)
        * r23 may be used as thread ptr on GNU/Linux (caller saved)
        * r24 is exception temp
        * r25 is break temporary
        * r26 is global pointer
        * r27 is stack pointer
        * r28 is frame pointer, or callee saved gp reg
        * r29 is exception return address
        * r30 is break return address (in reg bank0) or sstatus (in other banks)
        * r31 is return address

        Have a look to interrupt.S for interrupt context switch and preemption.
*/

FUNC_START(.text, cpu_context_switch)
        .set nobreak
        .set noat

        // get context local storage
        CPU_LOCAL ldw, __context_data_base, r1, r1
        // get context registers save array
        addi    r2,     r1,     nios_context_regs

        // save "callee saved" registers
        // save gp ptr, stack ptr, frame ptr, return address
        .irp    r, 16, 17, 18, 19, 20, 21, 22, 23, 27, 28, 31
        stw     r\r,            CPU_NIOS_CONTEXT_GPR(\r)(r2)
        .endr

        // resume directly at return address
        stw     r31,            CPU_NIOS_CONTEXT_PC(r2)

	rdctl   r8,             status
        stw     r8,             CPU_NIOS_CONTEXT_STATUS(r2)

        movui   r8,             CPU_NIOS_CONTEXT_RESTORE_CALLEE
        stw     r8,             CPU_NIOS_CONTEXT_SAVE_MASK(r2)

        br      cpu_context_jumpto
FUNC_END(cpu_context_switch)



        
FUNC_START(.text, cpu_context_jumpto)
        .set nobreak
        .set noat

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* enter memchecker command mode */
        movia   r1,             SOCLIB_MC_MAGIC_VAL
        stw     r1,             SOCLIB_MC_MAGIC(zero)
        
        /* switch to associated memchecker context */
        stw     r4,             SOCLIB_MC_CTX_SET(zero)
#endif

#ifdef CONFIG_ARCH_SMP
        // unlock some atomic value on context restore if requested
        ldw     r8,             HEXO_CONTEXT_S_UNLOCK * 4 (r4)
        beq     zero,   r8,     1f

        // unlock and clear unlock address
        stw     zero,           HEXO_CONTEXT_S_UNLOCK * 4 (r4)
        stw     zero,           (r8)
#endif

        // restore tls pointer from context struct
        ldw     r8,             HEXO_CONTEXT_S_TLS * 4 (r4)
        CPU_LOCAL stw,          __context_data_base, r8, r7

        addi    r2,     r8,     nios_context_regs

        // restore stack ptr, frame ptr, return address
        .irp    r, 27, 28, 31
        ldw      r\r,           CPU_NIOS_CONTEXT_GPR(\r)(r2)
        .endr

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* leave memchecker command mode */
        stw     zero,           SOCLIB_MC_MAGIC(zero)
#endif

        // prepare eret to retore status and pc
        ldw     r29,            CPU_NIOS_CONTEXT_PC(r2)
        ldw     r1,             CPU_NIOS_CONTEXT_STATUS(r2)
        wrctl   estatus,        r1

        ldw     r8,             CPU_NIOS_CONTEXT_SAVE_MASK(r2)

#if 0   // callee shall almost always get restored (except on context start)
        andi    r7,     r8,     CPU_NIOS_CONTEXT_RESTORE_CALLEE
        beq     r7,     zero,   1f
# endif
        
        // restore "callee saved" registers
        .irp    r, 16, 17, 18, 19, 20, 21, 22, 23
        ldw     r\r,            CPU_NIOS_CONTEXT_GPR(\r)(r2)
        .endr
1:      
        andi    r7,     r8,     CPU_NIOS_CONTEXT_RESTORE_CALLER
        beq     r7,     zero,   1f

        // restore "caller saved" register
        .irp    r, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 2
        ldw     r\r,            CPU_NIOS_CONTEXT_GPR(\r)(r2)
        .endr
1:

        // return and restore status
        eret
FUNC_END(cpu_context_jumpto)

FUNC_START(.text, cpu_context_set)
        .set nobreak
        .set noat
        
#ifdef CONFIG_SOCLIB_MEMCHECK
        /* enter memchecker command mode */
        movia   r1,             SOCLIB_MC_MAGIC_VAL
        stw     r1,             SOCLIB_MC_MAGIC(zero)
        
        /* switch to associated memchecker context */
        stw     r4,             SOCLIB_MC_CTX_SET(zero)

        /* mark current memchecker context as invalid */
        movi    r1,             SOCLIB_MC_CTX_ID_CURRENT
        stw     r1,             SOCLIB_MC_CTX_INVALIDATE(zero)

        /* create a new temporary memchecker context using passed stack */
        stw     r4,             SOCLIB_MC_R1(zero)
        stw     r5,             SOCLIB_MC_R2(zero)
        stw     r4,             SOCLIB_MC_CTX_CREATE_TMP(zero)

        /* switch to new temporary memchecker context */
        stw     r4,             SOCLIB_MC_CTX_SET(zero)
#endif

        /* Set stack pointer */
        add     r4,     r4,     r5
        addi    sp,     r4,     -CONFIG_HEXO_STACK_ALIGN
        
#ifdef CONFIG_SOCLIB_MEMCHECK
        /* leave memchecker command mode */
        stw     zero,           SOCLIB_MC_MAGIC(zero)
#endif

        /* Jump */
        jmp     r6
        
FUNC_END(cpu_context_set)

FUNC_START(.text, cpu_context_stack_use)
        .set nobreak
        .set noat

#ifdef CONFIG_SOCLIB_MEMCHECK
        /* enter memchecker command mode */
        movia   r1,             SOCLIB_MC_MAGIC_VAL
        stw     r1,             SOCLIB_MC_MAGIC(zero)
        
        /* switch to associated memchecker context */
        stw     r4,             SOCLIB_MC_CTX_SET(zero)
#endif
        
        // get tls pointer from context struct
        ldw     r8,             HEXO_CONTEXT_S_TLS * 4 (r4)
        // get context registers save array
        addi    r2,     r8,     nios_context_regs

        ldw     sp,             CPU_NIOS_CONTEXT_GPR(27)(r2)
        
#ifdef CONFIG_SOCLIB_MEMCHECK
        /* leave memchecker command mode */
        stw     zero,           SOCLIB_MC_MAGIC(zero)
#endif
        
#ifndef CONFIG_RELEASE
        // prevent use of previous context local storage
        CPU_LOCAL stw,   __context_data_base, zero, r1
#endif

        // private param
        mov     r4,             r6

        // call function
        jmp     r5
FUNC_END(cpu_context_stack_use)

#if 0
        static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  void	*unused1, *unused2, *unused3;

  __asm__ volatile (
		    ".set   noat                                \n"
 ifdef CONFIG_COMPILE_PIC_
		    /* save execution pointer based on current PC */
		    "	addi    sp, sp, -4*4                    \n"
		    "	stw     ra, 3*4(sp)                     \n"
		    "	nextpc  r31                             \n"
		    "	jmpi    1f                              \n"
		    "	jmpi    2f                              \n"
		    "1:	nop                                     \n"
 else
		    /* save execution pointer based on static label address */
		    "	addi    sp, sp, -4*4                    \n"
		    "	movia   r1, 2f                          \n"
		    "	stw     r1, 3*4(sp)                     \n"
 endif
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

 ifdef CONFIG_SOCLIB_MEMCHECK
		    /* enter memchecker command mode */
		    "	movia   r1, "  ASM_STR(SOCLIB_MC_MAGIC_VAL) "               \n"
		    "	stw     r1, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)             \n"
		    /* switch to associated memchecker context */
		    "	stw     %1, "  ASM_STR(SOCLIB_MC_CTX_SET) "(zero)           \n"
 endif

		    /* switch stack pointer */
		    "	stw     sp, 0(%0)                       \n"
		    "	ldw     r16, 0(%1)                      \n"

		    "	mov     sp, r16                         \n"

 ifdef CONFIG_SOCLIB_MEMCHECK
		    /* leave memchecker command mode */
		    "	stw     zero,  "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)          \n"
 endif

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
 if !defined(CONFIG_COMPILE_FRAMEPTR) || defined(__OPTIMIZE__)
		    , "r28"
 endif
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

 ifdef CONFIG_SOCLIB_MEMCHECK
		    /* enter memchecker command mode */
		    "   movia   r1, "  ASM_STR(SOCLIB_MC_MAGIC_VAL) "     \n"
		    "   stw     r1, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero)   \n"

		    /* mark current memchecker context as invalid */
		    "   movia   r1, "  ASM_STR(SOCLIB_MC_CTX_ID_CURRENT) "          \n"
		    "   stw     %1, "  ASM_STR(SOCLIB_MC_CTX_INVALIDATE) "(zero)    \n"

		    /* switch to associated memchecker context */
		    "   stw     %0, "  ASM_STR(SOCLIB_MC_CTX_SET) "(zero)  \n"
 endif

		    /* switch stack pointer */
		    "	mov     sp, r16                          \n"

 ifdef CONFIG_SOCLIB_MEMCHECK
		    "	stw	zero, "  ASM_STR(SOCLIB_MC_MAGIC) "(zero) \n"
 endif

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

 ifdef CONFIG_SOCLIB_MEMCHECK
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
 endif
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

/** kernel stack pointer value on user entry */
extern CONTEXT_LOCAL uintptr_t context_kstack;

void
__attribute__((noreturn))
cpu_context_set_user(uintptr_t kstack, uintptr_t ustack,
		     user_entry_t *entry, void *param);

         if defined(CONFIG_HEXO_USERMODE)
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

