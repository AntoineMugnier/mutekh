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
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009

*/

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#ifdef CONFIG_SMP
# define GET_HANDLER_ADDRESS(name, rd, rt, op2)						   \
	"ldr  " rd ", =" name "             \n\t"						   \
	"mrc p15,0," rt ",c13,c0," #op2 "   \n\t"						   \
	"ldr  " rd ", [" rd ", +" rt "]     \n\t"
#else
# define GET_HANDLER_ADDRESS(name, rd, rt, foo)						   \
	"ldr  " rd ", =" name "     \n\t"								   \
	"ldr  " rd ", [" rd "]      \n\t"
#endif

# define GET_CPULOCAL_HANDLER_ADDRESS(name, rd, rt) GET_HANDLER_ADDRESS(name, rd, rt, 3)
# define GET_CONTEXTLOCAL_HANDLER_ADDRESS(name, rd, rt) GET_HANDLER_ADDRESS(name, rd, rt, 4)


asm(
    ".section        .boot,\"ax\"			\n"

    ".globl cpu_boot				\n\t"
	".func cpu_boot				    \n\t"
	".type   cpu_boot, %function \n\t"
	"cpu_boot:                   \n\t"
	"b arm_boot                     \n\t"
	"b arm_exc_undef                \n\t"
	"b arm_exc_swi                  \n\t"
	"b arm_exc_pabt                 \n\t"
	"b arm_exc_dabt                 \n\t"
	"nop                            \n\t"
	"b arm_exc_irq                  \n\t"
	// We dont like FIQ. Therefore, we'll return straight.
	"subs pc, r14, #4               \n\t"
	".size   cpu_boot, .-cpu_boot\n\t"
	".endfunc \n\t"

    ".globl arm_boot				\n\t"
	".func arm_boot				    \n\t"
	".type   arm_boot, %function   \n\t"
	"arm_boot:                     \n\t"
	"mrc  p15,0,r4,c0,c0,5       \n\t"
# ifndef CONFIG_SMP
    "cmp	r4,	#0   			\n\t"
    "1: bne	1b			        \n\t"
# endif

	// Allocate 1K stacks
	"lsl  r5, r4, #10            \n\t"

    "ldr  r13, =__initial_stack-16   \n\t"
	"subs r13, r13, r5                         \n\t"

#ifdef CONFIG_SOCLIB_MEMCHECK
    "mov  r0, #1024           \n\t"
    "ldr  r2, =" ASM_STR(SOCLIB_MC_MAGIC_VAL) "  \n\t"
    "ldr  r1, =" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) " \n\t"
    "str  r2, [r1, #(" ASM_STR(SOCLIB_MC_MAGIC) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
    "str  r0, [r1, #(" ASM_STR(SOCLIB_MC_R2) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
	"sub  r0, sp, r0   \n\t"
	"add  r0, r0, #4   \n\t"
    "str  r0, [r1, #(" ASM_STR(SOCLIB_MC_R1) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
    "str  r4, [r1, #(" ASM_STR(SOCLIB_MC_CTX_CREATE) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
    "str  r4, [r1, #(" ASM_STR(SOCLIB_MC_CTX_SET) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"

    "mov  r0, #" ASM_STR(SOCLIB_MC_CHECK_SPFP+SOCLIB_MC_CHECK_INIT) " \n\t"
    "str  r0, [r1, #(" ASM_STR(SOCLIB_MC_ENABLE) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"

    "mov  r0, #0             \n\t"
    "str  r0, [r1, #(" ASM_STR(SOCLIB_MC_MAGIC) "-" ASM_STR(CONFIG_SOCLIB_MEMCHECK_ADDRESS) ")] \n\t"
#endif

    /* Get the device tree and put it in first arg */
#ifdef CONFIG_ARCH_DEVICE_TREE
    "ldr  r0, =dt_blob_start          \n\t"
#else
    "mov  r0, #0                      \n\t"
#endif
    /* Put a 0 in second arg */
    "mov  r1, #0                      \n\t"

	"ldr  r12, =arch_init             \n\t"
	"bx   r12                         \n\t"
	".size   arm_boot, .-arm_boot     \n\t"
	".endfunc \n\t"

	".globl arm_hw_interrupt_proxy    \n\t"
	".type   arm_hw_interrupt_proxy, %function   \n\t"
	"arm_hw_interrupt_proxy:          \n\t"
	"push {r0, r1, r2, r3, r12, lr}   \n\t"
	"mov  r0, #0                      \n\t"
	GET_CPULOCAL_HANDLER_ADDRESS("cpu_interrupt_handler", "r12", "r1")
	"mov  lr, pc                      \n\t"
	"bx   r12                         \n\t"
	"pop  {r0, r1, r2, r3, r12, pc}   \n\t"
	".size   arm_hw_interrupt_proxy, .-arm_hw_interrupt_proxy     \n\t"

	/* Exception handler, from switching code below, desc struct is in r0 (to restore) */
	".globl arm_ex_interrupt_proxy    \n\t"
	".type   arm_ex_interrupt_proxy, %function   \n\t"
	"arm_ex_interrupt_proxy:          \n\t"
	"push  {r1, r2, r3, r4, r12, lr}  \n\t"
	/* Leave room for r15 */
	"sub   sp, sp, #12                 \n\t"
	/* Get user's sp & lr and put them in table */
	"stmia r1, {r13, r14}^             \n\t"
	/* All those are directly from user. */
	"push  {r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12} \n\t"
	/* Get back user's r0 and put it in table */
	"ldr   r1, [r0, #8]                \n\t"
	"push  {r1}                        \n\t"
	/* Regtable is in *sp */
	"mov   a4, sp                      \n\t"
	/* Get user's r13 as stack pointer */
	"ldr   a1, [r1, #4*13]             \n\t"
	/* Push it as 5th argument */
	"push  {a1}                        \n\t"
	/* data pointer */
	"mrc p15, 0, a3, c6, c0, 0         \n\t"
	/* TODO: PC */
	"mrc p15, 0, a2, c6, c0, 2         \n\t"
	/* TODO: type */
	"mov   a1, #0                      \n\t"
	GET_CPULOCAL_HANDLER_ADDRESS("cpu_interrupt_handler", "r12", "r1")
	"mov  lr, pc                      \n\t"
	"bx   r12                         \n\t"
	/* 1 level for r5 and 15 levels for reg table (not 16) */
	"add  sp, #16*4                   \n\t"
	"pop  {r1, r2, r3, r4, r12, pc}   \n\t"
	".size   arm_ex_interrupt_proxy, .-arm_ex_interrupt_proxy     \n\t"


	/* Syscall handler, directly in Super32 */
	".globl arm_exc_swi                    \n\t"
	".type   arm_exc_swi, %function        \n\t"
	"arm_exc_swi:                          \n\t"
	"push {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, lr}  \n\t"
	"ldr  r0, [lr], #-4                \n\t"
	"and  r0, #0x00ffffff              \n\t"
	"mov  r1, sp                       \n\t"
	GET_CONTEXTLOCAL_HANDLER_ADDRESS("cpu_syscall_handler", "r12", "r1")
	"mov  lr, pc                      \n\t"
	"bx   r12                         \n\t"
	"pop  {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, lr}  \n\t"
	"movs pc, r14                     \n\t"
	".size   arm_exc_swi, .-arm_exc_swi     \n\t"

	/* Other handlers, in other modes to Super32 */
#define IMPLEMENT_EXC(exc_name, handler, ret_subs_offset, mode)		   \
    ".globl arm_" exc_name "           \n\t"						   \
	".type   arm_" exc_name ", %function  \n\t"						   \
    "arm_" exc_name ":                 \n\t"						   \
	/* save user's r0 */											   \
    "str    r0, [sp, #8]               \n\t"						   \
	/* save user's pc */											   \
    "str    lr, [sp]                   \n\t"						   \
	/* save user's cpsr from current spsr */						   \
	"mrs    r0, spsr                   \n\t"						   \
    "str    r0, [sp, #4]               \n\t"						   \
	/* pass struct pointer to super32 */							   \
    "mov    r0, sp                     \n\t"						   \
	/* switch to super32 */											   \
	"mrs    lr, cpsr                   \n\t"						   \
	"bic    lr, lr, #0x1f              \n\t"						   \
	"orr    lr, lr, #0x13              \n\t"						   \
	"msr    cpsr, lr                   \n\t"						   \
	"push   {lr}                       \n\t"						   \
    handler " \n\t"													   \
	"pop    {lr}                       \n\t"						   \
	/* switch back to mode we are from */							   \
	"mrs    r0, cpsr                   \n\t"						   \
	"bic    r0, r0, #0x1f              \n\t"						   \
	"orr    r0, r0, #" #mode "         \n\t"						   \
	"msr    cpsr, r0                   \n\t"						   \
	/* restore user's cpsr to current spsr */						   \
    "ldr    r0, [sp, #4]               \n\t"						   \
	"msr    spsr, r0                   \n\t"						   \
	/* restore user's pc */											   \
    "ldr    lr, [sp]                   \n\t"						   \
	/* restore r0 */												   \
    "ldr    r0, [sp, #8]               \n\t"						   \
	"subs   pc, lr, #" #ret_subs_offset "\n\t"						   \
	".size   arm_" exc_name ", .-arm_" exc_name "     \n\t"

	IMPLEMENT_EXC("exc_undef", "bl arm_ex_interrupt_proxy", 0, 0x1b)
	IMPLEMENT_EXC("exc_pabt", "bl arm_ex_interrupt_proxy", 4, 0x17)
	IMPLEMENT_EXC("exc_dabt", "bl arm_ex_interrupt_proxy", 8, 0x17)
	IMPLEMENT_EXC(
		"exc_irq",
		"push {r1, r2, r3, r12}   \n\t"
		GET_CPULOCAL_HANDLER_ADDRESS("cpu_interrupt_handler", "r1", "r0")
		"mov  r0, #0                      \n\t"
		"mov  lr, pc                      \n\t"
		"bx   r1                         \n\t"
		"pop  {r1, r2, r3, r12}   \n\t",
		4, 0x12)

	);
