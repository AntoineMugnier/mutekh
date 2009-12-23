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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2009

*/

#if CONFIG_SMP
# error No SMP supported
#endif

# define GET_HANDLER_ADDRESS(name, rd, rt, foo)						   \
	"ldr  " rt ", =" name "     \n\t"								   \
	"ldr  " rd ", [" rt "]      \n\t"

# define GET_CPULOCAL_HANDLER_ADDRESS(name, rd, rt) GET_HANDLER_ADDRESS(name, rd, rt, 3)
# define GET_CONTEXTLOCAL_HANDLER_ADDRESS(name, rd, rt) GET_HANDLER_ADDRESS(name, rd, rt, 4)


asm(
    ".section        .boot,\"ax\"	\n"

    ".globl cpu_boot				\n\t"
    ".func cpu_boot				    \n\t"
	".type   cpu_boot, %function    \n\t"
	"cpu_boot:                      \n\t"
	"ldr pc, =arm_boot                     \n\t"
	"ldr pc, =arm_c_exc_undef                \n\t"

/* 	"ldr pc, =arm_c_exc_swi                  \n\t" */
	"nop                                   \n\t"
/* 	"ldr pc, =arm_c_exc_pabt                 \n\t" */
	"nop                                   \n\t"
/* 	"ldr pc, =arm_c_exc_dabt                 \n\t" */
	"nop                                   \n\t"

	"nop                                   \n\t"
	"ldr pc, =arm_c_irq_handler           \n\t"
	"ldr pc, =arm_c_fiq_handler           \n\t"
	".size   cpu_boot, .-cpu_boot   \n\t"
	".endfunc \n\t"

    ".globl arm_boot				  \n\t"
    ".func  arm_boot				  \n\t"
	".type   arm_boot, %function      \n\t"
	"arm_boot:                        \n\t"
	/* set stack */
    "ldr  r13, =__initial_stack-16  \n\t"

    /* Get the device tree and put it in first arg */
#ifdef CONFIG_ARCH_DEVICE_TREE
    "ldr  r0, =dt_blob_start          \n\t"
#else
    "mov  r0, #0                      \n\t"
#endif
    /* Put a 0 in second arg */
    "mov  r1, #0                      \n\t"

	/* jump to arch_init */
    "ldr   r5, =arch_init             \n\t"
	"bx    r5                         \n\t"
	".size   arm_boot, .-arm_boot     \n\t"
	".endfunc \n\t"

	);
