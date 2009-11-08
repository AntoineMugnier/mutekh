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

/*
	".section      \".ppc_special." #name "\",\"ax\",@progbits			\n\t"
    ".globl ppc_special_" #name "_entry				\n\t"
	".type  ppc_special_" #name "_entry, @function   \n"
    "ppc_special_" #name "_entry:					\n\t"
    "stwu 1, -16(1)					\n\t"
    "stw 0, 8(1)					\n\t"
    "mflr 0					\n\t"
    "stw 0, 12(1)					\n\t"
	"bl " #func "\n\t"
    "lwz 0, 12(1)					\n\t"
    "mtlr 0					\n\t"
    "lwz 0, 8(1)					\n\t"
    "addi 1, 1, 16					\n\t"
	#ret_ins "\n\t"
	".size ppc_special_" #name "_entry, .-ppc_special_" #name "_entry \n\t"
*/

#define SAVE_REG_2_12(n, off)									\
    "stw   2, " #off "+4* 0(" #n ")      \n\t"					\
    "stw   3, " #off "+4* 1(" #n ")      \n\t"					\
    "stw   4, " #off "+4* 2(" #n ")      \n\t"					\
    "stw   5, " #off "+4* 3(" #n ")      \n\t"					\
    "stw   6, " #off "+4* 4(" #n ")      \n\t"					\
    "stw   7, " #off "+4* 5(" #n ")      \n\t"					\
    "stw   8, " #off "+4* 6(" #n ")      \n\t"					\
    "stw   9, " #off "+4* 7(" #n ")      \n\t"					\
    "stw  10, " #off "+4* 8(" #n ")      \n\t"					\
    "stw  11, " #off "+4* 9(" #n ")      \n\t"					\
    "stw  12, " #off "+4*10(" #n ")      \n\t"

#define RESTORE_REG_2_12(n, off)								\
    "lwz   2, " #off "+4* 0(" #n ")      \n\t"					\
    "lwz   3, " #off "+4* 1(" #n ")      \n\t"					\
    "lwz   4, " #off "+4* 2(" #n ")      \n\t"					\
    "lwz   5, " #off "+4* 3(" #n ")      \n\t"					\
    "lwz   6, " #off "+4* 4(" #n ")      \n\t"					\
    "lwz   7, " #off "+4* 5(" #n ")      \n\t"					\
    "lwz   8, " #off "+4* 6(" #n ")      \n\t"					\
    "lwz   9, " #off "+4* 7(" #n ")      \n\t"					\
    "lwz  10, " #off "+4* 8(" #n ")      \n\t"					\
    "lwz  11, " #off "+4* 9(" #n ")      \n\t"					\
    "lwz  12, " #off "+4*10(" #n ")      \n\t"

#ifdef CONFIG_SMP
# define GET_HANDLER_ADDRESS(name, rd, rt, spr)						   \
	"mfspr " #rt ", " #spr "                \n\t"					   \
    "lwz   " #rd ", " name "(" #rt ")    \n\t"
#else
# define GET_HANDLER_ADDRESS(name, rd, rt, foo)						   \
    "lis   " #rt ", " name "@ha           \n\t"						   \
    "lwz   " #rd ", " name "@l(" #rt ")     \n\t"
#endif

# define GET_CPULOCAL_HANDLER_ADDRESS(name, rd, rt) GET_HANDLER_ADDRESS(name, rd, rt, 0x105)
# define GET_CONTEXTLOCAL_HANDLER_ADDRESS(name, rd, rt) GET_HANDLER_ADDRESS(name, rd, rt, 0x104)

#define HANDLE_EXCEPTION(eno, srr02)								   \
	/* r1 -> @0 */													   \
    "stwu  1,-4*22(1)                      \n\t"					   \
	/* r0 -> @3 */													   \
    "stw   0, 4* 7(1)                      \n\t"					   \
	/* lr -> @19 */													   \
    "mflr  0                               \n\t"					   \
    "stw   0, 4*19(1)                      \n\t"					   \
	/* cr -> @21 */													   \
    "mfcr  0                               \n\t"					   \
    "stw   0, 4*21(1)                      \n\t"					   \
	/* r2-r12 -> @8-18 */											   \
	SAVE_REG_2_12(1, 32)											   \
																	   \
	/* ctr -> @20 */												   \
	"mfctr 0                               \n\t"					   \
    "stw   0, 4*20(1)                      \n\t"					   \
																	   \
	GET_CPULOCAL_HANDLER_ADDRESS("cpu_exception_handler", 0, 3)		   \
	"mtctr 0                               \n\t"					   \
																	   \
	/* Put handler arguments: */									   \
	/* a0: type */													   \
	/* a1: execptr (srr) */											   \
	/* a2: dataptr (dear) */										   \
	/* a3: regtable (sp + 7*4) */									   \
	/* a4: sp (sp + 22*4) */										   \
	"li    3, " #eno "                     \n\t"					   \
	"mfsrr" #srr02 " 4                     \n\t"					   \
	"mfspr  5, 981                         \n\t"					   \
	"addi   6, 1, 4* 7                     \n\t"					   \
	"addi   7, 1, 4*22                     \n\t"					   \
																	   \
	"bctrl                                 \n\t"					   \
																	   \
	/* ctr -> @20 */												   \
    "lwz   0, 4*20(1)                      \n\t"					   \
	"mtctr 0                               \n\t"					   \
																	   \
	/* r2-r12 -> @8-18 */											   \
	RESTORE_REG_2_12(1, 32)											   \
																	   \
	/* lr -> @19 */													   \
    "lwz   0, 4*19(1)                      \n\t"					   \
    "mtlr  0                               \n\t"					   \
																	   \
	/* cr -> @21 */													   \
    "lwz   0, 4*21(1)                      \n\t"					   \
    "mtcr  0                               \n\t"					   \
																	   \
	/* r0 -> @7 */													   \
    "lwz  0, 4* 7(1)                         \n\t"					   \
	/* r1 -> @0 */													   \
    "addi 1, 1, 4*18                      \n\t"

asm(
    ".section        .excep,\"ax\",@progbits		\n"

	/* critical interrupt (critical, async) */
	".org 0x100\n"

	/* machine check (critical, async, imprecise) */
	".org 0x200\n"
    ".globl ppc_special_machine_check_entry		      \n\t"
	".type  ppc_special_machine_check_entry, @function  \n"
    "ppc_special_machine_check_entry:		      \n\t"
	HANDLE_EXCEPTION(3, 2)
	"rfci                                  \n\t"
	".size ppc_special_machine_check_entry, .-ppc_special_machine_check_entry \n\t"

	/* data storage */
	".org 0x300\n"
    ".globl ppc_special_data_storage_entry		      \n\t"
	".type  ppc_special_data_storage_entry, @function  \n"
    "ppc_special_data_storage_entry:		      \n\t"
	HANDLE_EXCEPTION(4, 0)
	"rfi                                  \n\t"
	".size ppc_special_data_storage_entry, .-ppc_special_data_storage_entry \n\t"

	/* instr storage */
	".org 0x400\n"
    ".globl ppc_special_instr_storage_entry		      \n\t"
	".type  ppc_special_instr_storage_entry, @function  \n"
    "ppc_special_instr_storage_entry:		      \n\t"
	HANDLE_EXCEPTION(5, 0)
	"rfi                                  \n\t"
	".size ppc_special_instr_storage_entry, .-ppc_special_instr_storage_entry \n\t"

	/* external (async) */
	".org 0x500\n"
    ".globl ppc_special_external_entry		      \n\t"
	".type  ppc_special_external_entry, @function  \n"
    "ppc_special_external_entry:           \n\t"
	/* r1 -> @0 */
    "stwu  1,-4*18(1)                      \n\t"
	/* r0 -> @3 */
    "stw   0, 4* 3(1)                      \n\t"
	/* lr -> @15 */
    "mflr  0                               \n\t"
    "stw   0, 4*15(1)                      \n\t"
	/* cr -> @17 */
    "mfcr  0                               \n\t"
    "stw   0, 4*17(1)                      \n\t"
	/* r2-r12 -> @4-14 */
	SAVE_REG_2_12(1, 16)

	/* ctr -> @16 */
	"mfctr 0                               \n\t"
    "stw   0, 4*16(1)                      \n\t"

	GET_CPULOCAL_HANDLER_ADDRESS("cpu_interrupt_handler", 0, 3)
	"mtctr 0                               \n\t"

	GET_CPULOCAL_HANDLER_ADDRESS("cpu_interrupt_handler_arg", 3, 3)

	/* interrupt line is 0 */
	"li    4, 0                            \n\t"

	"bctrl                                 \n\t"

	/* ctr -> @16 */
    "lwz   0, 4*16(1)                      \n\t"
	"mtctr 0                               \n\t"

	/* r2-r12 -> @4-14 */
	RESTORE_REG_2_12(1, 16)

	/* lr -> @15 */
    "lwz   0, 4*15(1)                      \n\t"
    "mtlr  0                               \n\t"

	/* cr -> @17 */
    "lwz   0, 4*17(1)                      \n\t"
    "mtcr  0                               \n\t"

	/* r0 -> @3 */
    "lwz  0, 4* 3(1)                         \n\t"
	/* r1 -> @0 */
    "addi 1, 1, 4*18                      \n\t"

	"rfi                                  \n\t"
	".size ppc_special_external_entry, .-ppc_special_external_entry \n\t"


	/* alignment */
	".org 0x600\n"
    ".globl ppc_special_alignment_entry		      \n\t"
	".type  ppc_special_alignment_entry, @function  \n"
    "ppc_special_alignment_entry:		      \n\t"
	HANDLE_EXCEPTION(2, 0)
	"rfi                                  \n\t"
	".size ppc_special_alignment_entry, .-ppc_special_alignment_entry \n\t"

	/* program */
	".org 0x700\n"
    ".globl ppc_special_program_entry		      \n\t"
	".type  ppc_special_program_entry, @function  \n"
    "ppc_special_program_entry:		      \n\t"
	HANDLE_EXCEPTION(1, 0)
	"rfi                                  \n\t"
	".size ppc_special_program_entry, .-ppc_special_program_entry \n\t"

	/* fpu unusable */
	".org 0x800\n"

	/* syscall */
	".org 0xc00\n"
    ".globl ppc_special_syscall_entry		      \n\t"
	".type  ppc_special_syscall_entry, @function  \n"
    "ppc_special_syscall_entry:           \n\t"
    "stwu 1, -4*15(1)                     \n\t"
    "stw   0, 4* 1(1)                      \n\t"

	SAVE_REG_2_12(1, 16)

    /* Put syscall number on first arg */
	"or    3, 0, 0                         \n\t"
    "mflr  0                               \n\t"
    "stw   0, 4*13(1)                      \n\t"

	"mfctr 0                               \n\t"
    "stw   0, 4*14(1)                      \n\t"

	GET_CONTEXTLOCAL_HANDLER_ADDRESS("cpu_syscall_handler", 0, 3)
	"mtctr 0                               \n\t"
	"bctrl                                 \n\t"

    /* Put register table on second arg */
	/* Table is: sp, r0, r2, r3, ..., r12 */
	"or    4, 1, 1                         \n\t"
    "lwz   0, 4*14(1)                      \n\t"
	"mtctr 0                               \n\t"

    "lwz   0, 4*13(1)                      \n\t"
	RESTORE_REG_2_12(1, 16)

    "mtlr 0                               \n\t"
    "lwz  0, 4(1)                         \n\t"
    "lwz  1, 0(1)                         \n\t"

	"rfi                                  \n\t"
	".size ppc_special_syscall_entry, .-ppc_special_syscall_entry \n\t"

	/* apu unavailable */
	".org 0xf20\n"

	/* programmable-interval timer (async) */
	".org 0x1000\n"

	/* fixed-interval timer (async) */
	".org 0x1010\n"

	/* watchdog (critical, async) */
	".org 0x1020\n"

	/* data tlb */
	".org 0x1100\n"

	/* ins tlb */
	".org 0x1200\n"

	/* debug (critical, (a)sync) */
	".org 0x2000\n"

    );

