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



/**
 * 1st level interrupt handler (entry point).
 *
 * We have as many cpu handlers as irq on x86, so we must define as
 * many entry point as supported interrupts. Each entry point code
 * must pass its irq number to the called C function.
 *
 * Here we push all registers on stack, pass argument, do
 * function call, pop registers and do interrupt return.
 *
 * Function call is done via cpu-local function pointer.
 */

#define ENTRY_ALIGN	16

#ifdef CONFIG_SMP
# define CLS_SEG	%fs:
#else
# define CLS_SEG
#endif

/** interruption entry point template */
#define PUT_ENTRY(n, lbl)			\
.align ENTRY_ALIGN				; \
	pusha					; \
	mov	$0x##n, %edi			; \
	jmp	lbl##f

/** syscakk callback */
#define PUT_TAIL_SYS(lbl)			\

/** exception callback */
#define PUT_TAIL_EX(callback, lbl)		\

/** exception with error code callback */
#define PUT_TAIL_EX_ERR(callback, lbl)		\

.text

.globl x86_interrupt_ex_entry
.align ENTRY_ALIGN
x86_interrupt_ex_entry:
PUT_ENTRY(00, 1)
PUT_ENTRY(01, 1)
PUT_ENTRY(02, 1)
PUT_ENTRY(03, 1)
PUT_ENTRY(04, 1)
PUT_ENTRY(05, 1)
PUT_ENTRY(06, 1)
PUT_ENTRY(07, 1)
PUT_ENTRY(08, 2)
PUT_ENTRY(09, 1)
PUT_ENTRY(0a, 2)
PUT_ENTRY(0b, 2)
PUT_ENTRY(0c, 2)
PUT_ENTRY(0d, 2)
PUT_ENTRY(0e, 2)
PUT_ENTRY(0f, 1)
PUT_ENTRY(10, 1)
PUT_ENTRY(11, 2)
PUT_ENTRY(12, 1)
PUT_ENTRY(13, 1)
PUT_ENTRY(14, 1)
PUT_ENTRY(15, 1)
PUT_ENTRY(16, 1)
PUT_ENTRY(17, 1)
PUT_ENTRY(18, 1)
PUT_ENTRY(19, 1)
PUT_ENTRY(1a, 1)
PUT_ENTRY(1b, 1)
PUT_ENTRY(1c, 1)
PUT_ENTRY(1d, 1)
PUT_ENTRY(1e, 1)
PUT_ENTRY(1f, 1)

1:
#ifdef CONFIG_CPU_USER
	call	x86_seg_restore
#endif
	lea	44(%esp), %ebx
	pushl	%ebx				; /* stackptr */
	lea	4(%esp), %ebx
	pushl	%ebx				; /* regtable */
	movl	%cr2, %ebx
	pushl	%ebx				; /* dataptr */
	pushl	44(%esp)			; /* execptr */
	pushl	%edi				; /* type */
	call	CLS_SEG *(cpu_exception_handler)
	add	$20, %esp
	jmp	x86_interrupt_end

2:
#ifdef CONFIG_CPU_USER
	call	x86_seg_restore
#endif
	lea	48(%esp), %ebx
	pushl	%ebx				; /* stackptr */
	lea	4(%esp), %ebx
	pushl	%ebx				; /* regtable */
	mov	%cr2, %ebx
	pushl	%ebx				; /* dataptr */
	pushl	48(%esp)			; /* execptr */
	pushl	%edi				; /* type */
	call	CLS_SEG *(cpu_exception_handler)
	add	$24, %esp
	jmp	x86_interrupt_end

.globl x86_interrupt_hw_entry
.align ENTRY_ALIGN
x86_interrupt_hw_entry:
PUT_ENTRY(00, 1)
PUT_ENTRY(01, 1)
PUT_ENTRY(02, 1)
PUT_ENTRY(03, 1)
PUT_ENTRY(04, 1)
PUT_ENTRY(05, 1)
PUT_ENTRY(06, 1)
PUT_ENTRY(07, 1)
PUT_ENTRY(08, 1)
PUT_ENTRY(09, 1)
PUT_ENTRY(0a, 1)
PUT_ENTRY(0b, 1)
PUT_ENTRY(0c, 1)
PUT_ENTRY(0d, 1)
PUT_ENTRY(0e, 1)
PUT_ENTRY(0f, 1)
PUT_ENTRY(10, 1)
PUT_ENTRY(11, 1)
PUT_ENTRY(12, 1)
PUT_ENTRY(13, 1)
PUT_ENTRY(14, 1)
PUT_ENTRY(15, 1)
PUT_ENTRY(16, 1)
PUT_ENTRY(17, 1)
PUT_ENTRY(18, 1)
PUT_ENTRY(19, 1)
PUT_ENTRY(1a, 1)
PUT_ENTRY(1b, 1)
PUT_ENTRY(1c, 1)
PUT_ENTRY(1d, 1)
PUT_ENTRY(1e, 1)
PUT_ENTRY(1f, 1)
PUT_ENTRY(20, 1)
PUT_ENTRY(21, 1)
PUT_ENTRY(22, 1)
PUT_ENTRY(23, 1)
PUT_ENTRY(24, 1)
PUT_ENTRY(25, 1)
PUT_ENTRY(26, 1)
PUT_ENTRY(27, 1)
PUT_ENTRY(28, 1)
PUT_ENTRY(29, 1)
PUT_ENTRY(2a, 1)
PUT_ENTRY(2b, 1)
PUT_ENTRY(2c, 1)
PUT_ENTRY(2d, 1)
PUT_ENTRY(2e, 1)
PUT_ENTRY(2f, 1)
PUT_ENTRY(30, 1)
PUT_ENTRY(31, 1)
PUT_ENTRY(32, 1)
PUT_ENTRY(33, 1)
PUT_ENTRY(34, 1)
PUT_ENTRY(35, 1)
PUT_ENTRY(36, 1)
PUT_ENTRY(37, 1)
PUT_ENTRY(38, 1)
PUT_ENTRY(39, 1)
PUT_ENTRY(3a, 1)
PUT_ENTRY(3b, 1)
PUT_ENTRY(3c, 1)
PUT_ENTRY(3d, 1)
PUT_ENTRY(3e, 1)
PUT_ENTRY(3f, 1)
PUT_ENTRY(40, 1)
PUT_ENTRY(41, 1)
PUT_ENTRY(42, 1)
PUT_ENTRY(43, 1)
PUT_ENTRY(44, 1)
PUT_ENTRY(45, 1)
PUT_ENTRY(46, 1)
PUT_ENTRY(47, 1)
PUT_ENTRY(48, 1)
PUT_ENTRY(49, 1)
PUT_ENTRY(4a, 1)
PUT_ENTRY(4b, 1)
PUT_ENTRY(4c, 1)
PUT_ENTRY(4d, 1)
PUT_ENTRY(4e, 1)
PUT_ENTRY(4f, 1)
PUT_ENTRY(50, 1)
PUT_ENTRY(51, 1)
PUT_ENTRY(52, 1)
PUT_ENTRY(53, 1)
PUT_ENTRY(54, 1)
PUT_ENTRY(55, 1)
PUT_ENTRY(56, 1)
PUT_ENTRY(57, 1)
PUT_ENTRY(58, 1)
PUT_ENTRY(59, 1)
PUT_ENTRY(5a, 1)
PUT_ENTRY(5b, 1)
PUT_ENTRY(5c, 1)
PUT_ENTRY(5d, 1)
PUT_ENTRY(5e, 1)
PUT_ENTRY(5f, 1)

1:
#ifdef CONFIG_CPU_USER
	call	x86_seg_restore
#endif
	pushl	%edi
	movl    CLS_SEG (cpu_interrupt_handler_arg), %edi
	pushl	%edi
	call	CLS_SEG *(cpu_interrupt_handler)
	add	$8, %esp
	jmp	x86_interrupt_end

.globl x86_interrupt_sys_entry
.align ENTRY_ALIGN
x86_interrupt_sys_entry:
PUT_ENTRY(00, 1)
PUT_ENTRY(01, 1)
PUT_ENTRY(02, 1)
PUT_ENTRY(03, 1)
PUT_ENTRY(04, 1)
PUT_ENTRY(05, 1)
PUT_ENTRY(06, 1)
PUT_ENTRY(07, 1)
PUT_ENTRY(08, 1)
PUT_ENTRY(09, 1)
PUT_ENTRY(0a, 1)
PUT_ENTRY(0b, 1)
PUT_ENTRY(0c, 1)
PUT_ENTRY(0d, 1)
PUT_ENTRY(0e, 1)
PUT_ENTRY(0f, 1)
PUT_ENTRY(10, 1)
PUT_ENTRY(11, 1)
PUT_ENTRY(12, 1)
PUT_ENTRY(13, 1)
PUT_ENTRY(14, 1)
PUT_ENTRY(15, 1)
PUT_ENTRY(16, 1)
PUT_ENTRY(17, 1)
PUT_ENTRY(18, 1)
PUT_ENTRY(19, 1)
PUT_ENTRY(1a, 1)
PUT_ENTRY(1b, 1)
PUT_ENTRY(1c, 1)
PUT_ENTRY(1d, 1)
PUT_ENTRY(1e, 1)
PUT_ENTRY(1f, 1)
PUT_ENTRY(20, 1)
PUT_ENTRY(21, 1)
PUT_ENTRY(22, 1)
PUT_ENTRY(23, 1)
PUT_ENTRY(24, 1)
PUT_ENTRY(25, 1)
PUT_ENTRY(26, 1)
PUT_ENTRY(27, 1)
PUT_ENTRY(28, 1)
PUT_ENTRY(29, 1)
PUT_ENTRY(2a, 1)
PUT_ENTRY(2b, 1)
PUT_ENTRY(2c, 1)
PUT_ENTRY(2d, 1)
PUT_ENTRY(2e, 1)
PUT_ENTRY(2f, 1)
PUT_ENTRY(30, 1)
PUT_ENTRY(31, 1)
PUT_ENTRY(32, 1)
PUT_ENTRY(33, 1)
PUT_ENTRY(34, 1)
PUT_ENTRY(35, 1)
PUT_ENTRY(36, 1)
PUT_ENTRY(37, 1)
PUT_ENTRY(38, 1)
PUT_ENTRY(39, 1)
PUT_ENTRY(3a, 1)
PUT_ENTRY(3b, 1)
PUT_ENTRY(3c, 1)
PUT_ENTRY(3d, 1)
PUT_ENTRY(3e, 1)
PUT_ENTRY(3f, 1)
PUT_ENTRY(40, 1)
PUT_ENTRY(41, 1)
PUT_ENTRY(42, 1)
PUT_ENTRY(43, 1)
PUT_ENTRY(44, 1)
PUT_ENTRY(45, 1)
PUT_ENTRY(46, 1)
PUT_ENTRY(47, 1)
PUT_ENTRY(48, 1)
PUT_ENTRY(49, 1)
PUT_ENTRY(4a, 1)
PUT_ENTRY(4b, 1)
PUT_ENTRY(4c, 1)
PUT_ENTRY(4d, 1)
PUT_ENTRY(4e, 1)
PUT_ENTRY(4f, 1)
PUT_ENTRY(50, 1)
PUT_ENTRY(51, 1)
PUT_ENTRY(52, 1)
PUT_ENTRY(53, 1)
PUT_ENTRY(54, 1)
PUT_ENTRY(55, 1)
PUT_ENTRY(56, 1)
PUT_ENTRY(57, 1)
PUT_ENTRY(58, 1)
PUT_ENTRY(59, 1)
PUT_ENTRY(5a, 1)
PUT_ENTRY(5b, 1)
PUT_ENTRY(5c, 1)
PUT_ENTRY(5d, 1)
PUT_ENTRY(5e, 1)
PUT_ENTRY(5f, 1)
PUT_ENTRY(60, 1)
PUT_ENTRY(61, 1)
PUT_ENTRY(62, 1)
PUT_ENTRY(63, 1)
PUT_ENTRY(64, 1)
PUT_ENTRY(65, 1)
PUT_ENTRY(66, 1)
PUT_ENTRY(67, 1)
PUT_ENTRY(68, 1)
PUT_ENTRY(69, 1)
PUT_ENTRY(6a, 1)
PUT_ENTRY(6b, 1)
PUT_ENTRY(6c, 1)
PUT_ENTRY(6d, 1)
PUT_ENTRY(6e, 1)
PUT_ENTRY(6f, 1)
PUT_ENTRY(70, 1)
PUT_ENTRY(71, 1)
PUT_ENTRY(72, 1)
PUT_ENTRY(73, 1)
PUT_ENTRY(74, 1)
PUT_ENTRY(75, 1)
PUT_ENTRY(76, 1)
PUT_ENTRY(77, 1)
PUT_ENTRY(78, 1)
PUT_ENTRY(79, 1)
PUT_ENTRY(7a, 1)
PUT_ENTRY(7b, 1)
PUT_ENTRY(7c, 1)
PUT_ENTRY(7d, 1)
PUT_ENTRY(7e, 1)
PUT_ENTRY(7f, 1)

# ifdef CONFIG_CPU_X86_SYSENTER
.globl x86_interrupt_sys_enter
x86_interrupt_sys_enter:
	pushl	$0x23		; /* user SS */
	pushl	%ebx		; /* stack ptr */
	pushf
	cli
	pushl	$0x1b		; /* user CS */
	pushl	%eax		; /* return address */
	pusha
	mov	0x80, %edi
# endif

1:

#ifdef CONFIG_CPU_USER
	call	x86_seg_restore
#endif
	pushl	%esp				; /* regtable */
	pushl	%edi				; /* number */
	call	%gs: *(cpu_syscall_handler)
	add	$8, %esp
	jmp	x86_interrupt_end

#ifdef CONFIG_CPU_USER

	/* restore kernel mode segment registers */
x86_seg_restore:
	movl	$0x10, %eax	; /* restore kernel mode data segments */
	movl	%eax, %ds
	movl	%eax, %es

# ifdef CONFIG_SMP
	mov	$0x1b, %ecx			; /* get cpu id */
	rdmsr
	and	$0xfffff000, %eax
	mov	0x20(%eax), %eax
	shr	$24, %eax
	movl	cpu_local_storage_seg(,%eax,2), %eax ; /* restore cls segment */
	shl	$3, %eax
	movl	%eax, %fs
# endif
	movl	CLS_SEG (cpu_tls_seg), %eax	; /* restore tls segment */
	movl	%eax, %gs
	ret
#endif

	/* restore user mode segment registers */
x86_interrupt_end:	
#ifdef CONFIG_CPU_USER

	cmpl	$0x8, 36(%esp)	; /* test CS on stack, return to kernel code ? */
	je	1f
	mov	$0x20, %eax
	mov	%eax, %ds
	mov	%eax, %es
	mov	%eax, %fs
	mov	%eax, %gs

# ifdef CONFIG_CPU_X86_SYSENTER
	popa
	popl	%edx		; /* return address */
	add	$4, %esp
	popf
	popl	%ecx		; /* user stack pointer */
	add	$4, %esp
	sysexit
# endif

1:
#endif
	popa
	iret

