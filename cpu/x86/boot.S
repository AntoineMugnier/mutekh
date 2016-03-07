/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

/* 
 * 16 bits real mode initialisation code is used just after cpu
 * reset. It can be used when code is embedded in a ROM or for
 * booting non master CPUs in SMP systems.
*/

.macro  X86_ENTER_PMODE
.code16
	/* disable interrupts */
	cli
	/* setup current real mode DATA segment */
	xorl	%eax, %eax
	movw	%ax, %ds
	/* write GDT null descriptor */
	movl	$0x00000000,	0x00
	movl	$0x00000000,	0x04
	/* write GDT flat 32 bits data segment descriptor */
	movl	$0x0000ffff,	0x08
	movl	$0x00cf9200,	0x0c
	/* write GDT flat 32 bits code segment descriptor */
	movl	$0x0000ffff,	0x10
	movl	$0x00cf9e00,	0x14
	/* set GDT register */
	movw	$0x0018,	0x1a
	movl	$0x00000000,	0x1c
	lgdt	0x1a
	/* switch to protected mode */
	movl	%cr0,		%eax
	orl	$0x00000001,	%eax /* enable protected mode */
	movl	%eax,		%cr0
	/* jump to (load) 32 bits code segment */
	jmpl	$0x10, $1f
.code32
	/* load 32 bits data segment */
1:	movl	$0x08,		%eax
	movw	%ax,		%ds
	movw	%ax,		%es
	movw	%ax,		%ss
        xorl    %eax,           %eax
	movw	%ax,		%fs
	movw	%ax,		%gs
.endm

#ifdef CONFIG_LOAD_RESET_SEPARATE
.section  .reset,"ax",@progbits
CPU_NAME_DECL(reset_vector):
        .globl CPU_NAME_DECL(reset_vector)
        X86_ENTER_PMODE
        jmp mutekh_entry
#endif

#if defined(CONFIG_ARCH_SMP) && !defined(CONFIG_LOAD_SMP_RESET_MERGED)
.section  .smpreset,"ax",@progbits
CPU_NAME_DECL(smp_reset_vector):
        .globl CPU_NAME_DECL(smp_reset_vector)
        X86_ENTER_PMODE
        jmp mutekh_entry
#endif

FUNC_START(.text, mutekh_entry)
	cli

        /* enable normal cache mode */
	movl	%cr0,		%eax
	andl	$0x9fffffff,	%eax
	movl	%eax,		%cr0

        /* invalidate cls and tls segments */
        xorl    %eax,           %eax
	movw	%ax,		%fs
	movw	%ax,		%gs
        
#ifdef CONFIG_ARCH_SMP_CAPABLE

        /* is bootstrap processor ? */
	mov	$0x1b, %ecx
	rdmsr

	test	$0x100, %eax
	jnz  1f

# ifdef CONFIG_ARCH_SMP
	/* get CPU id */
	andl	$0xfffff000, %eax /* get local APIC base address */
	mov	0x20(%eax), %eax /* read local apic ID */
	shrl	$24, %eax
        xorl    $CPU_TREE_XOR_VALUE, %eax

        /* get cpu tree root */
        movl    cpu_tree, %ebx
        
4:
        /* not in tree */
        test    %ebx, %ebx
        jz      2f

        movl    CPU_TREE_CPU_ID(%ebx), %ecx
        xorl    $CPU_TREE_XOR_VALUE, %ecx
        cmp     %eax, %ecx

        /* matching node */
        je      3f

        /* get next child */
        setb    %cl
        movzx   %cl, %ecx
        dec     %ecx
        movl    CPU_TREE_CHILDS(0)(%ebx, %ecx, 4), %ebx

        jmp     4b
3:
        
        /* get address of the cpu stack */
        movl    CPU_TREE_STACK(%ebx), %esp
        addl    $CONFIG_HEXO_CPU_STACK_SIZE, %esp
#  ifdef CONFIG_COMPILE_FRAMEPTR
        movl    %esp, %ebp
#  endif

        /* jump to C code */
	jmp	mutekh_startup_smp

# endif
2:
	/* stop processor execution */
	cli
	hlt
1:      
#endif

        /* set startup stack pointer */
        movl    $(CONFIG_STARTUP_STACK_ADDR \
                 + CONFIG_STARTUP_STACK_SIZE), %esp

#ifdef CONFIG_COMPILE_FRAMEPTR
        movl    %esp, %ebp
#endif
        /* jump to C code */
	jmp	mutekh_startup

FUNC_END(mutekh_entry)

