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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#include <cpu/hexo/pmode.h>
#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/cpu.h>
#include <hexo/local.h>

FUNC_START(.text, cpu_context_switch)

#ifdef CONFIG_HEXO_USERMODE
        // context switch happened in kernel mode
        movl    $0, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, mask))
#endif
        
        // save stack
        lea     4(%esp), %eax
        movl    %eax, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.esp))

        // eflags
        pushf
        popl    TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.eflags))

        // save pc
        movl    (%esp), %eax
        movl    %eax, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.eip)) // return address

        // save "callee saved" registers only
        movl    %edi, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.edi))
        movl    %esi, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.esi))
        movl    %ebp, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.ebp))
        movl    %ebx, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, kregs.ebx))

        jmp     cpu_context_jumpto

FUNC_END(cpu_context_switch)




FUNC_START(.text, cpu_context_jumpto)

        // get param
        mov     4(%esp), %edx

# ifdef CONFIG_ARCH_SMP
        // unlock some atomic value on context restore if requested
        movl    _offsetof(context_s, unlock, 4)(%edx), %eax
        test    %eax,   %eax
        jz      1f

        // unlock and clear unlock address
        movl    $0,     _offsetof(context_s, unlock, 4)(%edx)        
        movl    $0,     (%eax)        
1:      
# endif

        // get register save array
        mov     _offsetof(context_s, tls, 4)(%edx), %esi

        // restore tls segment
        mov     context_tls_seg(%esi), TLS_SEG()

#ifdef CONFIG_HEXO_USERMODE
        // jump to user mode ?
        testl	$CPU_X86_CONTEXT_MASK_USER, \
                (x86_context + _offsetof(cpu_context_s, mask))(%esi)
        jz      cpu_context_restore_kernel

        // restore registers from user bank
        addl    $CPU_X86_CONTEXT_USER_SHIFT, %esi
#endif

        .globl cpu_context_restore_user
cpu_context_restore_user:
        
#ifdef CONFIG_HEXO_USERMODE
        // save current tls segment
        movw    TLS_SEG(), CLS_SEG(:) (cpu_tls_seg)

        // prepare iret user stack restoration
        pushl   $CPU_X86_SEGSEL(ARCH_GDT_USER_DATA_INDEX, CPU_X86_USER)
        pushl   (x86_context + _offsetof(cpu_context_s, kregs.esp))(%esi)

        // restore user data segments
        movl	$CPU_X86_SEGSEL(ARCH_GDT_USER_DATA_INDEX, CPU_X86_KERNEL), %eax
        movl    %eax, %ds
        movl    %eax, %es

# ifndef CONFIG_RELEASE
        xorl    %eax, %eax
        movl    %eax, %gs
        movl    %eax, %fs
# endif
        mov     $CPU_X86_SEGSEL(ARCH_GDT_USER_CODE_INDEX, CPU_X86_USER), %eax

        jmp     2f
#endif

        .globl cpu_context_restore_kernel
cpu_context_restore_kernel:
        // restore kernel stack pointer
        mov     (x86_context + _offsetof(cpu_context_s, kregs.esp))(%esi), %esp
        
        mov     $CPU_X86_SEGSEL(ARCH_GDT_CODE_INDEX, CPU_X86_KERNEL), %eax
2:
        // prepare iret eflags/eip restoration
        pushl   (x86_context + _offsetof(cpu_context_s, kregs.eflags))(%esi)
        pushl   %eax
        pushl   (x86_context + _offsetof(cpu_context_s, kregs.eip))(%esi)

        // restore gp regs
        mov     (x86_context + _offsetof(cpu_context_s, kregs.ebp))(%esi), %ebp
        mov     (x86_context + _offsetof(cpu_context_s, kregs.ebx))(%esi), %ebx
        mov     (x86_context + _offsetof(cpu_context_s, kregs.edx))(%esi), %edx
        mov     (x86_context + _offsetof(cpu_context_s, kregs.ecx))(%esi), %ecx
        mov     (x86_context + _offsetof(cpu_context_s, kregs.eax))(%esi), %eax
        mov     (x86_context + _offsetof(cpu_context_s, kregs.edi))(%esi), %edi
        mov     (x86_context + _offsetof(cpu_context_s, kregs.esi))(%esi), %esi

        iret

FUNC_END(cpu_context_jumpto)





FUNC_START(.text, cpu_context_set)

        movl    12(%esp), %eax

        // setup new stack
        movl    4(%esp), %ebx
        addl    8(%esp), %ebx
        movl    %ebx, %esp
        xorl	%ebp, %ebp

        // jump
        jmpl	*%eax

FUNC_END(cpu_context_set)





FUNC_START(.text, cpu_context_stack_use)

        // get params
        mov     4(%esp), %eax
        mov     8(%esp), %ecx
        mov     12(%esp), %edx

        // get register save array
        mov     _offsetof(context_s, tls, 4)(%eax), %eax

        // use context stack
        movl    (x86_context + _offsetof(cpu_context_s, kregs.esp))(%eax), %esp

#ifndef CONFIG_RELEASE
        // prevent use of context local storage
        xorl    %eax, %eax
        mov     %eax, TLS_SEG()
#endif

        // jump to function
        pushl   %edx
        call    *%ecx

FUNC_END(cpu_context_stack_use)



#ifdef CONFIG_HEXO_USERMODE

FUNC_START(.text, cpu_context_set_user)
        
        // get params
        mov     4(%esp), %ecx
        mov     8(%esp), %edx
        mov     12(%esp), %eax

        // push param on user stack
        subl    $CONFIG_HEXO_STACK_ALIGN, %ecx
        movl    %eax, 4(%ecx)

        // restore user data segments
        mov	$CPU_X86_SEGSEL(ARCH_GDT_USER_DATA_INDEX, CPU_X86_KERNEL), %ebx
        mov	%ebx, %ds
        mov	%ebx, %es

        // save current tls segment
        movl    TLS_SEG(), %eax
        movl    %eax, CLS_SEG(:) (cpu_tls_seg)

        movl    $1, TLS_SEG(:) (x86_context + _offsetof(cpu_context_s, mask))

# ifndef CONFIG_RELEASE
        // invalidate cls and tls segments
        xorl    %eax, %eax
        movl    %eax, %gs
        movl    %eax, %fs
# endif

# ifndef CONFIG_CPU_X86_SYSENTER
        // user stack for iret
        pushl   $CPU_X86_SEGSEL(ARCH_GDT_USER_DATA_INDEX, CPU_X86_USER)
        pushl   %ecx
# endif

        // eflags
# ifdef CONFIG_HEXO_IRQ
        pushl   $(CPU_X86_EFLAGS_NONE | CPU_X86_EFLAGS_IRQ)
# else
        pushl   $(CPU_X86_EFLAGS_NONE)
# endif

# ifdef CONFIG_CPU_X86_SYSENTER
//        andl    $~CPU_X86_EFLAGS_IRQ, (%esp)
//        popf
        sti
        sysexit
# else
        pushl   $CPU_X86_SEGSEL(ARCH_GDT_USER_CODE_INDEX, CPU_X86_USER)
        pushl   %edx
        iret
# endif

FUNC_END(cpu_context_set_user)

#endif
