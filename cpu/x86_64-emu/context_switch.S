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

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/cpu.h>

.macro GET_TLS_VAR r
#ifdef CONFIG_ARCH_SMP
        movq    __cpu_data_base, \r
        subq    $__cpu_data_start, \r
        addq    $__context_data_base, \r
#else
        leaq    __context_data_base, \r
#endif
.endm

        .extern x86emu_context

FUNC_START(.text, cpu_context_switch)

        GET_TLS_VAR %rcx

        // get context data base
        movq    (%rcx), %rdx
        subq    $__context_data_start, %rdx

        // save stack
        leaq    8(%rsp), %rax
        movq    %rax, (x86emu_context + _offsetof(cpu_context_s, kregs.rsp))(%rdx)

        // eflags
        pushf
        popq    (x86emu_context + _offsetof(cpu_context_s, kregs.rflags))(%rdx)

        // save pc
        movq    (%rsp), %rax
        movq    %rax, (x86emu_context + _offsetof(cpu_context_s, kregs.rip))(%rdx) // return address

        // save "callee saved" registers only
	mov     %rbp, (x86emu_context + _offsetof(cpu_context_s, kregs.rbp))(%rdx)
	mov     %rbx, (x86emu_context + _offsetof(cpu_context_s, kregs.rbx))(%rdx)
	mov     %r12, (x86emu_context + _offsetof(cpu_context_s, kregs.r12))(%rdx)
	mov     %r13, (x86emu_context + _offsetof(cpu_context_s, kregs.r13))(%rdx)
	mov     %r14, (x86emu_context + _offsetof(cpu_context_s, kregs.r14))(%rdx)
	mov     %r15, (x86emu_context + _offsetof(cpu_context_s, kregs.r15))(%rdx)

        jmp     1f

FUNC_END(cpu_context_switch)




FUNC_START(.text, cpu_context_jumpto)

        GET_TLS_VAR %rcx

1:

# ifdef CONFIG_ARCH_SMP
        // unlock some atomic value on context restore if requested
        movq    HEXO_CONTEXT_S_UNLOCK * 8(%rdi), %rax
        test    %rax,   %rax
        jz      1f

        // unlock and clear unlock address
        movq    $0,     HEXO_CONTEXT_S_UNLOCK * 8(%rdi)
        movl    $0,     (%rax)
1:      
# endif

        // get register save array
        movq    HEXO_CONTEXT_S_TLS * 8(%rdi), %rdx

        // switch context data base
        movq    %rdx, (%rcx)
        subq    $__context_data_start, %rdx

        // restore stack pointer
        mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rsp))(%rdx), %rsp

        // prepare restoration
        pushq   (x86emu_context + _offsetof(cpu_context_s, kregs.rip))(%rdx)
        pushq   (x86emu_context + _offsetof(cpu_context_s, kregs.rflags))(%rdx)

        // restore gp regs
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rbp))(%rdx), %rbp
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rbx))(%rdx), %rbx
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rcx))(%rdx), %rcx
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rax))(%rdx), %rax
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rdi))(%rdx), %rdi
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rsi))(%rdx), %rsi
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r8))(%rdx), %r8
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r9))(%rdx), %r9
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r10))(%rdx), %r10
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r11))(%rdx), %r11
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r12))(%rdx), %r12
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r13))(%rdx), %r13
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r14))(%rdx), %r14
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.r15))(%rdx), %r15
	mov     (x86emu_context + _offsetof(cpu_context_s, kregs.rdx))(%rdx), %rdx

        popf
        retq

FUNC_END(cpu_context_jumpto)





FUNC_START(.text, cpu_context_set)

        // setup new stack
        leaq    (%rsi, %rdi), %rsp
        xorq	%rbp, %rbp

        // jump
        jmp	*%rdx

FUNC_END(cpu_context_set)





FUNC_START(.text, cpu_context_stack_use)

        // get register save array
        movq    HEXO_CONTEXT_S_TLS * 8(%rdi), %rax
        subq    $__context_data_start, %rax

        // use context stack
        movq    (x86emu_context + _offsetof(cpu_context_s, kregs.rsp))(%rax), %rsp

#ifndef CONFIG_RELEASE
        // prevent use of context local storage
        GET_TLS_VAR %rax
        movq    $0, (%rax)
#endif

        // jump to function
        movq    %rdx, %rdi
        call    *%rsi

FUNC_END(cpu_context_stack_use)

