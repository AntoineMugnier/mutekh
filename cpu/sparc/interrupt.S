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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech
*/

#include <hexo/context.h>
#include <hexo/asm.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>

/*
  The exception handling code share the processor state
  array used to save context registers values. When an exception occurs,
  we only save "caller saved" registers before calling the handler.
  When the handler returns we have two possible choices:
  
    * Restore "caller saved" registers and return to interrupted code
    * Complete current context saving with "callee saved" and fpu registers
      and switch to an other context.
  
  The context preemtion handler is used to take the switching decision.
  This preemption handler must be setup by C code during exception handling
  if their is a need to switch context instead of returning.
  
  Doing this allow us to perform as few register saving operations as
  possible because registers are already partially saved for exception
  handling purpose if we decide to switch context.

  Have a look to context_switch.S for more.
*/

.macro PUT_ENTRY n k
        sethi   %hi(\k),          %l4
        or      %l4,    %lo(\k),  %l4
        jmp     %l4
        mov     \n, %l4
.endm

.macro PUT_ENTRIES i j k
        PUT_ENTRY \i, \k
    .if \i+1<=\j
        PUT_ENTRY (\i+1), \k
    .endif
    .if \i+2<=\j
        PUT_ENTRY (\i+2), \k
    .endif
    .if \i+3<=\j
        PUT_ENTRY (\i+3), \k
    .endif
    .if \i+4<=\j
        PUT_ENTRIES (\i+4), \j, \k
    .endif
.endm

.macro PUT_EQ_ENTRIES i j k id
        PUT_ENTRY \id, \k
    .if \i+1<=\j
        PUT_ENTRY \id, \k
    .endif
    .if \i+2<=\j
        PUT_ENTRY \id, \k
    .endif
    .if \i+3<=\j
        PUT_ENTRY \id, \k
    .endif
    .if \i+4<=\j
        PUT_EQ_ENTRIES (\i+4), \j, \k, \id
    .endif
.endm

.section .excep,"ax",@progbits

__exception_base_ptr:

      /* reset 0x00 */
        PUT_ENTRY       0,          cpu_boot

      /* 11 faults 0x01 - 0x0b */
        PUT_ENTRIES     1,    11,   sparc_excep_entry
        PUT_EQ_ENTRIES  0x0c, 0x10, sparc_excep_entry,  0

#ifdef CONFIG_HEXO_IRQ
      /* 15 irqs 0x11 - 0x1f */
        PUT_ENTRIES     0,    14,   sparc_irq_entry
#else
        PUT_EQ_ENTRIES  0,    14,   sparc_excep_entry, 0
#endif
        
      /* 10 faults 0x20 - 0x3c */
        PUT_ENTRIES     12,   13,   sparc_excep_entry /* 0x20, 0x21 */
        PUT_EQ_ENTRIES  0x22, 0x23, sparc_excep_entry, 0
        PUT_ENTRIES     14,   15,   sparc_excep_entry /* 0x24, 0x25 */
        PUT_EQ_ENTRIES  0x26, 0x27, sparc_excep_entry, 0
        PUT_ENTRIES     16,   20,   sparc_excep_entry /* 0x28 - 0x2c */
        PUT_EQ_ENTRIES  0x2d, 0x3b, sparc_excep_entry, 0
        PUT_ENTRY       21,         sparc_excep_entry /* 0x3c */
        PUT_EQ_ENTRIES  0x3d, 0x7f, sparc_excep_entry, 0

#ifdef CONFIG_HEXO_USERMODE
      /* syscalls */
        PUT_ENTRIES     0x80, 0xff, sparc_syscall_entry
#else
        PUT_EQ_ENTRIES  0x80, 0xff, sparc_excep_entry, 0
#endif

__exception_base_ptr_end:
        .globl __exception_base_ptr_end

.section .text,"ax",@progbits

/* This macro use all local regs to save caller saved register before calling C code */
.macro SPARC_EXCEPT_SAVE

#if CONFIG_CPU_SPARC_WINCOUNT < 3
        /* need to save %o* registers to preserve %i* registers in interrupted code */
        add     %g7, %lo(sparc_context_regs), %l0
        
        std     %o0, [%l0 + CPU_SPARC_CONTEXT_I(0)]
        std     %o2, [%l0 + CPU_SPARC_CONTEXT_I(2)]
        std     %o4, [%l0 + CPU_SPARC_CONTEXT_I(4)]
        std     %o6, [%l0 + CPU_SPARC_CONTEXT_I(6)]
#endif

        // preserve other caller saved registers
        mov     %y,     %l0
        mov     %g1,    %l3
        mov     %psr,   %l5
        // %i6 and %i7 are caller saved registers, preserve %o6 and %o7 in interrupted code
        mov     %i6,    %l6
        mov     %i7,    %l7

.endm

/* This macro restore caller saved register from local regs */
.macro SPARC_EXCEPT_RESTORE
        // restore caller saved registers
        mov     %l0,     %y
        mov     %l3,    %g1
        mov     %l5,   %psr
        mov     %l6,    %i6
        mov     %l7,    %i7

#if CONFIG_CPU_SPARC_WINCOUNT < 3
        /* need to restore %o* registers */
        add     %g7, %lo(sparc_context_regs), %l0

        ldd     [%l0 + CPU_SPARC_CONTEXT_I(0)], %o0
        ldd     [%l0 + CPU_SPARC_CONTEXT_I(2)], %o2
        ldd     [%l0 + CPU_SPARC_CONTEXT_I(4)], %o4
        ldd     [%l0 + CPU_SPARC_CONTEXT_I(6)], %o6
#endif
.endm

FUNC_START(.text, sparc_excep_entry)

        SPARC_EXCEPT_SAVE

        // use interrupted code stack
        mov     %i6,    %o6
#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     %sp,    %i7
#endif

        mov     %l4,    %o0

        // context registers save array
        add     %g7, %lo(sparc_context_regs), %l4

        // dump some registers
        st     %l5, [%l4 + CPU_SPARC_CONTEXT_PSR]
        st     %l1, [%l4 + CPU_SPARC_CONTEXT_PC]
        st     %l2, [%l4 + CPU_SPARC_CONTEXT_NPC]

#ifdef CONFIG_ARCH_SMP
        ld      [%g6 + %lo(cpu_exception_handler)], %o5
#else
        set     cpu_exception_handler, %o5
        ld      [%o5], %o5
#endif

        add     %l4, CPU_SPARC_CONTEXT_PC, %o1
        mov     %g0, %o2 /* FIXME data access address ? */
        mov     %l4, %o3
        call    %o5
        mov     %i6, %o4

        // read some registers back in case they were modified by c code
        ld     [%l4 + CPU_SPARC_CONTEXT_PSR], %l5
        ld     [%l4 + CPU_SPARC_CONTEXT_PC] , %l1
        ld     [%l4 + CPU_SPARC_CONTEXT_NPC], %l2

        SPARC_EXCEPT_RESTORE

        jmp     %l1
        rett    %l2
FUNC_END(sparc_excep_entry)


#ifdef CONFIG_HEXO_IRQ
FUNC_START(.text, sparc_irq_entry)

        SPARC_EXCEPT_SAVE

        // use interrupted code stack
        mov     %i6,    %o6
#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     %sp,    %i7
#endif

# ifdef CONFIG_ARCH_SMP
        ld      [%g6 + %lo(cpu_interrupt_handler)], %o1
# else
        set     cpu_interrupt_handler, %o1
        ld      [%o1], %o1
# endif

        call    %o1
        mov     %l4,    %o0

        SPARC_EXCEPT_RESTORE

        jmp     %l1
        rett    %l2

FUNC_END(sparc_irq_entry)
#endif


#ifdef CONFIG_HEXO_USERMODE
FUNC_START(.text, sparc_syscall_entry)

        SPARC_EXCEPT_SAVE
        
        // use interrupted code stack
        mov     %i6,    %o6
#ifdef CONFIG_COMPILE_FRAMEPTR
        mov     %sp,    %i7
#endif

        mov     %l4,    %o0
        
        // context registers save array
        add     %g7, %lo(sparc_context_regs), %l4

        // dump some registers
        st     %g1, [%l4 + CPU_SPARC_CONTEXT_G(1)]
        std    %i0, [%l4 + CPU_SPARC_CONTEXT_O(0)]
        std    %i2, [%l4 + CPU_SPARC_CONTEXT_O(2)]
        std    %i4, [%l4 + CPU_SPARC_CONTEXT_O(4)]
        std    %i6, [%l4 + CPU_SPARC_CONTEXT_O(6)]

        ld      [%g7 + %lo(cpu_syscall_handler)], %o2

        call    %o2
        mov     %l4, %o1

        SPARC_EXCEPT_RESTORE

        // return value
        ld     [%l4 + CPU_SPARC_CONTEXT_G(1)], %g1

        jmp     %l1
        rett    %l2

FUNC_END(sparc_syscall_entry)
#endif


// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

