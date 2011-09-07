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
        sethi   %hi(\k),          %l6
        or      %l6,    %lo(\k),  %l6
        jmp     %l6
        mov \n, %l7
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

.section .excep,"ax",@progbits

__exception_base_ptr:

      /* reset */
        PUT_ENTRY       0,          cpu_boot

      /* 11 faults 0x00 - 0x0b */
        PUT_ENTRIES     0,    4,    sparc_excep_entry
        PUT_ENTRY       5,          sparc_win_over
        PUT_ENTRY       6,          sparc_win_under
        PUT_ENTRIES     7,    10,   sparc_excep_entry
        PUT_ENTRIES     0x0c, 0x10, sparc_undef_entry

      /* 15 irqs 0x11 - 0x1f */
        PUT_ENTRIES     0,    14,   sparc_irq_entry

      /* 10 faults 0x20 - 0x3c */
        PUT_ENTRIES     11,   12,   sparc_excep_entry /* 0x20, 0x21 */
        PUT_ENTRIES     0x22, 0x23, sparc_undef_entry
        PUT_ENTRIES     13,   14,   sparc_excep_entry /* 0x24, 0x25 */
        PUT_ENTRIES     0x26, 0x27, sparc_undef_entry
        PUT_ENTRIES     15,   19,   sparc_excep_entry /* 0x28 - 0x2c */
        PUT_ENTRIES     0x2d, 0x3b, sparc_undef_entry
        PUT_ENTRY       20,         sparc_excep_entry /* 0x3c */
        PUT_ENTRIES     0x3d, 0x7f, sparc_undef_entry

      /* syscalls */
        PUT_ENTRIES     0x80, 0xff, sparc_syscall_entry

.section .text,"ax",@progbits

sparc_win_over:
sparc_win_under:
sparc_undef_entry:
        rett

sparc_excep_entry:
sparc_irq_entry:
sparc_syscall_entry:
        rett

// Local Variables:
// indent-tabs-mode: nil;
// vim: filetype=cpp:expandtab

