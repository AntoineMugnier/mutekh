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

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

struct cpu_context_regs_s
{
  uint32_t edi;
  uint32_t esi;
  uint32_t ebp;
  uint32_t esp;
  uint32_t ebx;
  uint32_t edx;
  uint32_t ecx;
  uint32_t eax;
  uint32_t eip;
  uint32_t eflags;
};

struct cpu_context_s
{
  uint32_t mask;
  /* sorted in iret order */
  union {
    reg_t gpr[10];
    struct cpu_context_regs_s kregs;
  };
# ifdef CONFIG_HEXO_FPU
  __attribute__((aligned(16)))
  uint8_t mm[512];  /* fpu and multimedia state */
# endif
};

/** name of registers accessible using cpu_context_s::gpr */
# define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "eip", "eflags"
/** number of registers in cpu_context_s::gpr */
# define CPU_CONTEXT_REG_COUNT 10

#else

.extern x86emu_context
.equ CPU_X86EMU_CONTEXT_mask,    0

.equ CPU_X86EMU_CONTEXT_edi,     4
.equ CPU_X86EMU_CONTEXT_esi,     8 
.equ CPU_X86EMU_CONTEXT_ebp,     12
.equ CPU_X86EMU_CONTEXT_esp,     16
.equ CPU_X86EMU_CONTEXT_ebx,     20
.equ CPU_X86EMU_CONTEXT_edx,     24
.equ CPU_X86EMU_CONTEXT_ecx,     28
.equ CPU_X86EMU_CONTEXT_eax,     32
.equ CPU_X86EMU_CONTEXT_EIP,     36
.equ CPU_X86EMU_CONTEXT_EFLAGS,  40

.equ CPU_X86EMU_CONTEXT_REGS_OFFSET, 4

.equ CPU_X86EMU_CONTEXT_MM,         48


#endif

