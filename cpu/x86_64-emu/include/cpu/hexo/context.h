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
  uint64_t rdi;
  uint64_t rsi;
  uint64_t rbp;
  uint64_t rsp;
  uint64_t rbx;
  uint64_t rdx;
  uint64_t rcx;
  uint64_t rax;
  uint64_t r8;
  uint64_t r9;
  uint64_t r10;
  uint64_t r11;
  uint64_t r12;
  uint64_t r13;
  uint64_t r14;
  uint64_t r15;
  uint64_t rip;
  uint64_t rflags;
};

struct cpu_context_s
{
  uint64_t mask;
  /* sorted in iret order */
  union {
    reg_t gpr[18];
    struct cpu_context_regs_s kregs;
  };
# ifdef CONFIG_HEXO_FPU
  __attribute__((aligned(16)))
  uint8_t mm[512];  /* fpu and multimedia state */
# endif
};

/** name of registers accessible using cpu_context_s::gpr */
# define CPU_CONTEXT_REG_NAMES CPU_GPREG_NAMES, "rip", "rflags"
/** number of registers in cpu_context_s::gpr */
# define CPU_CONTEXT_REG_COUNT 18

#endif

