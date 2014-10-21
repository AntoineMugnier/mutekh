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

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

#include <hexo/local.h>

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	8
#define CPU_GPREG_NAMES "edi", "esi", "ebp", "esp", "ebx", "edx", "ecx", "eax"


#ifndef __MUTEK_ASM__

#include <arch/hexo/emu_syscalls.h>

ALWAYS_INLINE cpu_id_t cpu_id(void)
{
  return emu_do_syscall(EMU_SYSCALL_GETPID, 0);
}

extern __compiler_sint_t __bootstrap_pid;

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
#ifdef CONFIG_ARCH_SMP
  return (cpu_id() == __bootstrap_pid);
#endif
  return 1;
}

ALWAYS_INLINE void cpu_trap()
{
#ifdef CONFIG_ARCH_EMU_TRAP_KILL
  /* kill process group */
  if (__bootstrap_pid > 1)
    emu_do_syscall(EMU_SYSCALL_KILL, 2, -__bootstrap_pid, EMU_SIG_TERM);
  emu_do_syscall(EMU_SYSCALL_EXIT, 1, 0);
#else
  asm volatile ("int3");
#endif
}

ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
#ifndef CONFIG_CPU_CACHE_COHERENCY
# error
#endif
}

ALWAYS_INLINE size_t cpu_dcache_line_size()
{
  return 0;			/* FIXME */
}

#endif
#endif


