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

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	8

#define CPU_GPREG_EDI	0
#define CPU_GPREG_ESI	1
#define CPU_GPREG_EBP	2
#define CPU_GPREG_ESP	3
#define CPU_GPREG_EBX	4
#define CPU_GPREG_EDX	5
#define CPU_GPREG_ECX	6
#define CPU_GPREG_EAX	7

#define CPU_X86_EFLAGS_NONE  (1<<1)
#define CPU_X86_EFLAGS_IRQ   (1<<9)

#ifndef __MUTEK_ASM__
		
#include <hexo/interrupt.h>
#include <hexo/iospace.h>
#include <hexo/local.h>

#include "pmode.h"
#include "msr.h"

#define CPU_GPREG_COUNT	8
#define CPU_GPREG_NAMES "edi", "esi", "ebp", "esp", "ebx", "edx", "ecx", "eax"

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  uint64_t	msr;

  asm ("rdmsr\n"
       : "=A" (msr)
       : "c" (0x1b)
       );

  return msr & 0x100 ? 1 : 0;
}

cpu_id_t cpu_id(void);

ALWAYS_INLINE void
cpu_trap()
{
  asm volatile ("int3");
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

#endif  /* __MUTEK_ASM__ */

#endif

