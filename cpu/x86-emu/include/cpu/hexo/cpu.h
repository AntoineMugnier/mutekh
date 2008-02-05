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

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

#include <assert.h>

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	8

static cpu_id_t cpu_id(void)
{
#ifdef CONFIG_SMP
# error not supported
#else
  return 0;
#endif  
}

static inline bool_t
cpu_isbootstrap(void)
{
#ifdef CONFIG_SMP
# error not supported
#endif
  return 1;
}

typedef uint64_t cpu_cycle_t;

static inline cpu_cycle_t
cpu_cycle_count(void)
{
  assert(!"not supported"); /* FIXME */
  return 0;
}

static inline
void cpu_trap()
{
  asm volatile ("int3");
}

#endif

