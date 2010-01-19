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

#include <hexo/local.h>

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	8

#define CPU_TYPE_NAME x86

extern CPU_LOCAL cpu_id_t _cpu_id;

static inline cpu_id_t cpu_id(void)
{
#ifdef CONFIG_SMP
    return _cpu_id;
#else
  return 0;
#endif  
}

static inline bool_t
cpu_isbootstrap(void)
{
#ifdef CONFIG_SMP
    return (_cpu_id == 0);
#endif
  return 1;
}

static inline
void cpu_trap()
{
  asm volatile ("int3");
}

typedef uint64_t cpu_cycle_t;

static inline cpu_cycle_t
cpu_cycle_count(void)
{
  cpu_trap(); /* FIXME not supported */
  return 0;
}

#ifdef CONFIG_SMP
extern void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

static inline void *cpu_get_cls(cpu_id_t cpu_id)
{
#ifdef CONFIG_SMP
    return cpu_local_storage[cpu_id];
#endif
  return NULL;
}

static inline void cpu_dcache_invld(void *ptr)
{
#ifndef CONFIG_CPU_CACHE_COHERENCY
# error
#endif
}

static inline size_t cpu_dcache_line_size()
{
  return 0;			/* FIXME */
}

#endif

