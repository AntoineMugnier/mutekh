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

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	8

static inline cpu_id_t cpu_id(void)
{
#ifdef CONFIG_SMP
  cpu_trap();			/* not supported */
#else
  return 0;
#endif  
}

static inline bool_t
cpu_isbootstrap(void)
{
#ifdef CONFIG_SMP
  cpu_trap();			/* not supported */
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

struct cpu_cld_s
{
#ifdef CONFIG_SMP
# error not supported
#endif
  /* CPU id */
  uint32_t	id;
  /* PID of the worker unix process */
  int32_t	worker_pid;
  /* PID of the unix process used to perform ptrace ops */
  int32_t	tracer_pid;
};

extern struct cpu_cld_s	*cpu_cld_list[CONFIG_CPU_MAXCOUNT];

static inline void *cpu_get_cls(cpu_id_t cpu_id)
{
#ifdef CONFIG_SMP
# error not supported
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

