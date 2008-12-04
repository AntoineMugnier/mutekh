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

#if !defined(ALLOC_H_) || defined(ARCH_ALLOC_H_)
#error This file can not be included directly
#else

#define ARCH_ALLOC_H_

extern struct mem_alloc_region_s mem_region_system;

/** system global allocated memory scope */
#define MEM_SCOPE_SYS		(&mem_region_system)

/** cpu local allocated memory scope */
#if defined(CONFIG_SMP)
extern struct mem_alloc_region_s mem_region_cpu;
# define MEM_SCOPE_CPU		(&mem_region_cpu)
#endif

/** cluster local allocated memory scope */
#if defined(CONFIG_CLUSTER)
extern struct mem_alloc_region_s mem_region_cluster;
# define MEM_SCOPE_CLUSTER	(&mem_region_cluster)
#endif

#if defined(CONFIG_CLUSTER)
# error FIXME CONFIG_CLUSTER
#endif

/** allocated memory scope is context local */
#if defined(CONFIG_MUTEK_SCHEDULER_MIGRATION) && !defined(CONFIG_CPU_CACHE_COHERENCY)
# define MEM_SCOPE_CONTEXT	MEM_SCOPE_SYS
#else
# define MEM_SCOPE_CONTEXT	MEM_SCOPE_CPU
#endif

/** use current default allocation policy */
#if defined(CONFIG_SMP) || defined(CONFIG_CLUSTER)
extern struct mem_alloc_region_s *mem_region_default;
#define MEM_SCOPE_DEFAULT	mem_region_default
#endif

/** set default allocation policy */
static inline void
mem_alloc_set_default(struct mem_alloc_region_s *region)
{
#if defined(CONFIG_SMP) || defined(CONFIG_CLUSTER)
  mem_region_default = region;
#endif
}

#endif

