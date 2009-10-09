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


#include <hexo/alloc.h>
#include <hexo/segment.h>

extern __ldscript_symbol_t __system_uncached_heap_start, __system_uncached_heap_end;
extern __ldscript_symbol_t __system_cached_heap_start, __system_cached_heap_end;

struct mem_alloc_region_s mem_region_system;

#if defined(CONFIG_SMP)
struct mem_alloc_region_s mem_region_cpu;
#endif

#if defined(CONFIG_CLUSTER)
struct mem_alloc_region_s mem_region_cluster;
#endif

#if defined(CONFIG_SMP) || defined(CONFIG_CLUSTER)
struct mem_alloc_region_s *mem_region_default;
#endif


void mem_init(void)
{
#if defined(CONFIG_SMP)

#ifdef CONFIG_HEXO_MMU
  uint32_t t = (uint32_t)(&__system_uncached_heap_start);
  mem_alloc_region_init(&mem_region_cpu,
			(void*)t,
			(void *)t
			);
  
  mem_alloc_region_init(&mem_region_system,
			(void *)t,
			(void *)t+CONFIG_SOCLIB_VMEM_MALLOC_REGION_SIZE
			);
  
#else
  mem_alloc_region_init(&mem_region_cpu,
			&__system_cached_heap_start,
			&__system_cached_heap_end
			);

  mem_alloc_region_init(&mem_region_system,
			&__system_uncached_heap_start,
			&__system_uncached_heap_end
			);
#endif

#else

#ifdef CONFIG_HEXO_MMU
  uint32_t t = (uint32_t)(&__system_uncached_heap_start);
  mem_alloc_region_init(&mem_region_system,
			(void*)t,
			(void *)t+CONFIG_SOCLIB_VMEM_MALLOC_REGION_SIZE
			);
  
#else
  mem_alloc_region_init(&mem_region_system,
			&__system_cached_heap_start,
			&__system_cached_heap_end
			);
#endif

#endif

#if defined(CONFIG_SMP)
  mem_alloc_set_default(&mem_region_system);
#endif

#if defined(CONFIG_CLUSTER)
# error FIXME CONFIG_CLUSTER
#endif
}

